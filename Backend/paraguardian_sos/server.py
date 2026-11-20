"""HTTP(S) API. Thin layer: size limits, JSON parsing, authentication, routing, privacy-conscious logs.

  POST /v1/contacts                        (signed)  set consented contacts (max 5, E.164)
  POST /v1/incidents                       (signed)  submit / update / cancel an incident (idempotent)
  GET  /v1/incidents/<key>                 (signed)  delivery status
  POST /v1/incidents/<key>/retry-unknown   (signed)  explicit re-send of jobs whose outcome is unknown
  POST /v1/provider/twilio/status          (Twilio signature)  delivery status callbacks
  POST /v1/provider/fake/status            (fake provider only) simulated callbacks for local tests
  GET  /v1/health                          aggregate diagnostics (no personal data)
TLS: if PG_TLS_CERT / PG_TLS_KEY are set the socket is wrapped with TLS 1.2+. Without them the server
refuses to bind anything but loopback (local development / tests only).
"""
import json
import logging
import ssl
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from . import auth
from .providers import twilio_signature
from .service import ApiError

log = logging.getLogger("paraguardian_sos")
LOOPBACK = ("127.0.0.1", "::1", "localhost")


def make_handler(service, cfg):
    class Handler(BaseHTTPRequestHandler):
        server_version = "ParaGuardianSOS/0.8"
        sys_version = ""

        def log_message(self, fmt, *args):   # no client addresses, paths only (incident keys are opaque)
            log.info("%s %s", self.command, self.path.split("?")[0])

        def _send(self, status, doc):
            raw = json.dumps(doc, separators=(",", ":")).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(raw)

        def _body(self):
            n = self.headers.get("Content-Length")
            if n is None or not n.isdigit():
                raise ApiError(411, "length_required", "Content-Length required")
            if int(n) > cfg.max_body_bytes:
                raise ApiError(413, "too_large", "request body too large")
            return self.rfile.read(int(n))

        def _auth(self, body):
            path = self.path.split("?")[0]
            try:
                device = auth.verify(service.store, cfg, self.command, path, self.headers, body)
            except auth.AuthError as e:
                raise ApiError(e.status, "unauthorized", "authentication failed")   # one answer for all causes
            service.check_rate(device)
            return device

        def _json(self, body):
            try:
                return json.loads(body.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                raise ApiError(400, "invalid_json", "body is not JSON")

        def do_GET(self):
            self._dispatch()

        def do_POST(self):
            self._dispatch()

        def _dispatch(self):
            try:
                path = self.path.split("?")[0]
                parts = [p for p in path.split("/") if p]
                if self.command == "GET" and path == "/v1/health":
                    return self._send(200, service.diagnostics())
                body = self._body() if self.command == "POST" else b""
                if self.command == "POST" and path == "/v1/contacts":
                    device = self._auth(body)
                    return self._send(200, service.set_contacts(device, self._json(body).get("contacts")))
                if self.command == "POST" and path == "/v1/incidents":
                    device = self._auth(body)
                    status, doc = service.submit(device, self._json(body))
                    return self._send(status, doc)
                if len(parts) == 3 and parts[:2] == ["v1", "incidents"] and self.command == "GET":
                    device = self._auth(body)
                    return self._send(200, service.status(device, parts[2]))
                if len(parts) == 4 and parts[:2] == ["v1", "incidents"] and parts[3] == "retry-unknown" \
                        and self.command == "POST":
                    device = self._auth(body)
                    return self._send(200, service.retry_unknown(device, parts[2]))
                if self.command == "POST" and path == "/v1/provider/twilio/status":
                    return self._twilio_callback(body)
                if self.command == "POST" and path == "/v1/provider/fake/status" and service.provider.name == "fake":
                    doc = self._json(body)
                    return self._send(200, {"result": service.handle_callback(str(doc.get("id", "")),
                                                                              str(doc.get("status", "")))})
                raise ApiError(404, "not_found", "no such endpoint")
            except ApiError as e:
                self._send(e.status, {"error": e.code, "message": e.message})
            except Exception:   # never leak internals
                log.exception("internal error")
                self._send(500, {"error": "internal", "message": "internal error"})

        def _twilio_callback(self, body):
            if service.provider.name != "twilio" or not cfg.public_base_url:
                raise ApiError(404, "not_found", "no such endpoint")
            params = {k: v[0] for k, v in urllib.parse.parse_qs(body.decode("utf-8", "replace")).items()}
            url = cfg.public_base_url.rstrip("/") + "/v1/provider/twilio/status"
            expected = twilio_signature(cfg.twilio_auth_token, url, params)
            if not auth.hmac.compare_digest(expected, self.headers.get("X-Twilio-Signature", "")):
                raise ApiError(403, "forbidden", "bad provider signature")
            result = service.handle_callback(params.get("MessageSid", ""), params.get("MessageStatus", ""))
            return self._send(200, {"result": result})

    return Handler


def build_server(service, cfg):
    tls = bool(cfg.tls_cert and cfg.tls_key)
    if not tls and cfg.host not in LOOPBACK:
        raise SystemExit("refusing to serve without TLS on a non-loopback address (set PG_TLS_CERT / PG_TLS_KEY)")
    httpd = ThreadingHTTPServer((cfg.host, cfg.port), make_handler(service, cfg))
    if tls:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.minimum_version = ssl.TLSVersion.TLSv1_2
        ctx.load_cert_chain(cfg.tls_cert, cfg.tls_key)
        httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)
    return httpd


class Worker(threading.Thread):
    """Background delivery worker (one per process is enough; leases make several safe)."""

    def __init__(self, service, idle_s=0.5):
        super().__init__(daemon=True)
        self.service = service
        self.idle_s = idle_s
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            try:
                busy = self.service.worker_step()
            except Exception:
                log.exception("worker step failed")
                busy = False
            if not busy:
                self.stop.wait(self.idle_s)
