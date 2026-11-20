"""Backend tests. No network access beyond 127.0.0.1; no real provider is ever contacted.

Run:  python3 -m unittest discover -s Backend/tests -t Backend      (from the repository root)
"""
import http.client
import io
import json
import os
import socket
import tempfile
import threading
import unittest
import urllib.error

from paraguardian_sos import auth
from paraguardian_sos.config import Config
from paraguardian_sos.providers import DisabledProvider, FakeProvider, TwilioProvider, twilio_signature
from paraguardian_sos.server import build_server
from paraguardian_sos.service import ApiError, EmergencyService
from paraguardian_sos.store import Store


class Clock:
    def __init__(self, t=1_800_000_000.0):
        self.t = t

    def __call__(self):
        return self.t


def alert(key="7", rev=1, kind="alert", **kw):
    d = {"incidentKey": key, "incidentId": int(key.split("-")[0]) if key.split("-")[0].isdigit() else 1,
         "revision": rev, "kind": kind, "trigger": 1, "test": False,
         "position": {"lat": 46.5, "lon": 7.9, "accuracyM": 8.0, "ageMs": 4000, "source": "gnss"},
         "batteryMv": 3700, "identityVolatile": False, "restored": False}
    d.update(kw)
    return d


class Base(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.cfg = Config(db_path=os.path.join(self.dir.name, "t.db"))
        self.store = Store(self.cfg.db_path, clock=self.clock)
        self.fake = FakeProvider()
        self.svc = EmergencyService(self.cfg, self.store, self.fake)
        self.secret = self.svc.provision_device("pg-dev-1")
        self.svc.set_contacts("pg-dev-1", [{"name": "Ana", "e164": "+41790000001", "consent": True},
                                           {"name": "Ben", "e164": "+41790000002", "consent": True}])

    def tearDown(self):
        self.store.close()
        self.dir.cleanup()

    def drain(self, n=50):
        for _ in range(n):
            if not self.svc.worker_step():
                break


class IdempotencyTests(Base):
    def test_duplicate_submissions_and_lost_responses_send_each_message_once(self):
        st, doc = self.svc.submit("pg-dev-1", alert())
        self.assertEqual(st, 202)
        for _ in range(5):   # Android retry after a lost HTTP response / MCU re-send of revision 1
            st, doc = self.svc.submit("pg-dev-1", alert())
            self.assertEqual(doc["result"], "duplicate")
        self.drain()
        self.assertEqual(len(self.fake.outbox), 2)                 # one per contact, never more
        st, doc = self.svc.submit("pg-dev-1", alert(rev=2))       # MCU reset: same incident, newer revision
        self.assertEqual(doc["result"], "updated")
        self.drain()
        self.assertEqual(len(self.fake.outbox), 2)
        self.assertEqual(self.svc.status("pg-dev-1", "7")["overall"], "provider_accepted")   # not "delivered"

    def test_provider_accepted_is_not_delivery_and_callbacks_are_idempotent(self):
        self.svc.submit("pg-dev-1", alert())
        self.drain()
        pid = self.fake.outbox[0][2]
        self.assertEqual(self.svc.handle_callback(pid, "sent"), "ignored")
        self.assertEqual(self.svc.handle_callback(pid, "delivered"), "applied")
        self.assertEqual(self.svc.handle_callback(pid, "delivered"), "duplicate")
        self.assertEqual(self.svc.handle_callback(pid, "undelivered"), "ignored")   # never downgraded
        self.assertEqual(self.svc.status("pg-dev-1", "7")["overall"], "delivered")
        self.assertEqual(self.svc.handle_callback("SMunknown", "delivered"), "unknown_message")

    def test_provider_timeout_after_send_is_unknown_and_never_resent_automatically(self):
        self.fake.script = ["unknown_after_send", "accepted"]
        self.svc.submit("pg-dev-1", alert())
        self.drain()
        st = self.svc.status("pg-dev-1", "7")
        self.assertEqual(sorted(j["state"] for j in st["jobs"]), ["provider_accepted", "unknown"])
        self.clock.t += 3600
        self.drain()
        self.assertEqual(len(self.fake.outbox), 2)   # the timed-out message is not repeated
        st = self.svc.retry_unknown("pg-dev-1", "7")  # only an explicit human decision re-sends it
        self.assertEqual(st["requeued"], 1)
        self.drain()
        self.assertEqual(len(self.fake.outbox), 3)

    def test_worker_crash_during_send_becomes_unknown_after_lease(self):
        self.svc.submit("pg-dev-1", alert())
        # simulate a worker that claimed the job and died before recording the result
        with self.store.tx() as db:
            db.execute("UPDATE jobs SET state='sending', lease_until=? WHERE contact_id='C1'", (self.clock.t + 60,))
        self.clock.t += 61
        self.drain()
        states = {j["contact"]: j["state"] for j in self.svc.status("pg-dev-1", "7")["jobs"]}
        self.assertEqual(states["C1"], "unknown")
        self.assertEqual(states["C2"], "provider_accepted")
        self.assertEqual(len(self.fake.outbox), 1)

    def test_temporary_provider_errors_retry_with_backoff_then_fail(self):
        self.fake.script = ["retry", "retry", "accepted"]
        self.svc.set_contacts("pg-dev-1", [{"name": "Ana", "e164": "+41790000001", "consent": True}])
        self.svc.submit("pg-dev-1", alert())
        self.drain()
        self.assertEqual(self.svc.status("pg-dev-1", "7")["jobs"][0]["state"], "queued")   # waiting for backoff
        self.clock.t += 5
        self.drain()
        self.clock.t += 10
        self.drain()
        self.assertEqual(self.svc.status("pg-dev-1", "7")["overall"], "provider_accepted")
        self.fake.script = ["retry"] * 10
        self.svc.submit("pg-dev-1", alert(key="8"))
        for _ in range(10):
            self.clock.t += 1000
            self.drain()
        j = self.svc.status("pg-dev-1", "8")["jobs"][0]
        self.assertEqual((j["state"], j["attempts"]), ("failed", self.cfg.max_attempts))

    def test_cancel_before_dispatch_and_after_send(self):
        self.svc.submit("pg-dev-1", alert(key="9"))
        st, doc = self.svc.submit("pg-dev-1", alert(key="9", rev=2, kind="cancel"))
        self.assertEqual(doc["result"], "cancelled_before_dispatch")
        self.drain()
        self.assertEqual(len(self.fake.outbox), 0)
        self.assertEqual(doc["overall"], "cancelled")
        self.svc.submit("pg-dev-1", alert(key="10"))
        self.drain()
        st, doc = self.svc.submit("pg-dev-1", alert(key="10", rev=2, kind="cancel"))
        self.assertEqual(doc["result"], "cancelled_notices_queued")
        self.drain()
        self.assertEqual(len(self.fake.outbox), 4)
        self.assertIn("CANCELLED", self.fake.outbox[-1][1])
        st, doc = self.svc.submit("pg-dev-1", alert(key="10", rev=3))   # alert after cancel: ignored
        self.assertEqual(doc["result"], "already_cancelled")
        self.drain()
        self.assertEqual(len(self.fake.outbox), 4)

    def test_test_incidents_never_use_the_live_provider(self):
        live = FakeProvider()
        live.name = "twilio"   # stands in for a live adapter
        svc = EmergencyService(self.cfg, self.store, live)
        svc.submit("pg-dev-1", alert(key="11", test=True))
        for _ in range(5):
            svc.worker_step()
        self.assertEqual(len(live.outbox), 0)
        self.assertEqual(len(svc.test_provider.outbox), 2)
        self.assertTrue(svc.test_provider.outbox[0][1].startswith("TEST ONLY"))

    def test_volatile_identity_keys_are_separate_incidents(self):
        self.svc.submit("pg-dev-1", alert(key="1-v1700000000000", identityVolatile=True))
        self.svc.submit("pg-dev-1", alert(key="1-v1700000999000", identityVolatile=True))
        self.drain()
        self.assertEqual(len(self.fake.outbox), 4)


class ValidationTests(Base):
    def test_rejects_malformed_and_out_of_range_input(self):
        bad = [alert(incidentId=0), alert(revision=70000), alert(kind="hack"), alert(trigger=9),
               alert(position={"lat": 91, "lon": 0}), alert(position={"lat": 1}), alert(extra=1),
               alert(batteryMv=-1), alert(incidentKey="../etc"), alert(test="yes"), "not a dict"]
        for b in bad:
            with self.assertRaises(ApiError):
                self.svc.submit("pg-dev-1", b)

    def test_contacts_need_consent_valid_numbers_and_a_limit(self):
        for contacts in ([{"name": "A", "e164": "+41790000001", "consent": False}],
                         [{"name": "A", "e164": "0790000001", "consent": True}],
                         [{"name": "A", "e164": "+41790000001", "consent": True}] * 2,
                         [{"name": "A%d" % i, "e164": "+4179000000%d" % i, "consent": True} for i in range(6)]):
            with self.assertRaises(ApiError):
                self.svc.set_contacts("pg-dev-1", contacts)

    def test_rate_limits(self):
        for i in range(self.cfg.new_incidents_per_hour):
            self.svc.submit("pg-dev-1", alert(key=str(100 + i)))
        with self.assertRaises(ApiError) as e:
            self.svc.submit("pg-dev-1", alert(key="200"))
        self.assertEqual(e.exception.status, 429)
        for _ in range(self.cfg.requests_per_minute):
            self.svc.check_rate("pg-dev-2")
        with self.assertRaises(ApiError):
            self.svc.check_rate("pg-dev-2")
        self.clock.t += 60
        self.svc.check_rate("pg-dev-2")

    def test_purge_minimises_location_data(self):
        self.svc.submit("pg-dev-1", alert())
        self.clock.t += 31 * 86400
        self.assertEqual(self.svc.purge(), 1)
        with self.store.tx() as db:
            row = db.execute("SELECT lat, lon FROM incidents").fetchone()
        self.assertIsNone(row["lat"])


class ProviderTests(unittest.TestCase):
    def test_twilio_request_construction_and_response_mapping_without_network(self):
        sent = {}

        class Resp:
            status = 201

            def __init__(self, body):
                self.body = body

            def read(self, n):
                return self.body

            def __enter__(self):
                return self

            def __exit__(self, *a):
                return False

        def opener(req, timeout):
            sent["url"], sent["data"], sent["auth"], sent["timeout"] = (req.full_url, req.data,
                                                                         req.get_header("Authorization"), timeout)
            return Resp(b'{"sid":"SM0123","status":"queued"}')

        p = TwilioProvider("AC_test", "tok", "+15550000000", 7.0, opener)
        r = p.send("+41790000001", "hello", "https://example.invalid/cb")
        self.assertEqual((r.outcome, r.provider_id), ("accepted", "SM0123"))
        self.assertEqual(sent["url"], "https://api.twilio.com/2010-04-01/Accounts/AC_test/Messages.json")
        self.assertIn(b"To=%2B41790000001", sent["data"])
        self.assertIn(b"StatusCallback=", sent["data"])
        self.assertTrue(sent["auth"].startswith("Basic "))
        self.assertEqual(sent["timeout"], 7.0)

        def raises(exc):
            def f(req, timeout):
                raise exc
            return f
        hdr = io.BytesIO(b"")
        cases = [(urllib.error.HTTPError("u", 400, "bad", {}, hdr), "rejected"),
                 (urllib.error.HTTPError("u", 429, "slow", {}, hdr), "retry"),
                 (urllib.error.HTTPError("u", 503, "down", {}, hdr), "retry"),
                 (socket.timeout(), "unknown"),
                 (urllib.error.URLError(ConnectionRefusedError()), "retry"),
                 (urllib.error.URLError(socket.timeout()), "unknown")]
        for exc, outcome in cases:
            self.assertEqual(TwilioProvider("AC", "t", "+1", 1.0, raises(exc)).send("+1", "x").outcome, outcome)
        self.assertEqual(TwilioProvider("AC", "t", "+1", 1.0, lambda r, timeout: Resp(b"<html>")).send("+1", "x").outcome,
                         "unknown")

    def test_live_delivery_needs_explicit_switch_and_credentials(self):
        c = Config(provider="twilio", twilio_account_sid="AC", twilio_auth_token="t", twilio_from="+1")
        self.assertFalse(c.live_ready())
        c.live_delivery_enabled = True
        self.assertTrue(c.live_ready())
        self.assertEqual(DisabledProvider().send("+1", "x").outcome, "rejected")

    def test_twilio_signature_known_vector(self):
        # Example from Twilio's webhook-security documentation (token 12345).
        url = "https://mycompany.com/myapp.php?foo=1&bar=2"
        params = {"CallSid": "CA1234567890ABCDE", "Caller": "+12349013030", "Digits": "1234", "From": "+12349013030",
                  "To": "+18005551212"}
        self.assertEqual(twilio_signature("12345", url, params), "0/KCTR6DLpKmkAf8muzZqo1nDgQ=")


class HttpEndToEndTests(Base):
    """Real HTTP over 127.0.0.1, real authentication, real service + fake provider."""

    def setUp(self):
        super().setUp()
        self.cfg.port = 0
        self.httpd = build_server(self.svc, self.cfg)
        self.port = self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.n = 0

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        super().tearDown()

    def call(self, method, path, doc=None, secret=None, device="pg-dev-1", nonce=None, ts=None, tamper=False):
        body = b"" if doc is None else json.dumps(doc).encode()
        self.n += 1
        nonce = nonce or ("nonce%016d" % self.n)
        ts = int(self.clock.t) if ts is None else ts
        sig = auth.sign(secret or self.secret, method, path, ts, nonce, body)
        if tamper:
            body = body.replace(b"46.5", b"46.6")
        c = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        c.request(method, path, body=body if method == "POST" else None,
                  headers={"X-PG-Device": device, "X-PG-Timestamp": str(ts), "X-PG-Nonce": nonce,
                           "X-PG-Signature": sig, "Content-Type": "application/json"})
        r = c.getresponse()
        out = (r.status, json.loads(r.read() or b"{}"))
        c.close()
        return out

    def test_end_to_end_submit_status_callback(self):
        st, doc = self.call("POST", "/v1/incidents", alert(key="42"))
        self.assertEqual((st, doc["overall"]), (202, "pending"))
        st, doc = self.call("POST", "/v1/incidents", alert(key="42"))   # retried after a "lost" response
        self.assertEqual((st, doc["result"]), (200, "duplicate"))
        self.drain()
        st, doc = self.call("GET", "/v1/incidents/42")
        self.assertEqual(doc["overall"], "provider_accepted")
        self.assertEqual(doc["provider"], "fake")
        self.assertFalse(doc["liveDelivery"])
        c = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        cb = json.dumps({"id": self.fake.outbox[0][2], "status": "delivered"}).encode()
        c.request("POST", "/v1/provider/fake/status", body=cb, headers={"Content-Length": str(len(cb))})
        self.assertEqual(json.loads(c.getresponse().read())["result"], "applied")
        st, doc = self.call("GET", "/v1/incidents/42")
        self.assertEqual(doc["overall"], "delivered")
        self.assertEqual(len(self.fake.outbox), 2)

    def test_authentication_replay_skew_tamper_and_size(self):
        self.assertEqual(self.call("POST", "/v1/incidents", alert(), secret=b"x" * 32)[0], 401)
        self.assertEqual(self.call("POST", "/v1/incidents", alert(), device="pg-unknown")[0], 401)
        self.assertEqual(self.call("POST", "/v1/incidents", alert(), ts=int(self.clock.t) - 1000)[0], 401)
        self.assertEqual(self.call("POST", "/v1/incidents", alert(), tamper=True)[0], 401)
        self.assertEqual(self.call("POST", "/v1/incidents", alert(key="5"), nonce="replaynonce0000001")[0], 202)
        self.assertEqual(self.call("POST", "/v1/incidents", alert(key="6"), nonce="replaynonce0000001")[0], 401)
        st, doc = self.call("POST", "/v1/incidents", {"incidentKey": "x" * 5000})
        self.assertEqual(st, 413)
        self.assertEqual(self.call("GET", "/v1/incidents/does-not-exist")[0], 404)
        c = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        c.request("GET", "/v1/health")
        health = json.loads(c.getresponse().read())
        self.assertNotIn("+4179", json.dumps(health))   # aggregate only, no personal data

    def test_refuses_plain_http_on_non_loopback(self):
        cfg = Config(db_path=self.cfg.db_path, host="0.0.0.0", port=0)
        with self.assertRaises(SystemExit):
            build_server(self.svc, cfg)


if __name__ == "__main__":
    unittest.main()
