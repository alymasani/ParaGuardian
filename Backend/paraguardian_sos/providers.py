"""Messaging provider adapters.

Outcome of one send attempt (SendResult.outcome):
  accepted      the provider API accepted the message for sending (NOT delivered to a person)
  rejected      the provider refused it permanently (bad number, auth, policy): do not retry
  retry         temporary refusal (HTTP 429 / 5xx): safe to retry, nothing was sent
  unknown       the request may or may not have been processed (timeout, connection lost after
                sending, unparseable answer): NEVER retried automatically -> no duplicate messages
Delivery itself is learned only from provider status callbacks ("delivered" / "undelivered" / "failed"),
and even "delivered" means the carrier reported handset delivery, not that a human read it.
"""
import base64
import hashlib
import hmac
import json
import socket
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass


@dataclass
class SendResult:
    outcome: str
    provider_id: str = ""
    detail: str = ""


class FakeProvider:
    """Deterministic, offline. Records every message instead of sending it. Used by tests and local dev."""

    name = "fake"

    def __init__(self):
        self.outbox = []        # (to, body, provider_id)
        self.script = []        # queue of outcomes to return next ("accepted" default)
        self.counter = 0

    def send(self, to, body, status_callback=""):
        self.counter += 1
        outcome = self.script.pop(0) if self.script else "accepted"
        pid = "FAKE%06d" % self.counter
        if outcome == "accepted":
            self.outbox.append((to, body, pid))
            return SendResult("accepted", pid)
        if outcome == "unknown_after_send":   # the provider got it, the answer was lost
            self.outbox.append((to, body, pid))
            return SendResult("unknown", "", "timeout after request was sent")
        return SendResult(outcome, "", "scripted " + outcome)


class DisabledProvider:
    """Real delivery not configured / not enabled: refuses everything, sends nothing."""

    name = "disabled"

    def send(self, to, body, status_callback=""):
        return SendResult("rejected", "", "live delivery is not enabled")


class TwilioProvider:
    """Twilio Programmable Messaging REST adapter (https://api.twilio.com/2010-04-01).

    Real request construction, Basic authentication, response parsing, timeout and error mapping.
    It is only constructed when Config.live_ready() is true (explicit switch + credentials).
    `opener` is injectable so tests exercise the exact HTTP request without any network access.
    """

    name = "twilio"
    API = "https://api.twilio.com/2010-04-01/Accounts/%s/Messages.json"

    def __init__(self, account_sid, auth_token, from_number, timeout_s=10.0, opener=None):
        self.sid = account_sid
        self.token = auth_token
        self.from_number = from_number
        self.timeout = timeout_s
        self.opener = opener or urllib.request.urlopen

    def build_request(self, to, body, status_callback=""):
        fields = {"To": to, "From": self.from_number, "Body": body}
        if status_callback:
            fields["StatusCallback"] = status_callback
        data = urllib.parse.urlencode(fields).encode("utf-8")
        req = urllib.request.Request(self.API % urllib.parse.quote(self.sid, safe=""), data=data, method="POST")
        cred = base64.b64encode(("%s:%s" % (self.sid, self.token)).encode("utf-8")).decode("ascii")
        req.add_header("Authorization", "Basic " + cred)
        req.add_header("Content-Type", "application/x-www-form-urlencoded")
        req.add_header("Accept", "application/json")
        return req

    def send(self, to, body, status_callback=""):
        req = self.build_request(to, body, status_callback)
        try:
            with self.opener(req, timeout=self.timeout) as resp:
                raw = resp.read(65536)
                status = resp.status
        except urllib.error.HTTPError as e:
            code = e.code
            if code == 429 or 500 <= code < 600:
                return SendResult("retry", "", "HTTP %d" % code)
            return SendResult("rejected", "", "HTTP %d" % code)
        except (socket.timeout, TimeoutError):
            return SendResult("unknown", "", "timeout")
        except urllib.error.URLError as e:
            # Connection refused / DNS: nothing reached the provider -> retry is safe.
            reason = getattr(e, "reason", None)
            if isinstance(reason, (socket.timeout, TimeoutError)):
                return SendResult("unknown", "", "timeout")
            return SendResult("retry", "", "connection error")
        try:
            doc = json.loads(raw.decode("utf-8"))
            sid = str(doc.get("sid", ""))
        except (ValueError, UnicodeDecodeError, AttributeError):
            return SendResult("unknown", "", "unparseable provider answer")
        if 200 <= status < 300 and sid.startswith("SM"):
            return SendResult("accepted", sid, str(doc.get("status", "")))
        return SendResult("unknown", "", "unexpected provider answer")


def twilio_signature(auth_token, url, params):
    """X-Twilio-Signature: base64(HMAC-SHA1(token, url + concatenated sorted key+value))."""
    s = url + "".join(k + params[k] for k in sorted(params))
    return base64.b64encode(hmac.new(auth_token.encode("utf-8"), s.encode("utf-8"), hashlib.sha1).digest()).decode()


# Provider callback status -> job state. Anything else (queued, sending, sent...) keeps the job as it is.
CALLBACK_STATES = {"delivered": "delivered", "undelivered": "failed", "failed": "failed"}
