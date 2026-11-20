"""Request authentication: per-device HMAC-SHA256 signatures with timestamp + nonce.

Canonical string (UTF-8):  METHOD \\n PATH \\n TIMESTAMP \\n NONCE \\n hex(SHA-256(body))
Headers: X-PG-Device, X-PG-Timestamp (unix seconds), X-PG-Nonce (16-64 chars), X-PG-Signature (hex).

- The device id alone authorises nothing: a request must be signed with that device's secret.
- Replay resistance: timestamps outside +-clock_skew_s are rejected; a nonce is accepted once per device
  within the window (stored in SQLite, pruned after the window).
- The secret is provisioned out of band (python -m paraguardian_sos provision-device) and stored in the
  phone's Android Keystore-backed storage. It is a per-device credential, not a shared app secret; it is
  never placed in source, Gradle files, APK resources or logs.
- Limitation (documented): the server must store the secret to verify HMACs. A production deployment
  should keep it in a secrets manager / KMS and use TLS everywhere (the signature does not encrypt).
"""
import hashlib
import hmac
import re
import secrets

NONCE_RE = re.compile(r"^[A-Za-z0-9_-]{16,64}$")
DEVICE_RE = re.compile(r"^[A-Za-z0-9_.:-]{3,64}$")


def body_hash(body):
    return hashlib.sha256(body).hexdigest()


def canonical(method, path, ts, nonce, body):
    return "\n".join([method.upper(), path, str(ts), nonce, body_hash(body)]).encode("utf-8")


def sign(secret, method, path, ts, nonce, body):
    return hmac.new(secret, canonical(method, path, ts, nonce, body), hashlib.sha256).hexdigest()


def new_secret():
    return secrets.token_bytes(32)


class AuthError(Exception):
    def __init__(self, status, reason):
        super().__init__(reason)
        self.status = status
        self.reason = reason


def verify(store, cfg, method, path, headers, body):
    """Returns the authenticated device id or raises AuthError. Records the nonce on success."""
    device = headers.get("X-PG-Device", "")
    ts_raw = headers.get("X-PG-Timestamp", "")
    nonce = headers.get("X-PG-Nonce", "")
    sig = headers.get("X-PG-Signature", "")
    if not DEVICE_RE.match(device) or not NONCE_RE.match(nonce) or not re.match(r"^[0-9a-f]{64}$", sig):
        raise AuthError(401, "missing or malformed authentication headers")
    try:
        ts = int(ts_raw)
    except ValueError:
        raise AuthError(401, "bad timestamp")
    now = store.clock()
    if abs(now - ts) > cfg.clock_skew_s:
        raise AuthError(401, "timestamp outside the allowed window")
    with store.tx() as db:
        row = db.execute("SELECT secret, disabled FROM devices WHERE device_id=?", (device,)).fetchone()
        if row is None or row["disabled"]:
            raise AuthError(401, "unknown or disabled device")   # same answer as a bad signature
        expected = sign(bytes(row["secret"]), method, path, ts, nonce, body)
        if not hmac.compare_digest(expected, sig):
            raise AuthError(401, "bad signature")
        db.execute("DELETE FROM nonces WHERE ts < ?", (now - 2 * cfg.clock_skew_s,))
        try:
            db.execute("INSERT INTO nonces(device_id, nonce, ts) VALUES (?,?,?)", (device, nonce, now))
        except Exception:
            raise AuthError(401, "replayed request")
    return device
