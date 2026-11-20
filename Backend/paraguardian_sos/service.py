"""Core emergency delivery logic (transport-independent; the HTTP layer only parses and authenticates).

IDEMPOTENCY (no duplicate messages under retries):
  - An incident is identified by (device_id, incident_key). Android sends the MCU incident id as the key,
    or "<id>-v<first-seen-ms>" when the firmware marked the id volatile (it may repeat after a reset).
  - Re-submitting the same or an older revision changes nothing and returns the current status
    (Android retries after a lost HTTP response; the MCU re-sends after a reset).
  - Outbound messages are rows in `jobs`, UNIQUE per (incident, contact, purpose): created once, ever.
  - A job is claimed with a lease before the provider call. If the worker dies mid-call, the lease
    expires and the job becomes UNKNOWN, never re-sent automatically (the message may already be out).
    Only an explicit retry-unknown request (a person deciding) re-queues it.
  - Provider timeouts are UNKNOWN for the same reason; only definite "nothing was sent" answers
    (HTTP 429 / 5xx / connection refused) are retried, with exponential backoff, maxAttempts times.
Exactly-once delivery is not achievable with SMS providers; this design is at-most-once per job unless a
person explicitly asks for a resend, and reports every uncertain case as "unknown".
"""
import re
import time

from . import auth
from .providers import CALLBACK_STATES, FakeProvider
from .store import (CANCELLED, DELIVERED, FAILED, PROVIDER_ACCEPTED, QUEUED, SENDING, TERMINAL, UNKNOWN)

E164 = re.compile(r"^\+[1-9][0-9]{6,14}$")
KEY = re.compile(r"^[A-Za-z0-9._-]{1,64}$")
INCIDENT_FIELDS = {"incidentKey", "incidentId", "revision", "kind", "trigger", "test", "position", "batteryMv",
                   "identityVolatile", "restored"}
POSITION_FIELDS = {"lat", "lon", "accuracyM", "ageMs", "source"}


class ApiError(Exception):
    def __init__(self, status, code, message):
        super().__init__(message)
        self.status = status
        self.code = code
        self.message = message


def _int(v, lo, hi, name):
    if isinstance(v, bool) or not isinstance(v, int) or v < lo or v > hi:
        raise ApiError(400, "invalid_field", "%s out of range" % name)
    return v


def _num(v, lo, hi, name):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v != v or v < lo or v > hi:
        raise ApiError(400, "invalid_field", "%s out of range" % name)
    return float(v)


def validate_incident(doc):
    if not isinstance(doc, dict) or not set(doc).issubset(INCIDENT_FIELDS):
        raise ApiError(400, "invalid_body", "unknown or missing fields")
    key = doc.get("incidentKey")
    if not isinstance(key, str) or not KEY.match(key):
        raise ApiError(400, "invalid_field", "incidentKey")
    out = {
        "key": key,
        "incident_id": _int(doc.get("incidentId"), 1, 0xFFFFFFFF, "incidentId"),
        "revision": _int(doc.get("revision"), 1, 65535, "revision"),
        "trigger": _int(doc.get("trigger", 0), 0, 4, "trigger"),
    }
    kind = doc.get("kind")
    if kind not in ("alert", "cancel"):
        raise ApiError(400, "invalid_field", "kind")
    out["kind"] = kind
    for b in ("test", "identityVolatile", "restored"):
        if not isinstance(doc.get(b, False), bool):
            raise ApiError(400, "invalid_field", b)
    out["test"] = doc.get("test", False)
    out["identity_volatile"] = doc.get("identityVolatile", False)
    out["restored"] = doc.get("restored", False)
    bat = doc.get("batteryMv")
    out["battery_mv"] = None if bat is None else _int(bat, 1, 20000, "batteryMv")
    pos = doc.get("position")
    out["position"] = None
    if pos is not None:
        if not isinstance(pos, dict) or not set(pos).issubset(POSITION_FIELDS) or "lat" not in pos or "lon" not in pos:
            raise ApiError(400, "invalid_field", "position")
        src = pos.get("source", "gnss")
        if src not in ("gnss", "phone"):
            raise ApiError(400, "invalid_field", "position.source")
        acc = pos.get("accuracyM")
        out["position"] = {
            "lat": _num(pos["lat"], -90, 90, "lat"), "lon": _num(pos["lon"], -180, 180, "lon"),
            "accuracy_m": None if acc is None else _num(acc, 0, 100000, "accuracyM"),
            "age_ms": _int(pos.get("ageMs", 0), 0, 7 * 24 * 3600 * 1000, "ageMs"), "source": src,
        }
    return out


def message_text(inc, purpose, test):
    lines = []
    if test:
        lines.append("TEST ONLY - NOT A REAL EMERGENCY.")
    if purpose == "cancel_notice":
        lines.append("ParaGuardian: the earlier SOS (#%s) was CANCELLED by the pilot or device." % inc["incident_key"])
        lines.append("If you already acted on it, please confirm the pilot is safe directly.")
        return "\n".join(lines)
    lines.append("ParaGuardian SOS #%s." % inc["incident_key"])
    lines.append("Manual SOS from the pilot." if inc["trigger"] == 2 else
                 "Possible paragliding crash detected automatically (unverified).")
    if inc["lat"] is not None:
        age = int((inc["position_age_ms"] or 0) / 1000)
        lines.append("Last known position (%s, %ds old): https://maps.google.com/?q=%.6f,%.6f"
                     % (inc["position_source"], age, inc["lat"], inc["lon"]))
    else:
        lines.append("Location unavailable.")
    lines.append("Call local emergency services if you cannot reach the pilot.")
    return "\n".join(lines)


class EmergencyService:
    def __init__(self, cfg, store, provider, clock=None):
        self.cfg = cfg
        self.store = store
        self.provider = provider
        self.test_provider = FakeProvider()   # test incidents NEVER reach a live provider
        self.clock = clock or store.clock
        self._buckets = {}

    # ---------------- provisioning / contacts ----------------
    def provision_device(self, device_id):
        if not auth.DEVICE_RE.match(device_id):
            raise ApiError(400, "invalid_field", "device id")
        secret = auth.new_secret()
        with self.store.tx() as db:
            db.execute("INSERT OR REPLACE INTO devices(device_id, secret, created, disabled) VALUES (?,?,?,0)",
                       (device_id, secret, self.clock()))
        return secret

    def set_contacts(self, device, contacts):
        if not isinstance(contacts, list) or len(contacts) > self.cfg.max_contacts:
            raise ApiError(400, "invalid_body", "contacts must be a list of at most %d" % self.cfg.max_contacts)
        rows = []
        seen = set()
        for i, c in enumerate(contacts):
            if not isinstance(c, dict) or set(c) != {"name", "e164", "consent"}:
                raise ApiError(400, "invalid_body", "contact fields: name, e164, consent")
            name, num = c["name"], c["e164"]
            if not isinstance(name, str) or not (1 <= len(name.strip()) <= 60) or not isinstance(num, str) \
                    or not E164.match(num) or num in seen:
                raise ApiError(400, "invalid_field", "contact %d" % i)
            if c["consent"] is not True:   # nobody receives emergency messages without having agreed to
                raise ApiError(400, "consent_required", "contact %d has not consented" % i)
            seen.add(num)
            rows.append(("C%d" % (i + 1), name.strip(), num))
        with self.store.tx() as db:
            db.execute("DELETE FROM contacts WHERE device_id=?", (device,))
            for cid, name, num in rows:
                db.execute("INSERT INTO contacts VALUES (?,?,?,?,?)", (device, cid, name, num, self.clock()))
            self.store.event(device, None, "contacts set: %d" % len(rows))
        return {"contacts": len(rows)}

    # ---------------- rate limiting ----------------
    def check_rate(self, device):
        now = self.clock()
        tokens, last = self._buckets.get(device, (float(self.cfg.requests_per_minute), now))
        tokens = min(float(self.cfg.requests_per_minute), tokens + (now - last) * self.cfg.requests_per_minute / 60.0)
        if tokens < 1.0:
            self._buckets[device] = (tokens, now)
            raise ApiError(429, "rate_limited", "too many requests")
        self._buckets[device] = (tokens - 1.0, now)

    # ---------------- incidents ----------------
    def submit(self, device, doc):
        inc = validate_incident(doc)
        now = self.clock()
        pos = inc["position"] or {}
        with self.store.tx() as db:
            row = db.execute("SELECT * FROM incidents WHERE device_id=? AND incident_key=?",
                             (device, inc["key"])).fetchone()
            if row is not None and inc["revision"] <= row["revision"]:
                result = "duplicate"              # replay / retry: nothing changes, nothing is re-sent
            elif row is None:
                recent = db.execute("SELECT COUNT(*) FROM incidents WHERE device_id=? AND created > ?",
                                    (device, now - 3600)).fetchone()[0]
                if recent >= self.cfg.new_incidents_per_hour:
                    raise ApiError(429, "incident_rate_limited", "too many new incidents for this device")
                db.execute(
                    "INSERT INTO incidents(device_id, incident_key, revision, kind, trigger, test, lat, lon, accuracy_m,"
                    " position_age_ms, position_source, battery_mv, identity_volatile, restored, created, updated,"
                    " cancelled) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                    (device, inc["key"], inc["revision"], inc["kind"], inc["trigger"], int(inc["test"]), pos.get("lat"),
                     pos.get("lon"), pos.get("accuracy_m"), pos.get("age_ms"), pos.get("source"), inc["battery_mv"],
                     int(inc["identity_volatile"]), int(inc["restored"]), now, now, int(inc["kind"] == "cancel")))
                if inc["kind"] == "alert":
                    contacts = db.execute("SELECT contact_id FROM contacts WHERE device_id=?", (device,)).fetchall()
                    for c in contacts:
                        db.execute("INSERT OR IGNORE INTO jobs(device_id, incident_key, contact_id, purpose, state,"
                                   " next_attempt, created, updated) VALUES (?,?,?,?,?,?,?,?)",
                                   (device, inc["key"], c["contact_id"], "alert", QUEUED, now, now, now))
                    result = "created" if contacts else "no_recipients"
                else:
                    result = "cancelled_before_dispatch"
                self.store.event(device, inc["key"], "incident %s rev %d: %s" % (inc["kind"], inc["revision"], result))
            else:
                db.execute("UPDATE incidents SET revision=?, updated=?, lat=COALESCE(?, lat), lon=COALESCE(?, lon),"
                           " accuracy_m=COALESCE(?, accuracy_m), position_age_ms=COALESCE(?, position_age_ms),"
                           " position_source=COALESCE(?, position_source), battery_mv=COALESCE(?, battery_mv),"
                           " restored=MAX(restored, ?) WHERE device_id=? AND incident_key=?",
                           (inc["revision"], now, pos.get("lat"), pos.get("lon"), pos.get("accuracy_m"),
                            pos.get("age_ms"), pos.get("source"), inc["battery_mv"], int(inc["restored"]), device,
                            inc["key"]))
                result = "updated"
                if inc["kind"] == "cancel" and not row["cancelled"]:
                    result = self._cancel(db, device, inc["key"], now)
                elif row["cancelled"] and inc["kind"] == "alert":
                    result = "already_cancelled"
                self.store.event(device, inc["key"], "incident rev %d: %s" % (inc["revision"], result))
            status = self._status(db, device, inc["key"])
        status["result"] = result
        return (202 if result == "created" else 200), status

    def _cancel(self, db, device, key, now):
        db.execute("UPDATE incidents SET cancelled=1, kind='cancel' WHERE device_id=? AND incident_key=?", (device, key))
        db.execute("UPDATE jobs SET state=?, updated=? WHERE device_id=? AND incident_key=? AND purpose='alert'"
                   " AND state=?", (CANCELLED, now, device, key, QUEUED))
        maybe_sent = db.execute(
            "SELECT contact_id FROM jobs WHERE device_id=? AND incident_key=? AND purpose='alert' AND state IN (?,?,?,?)",
            (device, key, SENDING, PROVIDER_ACCEPTED, DELIVERED, UNKNOWN)).fetchall()
        for c in maybe_sent:   # the alert may be out: tell exactly those contacts it was cancelled
            db.execute("INSERT OR IGNORE INTO jobs(device_id, incident_key, contact_id, purpose, state, next_attempt,"
                       " created, updated) VALUES (?,?,?,?,?,?,?,?)",
                       (device, key, c["contact_id"], "cancel_notice", QUEUED, now, now, now))
        return "cancelled_notices_queued" if maybe_sent else "cancelled_before_dispatch"

    def status(self, device, key):
        if not KEY.match(key or ""):
            raise ApiError(400, "invalid_field", "incident key")
        with self.store.tx() as db:
            st = self._status(db, device, key)
        if st is None:
            raise ApiError(404, "not_found", "unknown incident")
        return st

    def _status(self, db, device, key):
        inc = db.execute("SELECT * FROM incidents WHERE device_id=? AND incident_key=?", (device, key)).fetchone()
        if inc is None:
            return None
        jobs = db.execute("SELECT contact_id, purpose, state, attempts, updated FROM jobs WHERE device_id=? AND"
                          " incident_key=? ORDER BY job_id", (device, key)).fetchall()
        alerts = [j["state"] for j in jobs if j["purpose"] == "alert"]
        if not alerts:
            overall = "cancelled" if inc["cancelled"] else "no_recipients"
        elif DELIVERED in alerts:
            overall = "delivered"           # at least one contact's carrier reported delivery
        elif PROVIDER_ACCEPTED in alerts:
            overall = "provider_accepted"   # handed to the provider; NOT delivered yet
        elif QUEUED in alerts or SENDING in alerts:
            overall = "pending"
        elif UNKNOWN in alerts:
            overall = "unknown"
        elif all(s == CANCELLED for s in alerts):
            overall = "cancelled"
        else:
            overall = "failed"
        return {
            "incidentKey": key, "revision": inc["revision"], "cancelled": bool(inc["cancelled"]),
            "test": bool(inc["test"]), "overall": overall,
            "jobs": [{"contact": j["contact_id"], "purpose": j["purpose"], "state": j["state"],
                      "attempts": j["attempts"], "updated": j["updated"]} for j in jobs],
            "provider": self.provider.name, "liveDelivery": self.provider.name != "fake" and
            self.provider.name != "disabled",
        }

    def retry_unknown(self, device, key):
        """Explicit human decision: re-send messages whose outcome is unknown (may duplicate them)."""
        now = self.clock()
        with self.store.tx() as db:
            n = db.execute("UPDATE jobs SET state=?, next_attempt=?, updated=? WHERE device_id=? AND incident_key=?"
                           " AND state=?", (QUEUED, now, now, device, key, UNKNOWN)).rowcount
            self.store.event(device, key, "retry-unknown: %d job(s) requeued by request" % n)
            st = self._status(db, device, key)
        if st is None:
            raise ApiError(404, "not_found", "unknown incident")
        st["requeued"] = n
        return st

    # ---------------- worker ----------------
    def worker_step(self):
        """Claims and sends ONE due job. Returns True if a job was processed."""
        now = self.clock()
        with self.store.tx() as db:
            lost = db.execute("UPDATE jobs SET state=?, last_error='worker lost during send', updated=? WHERE state=?"
                              " AND lease_until < ?", (UNKNOWN, now, SENDING, now)).rowcount
            if lost:
                self.store.event(None, None, "%d job(s) marked unknown after an expired lease" % lost)
            job = db.execute("SELECT * FROM jobs WHERE state=? AND next_attempt <= ? ORDER BY job_id LIMIT 1",
                             (QUEUED, now)).fetchone()
            if job is None:
                return False
            lease = now + self.cfg.lease_s
            db.execute("UPDATE jobs SET state=?, attempts=attempts+1, lease_until=?, updated=? WHERE job_id=?",
                       (SENDING, lease, now, job["job_id"]))
            inc = db.execute("SELECT * FROM incidents WHERE device_id=? AND incident_key=?",
                             (job["device_id"], job["incident_key"])).fetchone()
            contact = db.execute("SELECT e164 FROM contacts WHERE device_id=? AND contact_id=?",
                                 (job["device_id"], job["contact_id"])).fetchone()
        if contact is None:
            return self._finish(job, lease, "rejected", "", "contact removed")
        test = bool(inc["test"])
        provider = self.test_provider if test else self.provider
        body = message_text(inc, job["purpose"], test or provider.name == "fake")
        callback = (self.cfg.public_base_url.rstrip("/") + "/v1/provider/twilio/status"
                    if self.cfg.public_base_url and provider.name == "twilio" else "")
        try:
            res = provider.send(contact["e164"], body, callback)
            outcome, pid, detail = res.outcome, res.provider_id, res.detail
        except Exception as e:   # adapter bug: the request may have gone out
            outcome, pid, detail = "unknown", "", "adapter error: %s" % type(e).__name__
        return self._finish(job, lease, outcome, pid, detail)

    def _finish(self, job, lease, outcome, pid, detail):
        now = self.clock()
        with self.store.tx() as db:
            cur = db.execute("SELECT state, attempts, lease_until FROM jobs WHERE job_id=?", (job["job_id"],)).fetchone()
            if cur["state"] != SENDING or cur["lease_until"] != lease:
                return True   # lease was lost meanwhile: the other path already recorded "unknown"
            attempts = cur["attempts"]
            if outcome == "accepted":
                state, nxt = PROVIDER_ACCEPTED, now
            elif outcome == "retry" and attempts < self.cfg.max_attempts:
                state, nxt = QUEUED, now + self.cfg.retry_base_s * (2 ** (attempts - 1))
            elif outcome == "unknown":
                state, nxt = UNKNOWN, now
            else:
                state, nxt = FAILED, now
            db.execute("UPDATE jobs SET state=?, next_attempt=?, provider_id=COALESCE(NULLIF(?,''), provider_id),"
                       " last_error=?, lease_until=0, updated=? WHERE job_id=?",
                       (state, nxt, pid, detail[:120], now, job["job_id"]))
            self.store.event(job["device_id"], job["incident_key"],
                             "job %d %s: %s" % (job["job_id"], job["purpose"], state))
        return True

    def handle_callback(self, provider_id, status):
        if not provider_id or len(provider_id) > 64:
            raise ApiError(400, "invalid_field", "provider id")
        target = CALLBACK_STATES.get(status)
        now = self.clock()
        with self.store.tx() as db:
            try:
                db.execute("INSERT INTO callbacks(provider_id, status, ts) VALUES (?,?,?)", (provider_id, status, now))
            except Exception:
                return "duplicate"
            job = db.execute("SELECT job_id, state, device_id, incident_key FROM jobs WHERE provider_id=?",
                             (provider_id,)).fetchone()
            if job is None:
                return "unknown_message"
            if target is None or job["state"] in TERMINAL and job["state"] != FAILED:
                return "ignored"
            if target == FAILED and job["state"] == DELIVERED:
                return "ignored"   # never downgrade a reported delivery
            db.execute("UPDATE jobs SET state=?, updated=? WHERE job_id=?", (target, now, job["job_id"]))
            self.store.event(job["device_id"], job["incident_key"], "callback: job %d -> %s" % (job["job_id"], target))
        return "applied"

    # ---------------- operations ----------------
    def diagnostics(self):
        with self.store.tx() as db:
            states = {r[0]: r[1] for r in db.execute("SELECT state, COUNT(*) FROM jobs GROUP BY state")}
            devices = db.execute("SELECT COUNT(*) FROM devices").fetchone()[0]
        return {"provider": self.provider.name, "liveDelivery": self.provider.name not in ("fake", "disabled"),
                "devices": devices, "jobs": states}

    def purge(self):
        """Minimal retention: coordinates / battery dropped after retention_days; audit trail trimmed."""
        cutoff = self.clock() - self.cfg.retention_days * 86400
        with self.store.tx() as db:
            n = db.execute("UPDATE incidents SET lat=NULL, lon=NULL, accuracy_m=NULL, battery_mv=NULL WHERE updated < ?",
                           (cutoff,)).rowcount
            db.execute("DELETE FROM events WHERE ts < ?", (cutoff,))
            db.execute("DELETE FROM callbacks WHERE ts < ?", (cutoff,))
        return n
