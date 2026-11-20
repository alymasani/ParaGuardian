"""Durable state in SQLite (WAL). Every state change is one transaction.

Tables
  devices    provisioned devices and their request-signing secret
  nonces     recently used request nonces (replay protection)
  contacts   the device owner's consented emergency contacts (resolved SERVER-side)
  incidents  one row per (device, incident key): newest revision, kind, minimal payload
  jobs       one outbound message per (incident, contact, purpose): the idempotency unit
  callbacks  provider status callbacks already applied (duplicates ignored)
  events     privacy-conscious audit trail (no phone numbers, no coordinates)
"""
import sqlite3
import threading
import time

SCHEMA = """
PRAGMA journal_mode=WAL;
CREATE TABLE IF NOT EXISTS devices (
  device_id TEXT PRIMARY KEY, secret BLOB NOT NULL, created REAL NOT NULL, disabled INTEGER NOT NULL DEFAULT 0);
CREATE TABLE IF NOT EXISTS nonces (
  device_id TEXT NOT NULL, nonce TEXT NOT NULL, ts REAL NOT NULL, PRIMARY KEY (device_id, nonce));
CREATE TABLE IF NOT EXISTS contacts (
  device_id TEXT NOT NULL, contact_id TEXT NOT NULL, name TEXT NOT NULL, e164 TEXT NOT NULL,
  consent_at REAL NOT NULL, PRIMARY KEY (device_id, contact_id));
CREATE TABLE IF NOT EXISTS incidents (
  device_id TEXT NOT NULL, incident_key TEXT NOT NULL, revision INTEGER NOT NULL, kind TEXT NOT NULL,
  trigger INTEGER NOT NULL, test INTEGER NOT NULL, lat REAL, lon REAL, accuracy_m REAL, position_age_ms INTEGER,
  position_source TEXT, battery_mv INTEGER, identity_volatile INTEGER NOT NULL, restored INTEGER NOT NULL,
  created REAL NOT NULL, updated REAL NOT NULL, cancelled INTEGER NOT NULL DEFAULT 0,
  PRIMARY KEY (device_id, incident_key));
CREATE TABLE IF NOT EXISTS jobs (
  job_id INTEGER PRIMARY KEY AUTOINCREMENT, device_id TEXT NOT NULL, incident_key TEXT NOT NULL,
  contact_id TEXT NOT NULL, purpose TEXT NOT NULL, state TEXT NOT NULL, attempts INTEGER NOT NULL DEFAULT 0,
  next_attempt REAL NOT NULL, lease_until REAL NOT NULL DEFAULT 0, provider_id TEXT, last_error TEXT,
  created REAL NOT NULL, updated REAL NOT NULL,
  UNIQUE (device_id, incident_key, contact_id, purpose));
CREATE INDEX IF NOT EXISTS jobs_due ON jobs(state, next_attempt);
CREATE INDEX IF NOT EXISTS jobs_provider ON jobs(provider_id);
CREATE TABLE IF NOT EXISTS callbacks (
  provider_id TEXT NOT NULL, status TEXT NOT NULL, ts REAL NOT NULL, PRIMARY KEY (provider_id, status));
CREATE TABLE IF NOT EXISTS events (
  id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL NOT NULL, device_id TEXT, incident_key TEXT, what TEXT NOT NULL);
"""

# Job states. "provider_accepted" is NOT delivery; "unknown" is never turned into success or resent blindly.
QUEUED, SENDING, PROVIDER_ACCEPTED, DELIVERED, FAILED, UNKNOWN, CANCELLED = (
    "queued", "sending", "provider_accepted", "delivered", "failed", "unknown", "cancelled")
TERMINAL = (DELIVERED, FAILED, CANCELLED)


class Store:
    def __init__(self, path, clock=time.time):
        self.path = path
        self.clock = clock
        self._lock = threading.RLock()
        self.db = sqlite3.connect(path, check_same_thread=False, isolation_level=None, timeout=10.0)
        self.db.row_factory = sqlite3.Row
        self.db.executescript(SCHEMA)

    def tx(self):
        return _Tx(self)

    def event(self, device_id, incident_key, what):
        self.db.execute("INSERT INTO events(ts, device_id, incident_key, what) VALUES (?,?,?,?)",
                        (self.clock(), device_id, incident_key, what))

    def close(self):
        self.db.close()


class _Tx:
    """BEGIN IMMEDIATE ... COMMIT under a process lock: serialises writers (API threads + worker)."""

    def __init__(self, store):
        self.s = store

    def __enter__(self):
        self.s._lock.acquire()
        self.s.db.execute("BEGIN IMMEDIATE")
        return self.s.db

    def __exit__(self, exc_type, exc, tb):
        try:
            self.s.db.execute("ROLLBACK" if exc_type else "COMMIT")
        finally:
            self.s._lock.release()
        return False
