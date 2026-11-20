"""Configuration from environment variables only. No secret is ever stored in source."""
import os
from dataclasses import dataclass, field


def _bool(name, default=False):
    v = os.environ.get(name)
    if v is None:
        return default
    return v.strip().lower() in ("1", "true", "yes", "on")


@dataclass
class Config:
    db_path: str = "paraguardian_sos.db"
    host: str = "127.0.0.1"          # local development: loopback only
    port: int = 8443
    tls_cert: str = ""               # PEM paths; required for any non-loopback deployment
    tls_key: str = ""
    provider: str = "fake"           # "fake" | "twilio"
    live_delivery_enabled: bool = False   # second, explicit switch: real messages only if True
    twilio_account_sid: str = ""
    twilio_auth_token: str = ""
    twilio_from: str = ""
    public_base_url: str = ""        # for provider status callbacks + webhook signature checks
    max_body_bytes: int = 4096
    clock_skew_s: int = 300
    requests_per_minute: int = 30
    new_incidents_per_hour: int = 6
    max_contacts: int = 5
    max_attempts: int = 5
    retry_base_s: float = 5.0
    lease_s: float = 60.0
    provider_timeout_s: float = 10.0
    retention_days: int = 30
    extra: dict = field(default_factory=dict)

    @staticmethod
    def from_env():
        c = Config()
        c.db_path = os.environ.get("PG_DB", c.db_path)
        c.host = os.environ.get("PG_HOST", c.host)
        c.port = int(os.environ.get("PG_PORT", c.port))
        c.tls_cert = os.environ.get("PG_TLS_CERT", "")
        c.tls_key = os.environ.get("PG_TLS_KEY", "")
        c.provider = os.environ.get("PG_PROVIDER", "fake")
        c.live_delivery_enabled = _bool("PG_LIVE_DELIVERY_ENABLED", False)
        c.twilio_account_sid = os.environ.get("TWILIO_ACCOUNT_SID", "")
        c.twilio_auth_token = os.environ.get("TWILIO_AUTH_TOKEN", "")
        c.twilio_from = os.environ.get("TWILIO_FROM", "")
        c.public_base_url = os.environ.get("PG_PUBLIC_BASE_URL", "")
        return c

    def live_ready(self):
        """Real delivery needs BOTH the explicit switch and complete provider credentials."""
        return (self.provider == "twilio" and self.live_delivery_enabled and bool(self.twilio_account_sid)
                and bool(self.twilio_auth_token) and bool(self.twilio_from))
