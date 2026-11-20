"""CLI:  python -m paraguardian_sos serve | provision-device <id> | purge | diagnostics

Environment (see README.md): PG_DB, PG_HOST, PG_PORT, PG_TLS_CERT, PG_TLS_KEY, PG_PROVIDER (fake|twilio),
PG_LIVE_DELIVERY_ENABLED (must be "true" for real messages), TWILIO_ACCOUNT_SID, TWILIO_AUTH_TOKEN,
TWILIO_FROM, PG_PUBLIC_BASE_URL.
"""
import json
import logging
import sys

from .config import Config
from .providers import DisabledProvider, FakeProvider, TwilioProvider
from .server import Worker, build_server
from .service import EmergencyService
from .store import Store


def make_provider(cfg):
    if cfg.provider == "fake":
        return FakeProvider()
    if cfg.provider == "twilio":
        if not cfg.live_ready():
            print("WARNING: PG_PROVIDER=twilio but live delivery is not enabled or credentials are missing: "
                  "every message will be REFUSED (nothing is sent).", file=sys.stderr)
            return DisabledProvider()
        return TwilioProvider(cfg.twilio_account_sid, cfg.twilio_auth_token, cfg.twilio_from, cfg.provider_timeout_s)
    raise SystemExit("unknown PG_PROVIDER %r" % cfg.provider)


def main(argv):
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    cfg = Config.from_env()
    if not argv:
        print(__doc__)
        return 2
    service = EmergencyService(cfg, Store(cfg.db_path), make_provider(cfg))
    cmd = argv[0]
    if cmd == "provision-device" and len(argv) == 2:
        secret = service.provision_device(argv[1])
        # Printed ONCE for transfer into the phone's secure storage; never logged.
        print(json.dumps({"deviceId": argv[1], "secretHex": secret.hex()}))
        return 0
    if cmd == "purge":
        print("records minimised:", service.purge())
        return 0
    if cmd == "diagnostics":
        print(json.dumps(service.diagnostics(), indent=2))
        return 0
    if cmd == "serve":
        httpd = build_server(service, cfg)
        worker = Worker(service)
        worker.start()
        mode = "LIVE DELIVERY" if service.diagnostics()["liveDelivery"] else "NO REAL MESSAGES (provider %s)" % \
            service.provider.name
        print("ParaGuardian SOS backend on %s:%d - %s" % (cfg.host, cfg.port, mode), file=sys.stderr)
        try:
            httpd.serve_forever()
        finally:
            worker.stop.set()
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
