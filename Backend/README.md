# ParaGuardian emergency backend (Milestone 8)

This is a reference implementation of the unattended SOS path:

```
Android → HTTPS → Emergency API (HMAC per-device auth) → job store (SQLite) → provider adapter → consented contacts
```

It uses only the Python standard library (3.9+). **No real messages are sent unless you deliberately configure them.**

```sh
cd Backend
python3 -m unittest discover -s tests -t .                 # 18 tests; fake provider, loopback only
PG_DB=/tmp/pg.db python3 -m paraguardian_sos provision-device pg-dev-1   # prints the secret ONCE
PG_DB=/tmp/pg.db python3 -m paraguardian_sos serve          # 127.0.0.1:8443, fake provider
```

## Configuration (environment only; never commit secrets)

| Variable | Meaning |
|---|---|
| `PG_DB` | SQLite path |
| `PG_HOST` | bind address. Any non-loopback address **requires** TLS. |
| `PG_PORT` | port |
| `PG_TLS_CERT` / `PG_TLS_KEY` | PEM certificate and key, TLS 1.2+ |
| `PG_PROVIDER` | `fake` (default) or `twilio` |
| `PG_LIVE_DELIVERY_ENABLED` | must be `true` **and** the Twilio credentials must be present, or every message is refused |
| `TWILIO_ACCOUNT_SID`, `TWILIO_AUTH_TOKEN`, `TWILIO_FROM` | provider credentials (from a secrets manager) |
| `PG_PUBLIC_BASE_URL` | public HTTPS base for provider status callbacks (signature-verified) |

## API

| Request | Purpose |
|---|---|
| `POST /v1/contacts` | at most 5 contacts, E.164 numbers, `consent: true` required |
| `POST /v1/incidents` | idempotent on (device, incidentKey, revision); `kind` is `alert` or `cancel` |
| `GET /v1/incidents/<key>` | `overall`: `pending`, `provider_accepted`, `delivered`, `failed`, `unknown`, `cancelled` or `no_recipients` |
| `POST /v1/incidents/<key>/retry-unknown` | explicit human decision; this may duplicate messages |
| `POST /v1/provider/twilio/status` | provider delivery callbacks |
| `GET /v1/health` | aggregate counts only |

## Guarantees and limits

**Guarantees:**
- At most one send per (incident, contact, purpose) unless `retry-unknown` is called.
- Provider timeouts and worker crashes become `unknown`; they are never resent automatically.
- `provider_accepted` is not delivery.
- `test` incidents use only the fake provider.

**Limits:**
- The server stores the per-device HMAC secrets (production: use a secrets manager).
- No deployment was made.
- The Twilio adapter has never contacted Twilio. It is tested with an injected transport only.
