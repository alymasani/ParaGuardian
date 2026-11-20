"""ParaGuardian emergency delivery backend (local reference implementation).

Android companion --HTTPS--> authenticated Emergency API --> provider adapter --> consented contacts.
Standard library only (http.server, sqlite3, hmac, urllib): runs anywhere Python 3.9+ runs.
Real delivery is DISABLED by default; the fake provider never contacts anybody.
"""
__version__ = "0.8.0"
