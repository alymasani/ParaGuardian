#!/usr/bin/env python3
"""Decode ParaGuardian flash-log records (schema v1) to CSV or JSON.

Input: a raw dump of a log partition (flight log 0x020000.., critical log 0x012000.., see
Components/Storage/NorFlash.hpp) or a file of concatenated records.
  python3 pglog_decode.py dump.bin --region [--json]
Sectors (4 KiB) start with a 16-byte header "PGLS"; records never cross 256-byte pages; 0xFF = unused.
Records failing their CRC are skipped and counted (torn writes / corruption), never guessed.
The payload layouts mirror Components/Storage/LogRecords.hpp + FlightRecorder.cpp.
"""
import argparse
import csv
import json
import struct
import sys
import zlib

SYNC, SCHEMA, SECTOR, PAGE, HDR = 0xA5, 1, 4096, 256, 16
TYPES = {
    1: ("Boot", "<IIBBB", ["resetFlags", "bootCount", "journalMount", "criticalMount", "flightMount"]),
    2: ("SensorQuality", "<IBBBBBBH", ["qualityFlags", "accelStatus", "gyroStatus", "baroStatus", "accelMask",
                                      "gyroMask", "baroMask", "sourcesHealthy"]),
    3: ("Navigation", "<BiiiiI", ["state", "alt_cm", "vz_mmps", "az_mmps2", "altSigma_cm", "flags"]),
    4: ("SafetyEvent", "<IBBBBBBIii", ["eventId", "type", "severity", "previous", "transition", "source", "phase",
                                      "episode", "vz_mmps", "clearance_cm"]),
    5: ("CrashSummary", "<BBBIiiII", ["state", "decision", "class", "evidence", "peak_cg", "rate_dps", "candidates",
                                     "suspicions"]),
    6: ("Emergency", "<BBBBIHHIBB", ["state", "trigger", "stage", "cancelNotice", "incident", "revision",
                                    "reportedStages", "flags", "persist", "attempts"]),
    7: ("StorageFault", "<BBII", ["component", "status", "detail", "count"]),
    8: ("Health", "<IIIIII", ["freeHeap", "minHeap", "procMaxMs", "lateWakes", "stackMinWords", "faults"]),
    9: ("Power", "<BHBBBBB", ["batteryLevel", "mv", "flags", "rail", "amp", "lidarPower", "supply"]),
    10: ("Profile", "<BB", ["profile", "reason"]),
}
INVALID = -2 ** 31


def decode_record(buf, off):
    """Returns (record dict, length) or (None, 0)."""
    if len(buf) - off < 16 or buf[off] != SYNC:
        return None, 0
    typ, ln, schema = buf[off + 1], buf[off + 2], buf[off + 3]
    total = 12 + ln + 4
    if schema != SCHEMA or typ not in TYPES or off + total > len(buf):
        return None, 0
    (crc,) = struct.unpack_from("<I", buf, off + 12 + ln)
    if crc != zlib.crc32(bytes(buf[off:off + 12 + ln])) & 0xFFFFFFFF:
        return None, 0
    seq, ts = struct.unpack_from("<II", buf, off + 4)
    name, fmt, fields = TYPES[typ]
    rec = {"seq": seq, "t_ms": ts, "type": name}
    if struct.calcsize(fmt) == ln:
        for k, v in zip(fields, struct.unpack_from(fmt, buf, off + 12)):
            rec[k] = None if v == INVALID else v
    else:
        rec["raw"] = bytes(buf[off + 12:off + 12 + ln]).hex()   # newer payload layout: kept, not guessed
    return rec, total


def parse_page(buf, start, end, stats):
    out, off = [], start
    while off < end and buf[off] != 0xFF:
        rec, n = decode_record(buf[:end], off)
        if rec is None:
            stats["torn_or_corrupt_pages"] += 1
            break
        out.append(rec)
        off += n
    return out


def decode_region(data):
    stats = {"sectors": 0, "torn_or_corrupt_pages": 0}
    sectors = []
    for base in range(0, len(data) - SECTOR + 1, SECTOR):
        h = data[base:base + HDR]
        magic, ver, tag, _, seq, crc = struct.unpack("<IBBHII", h)
        if magic != 0x534C4750 or ver != 1 or crc != zlib.crc32(h[:12]) & 0xFFFFFFFF:
            continue
        stats["sectors"] += 1
        recs = []
        for p in range(SECTOR // PAGE):
            s = base + p * PAGE
            recs += parse_page(data, s + (HDR if p == 0 else 0), s + PAGE, stats)
        sectors.append((seq, recs))
    records = [r for _, recs in sorted(sectors, key=lambda x: x[0]) for r in recs]
    return records, stats


def decode_stream(data):
    stats = {"sectors": 0, "torn_or_corrupt_pages": 0}
    out, off = [], 0
    while off < len(data):
        rec, n = decode_record(data, off)
        if rec is None:
            stats["torn_or_corrupt_pages"] += 1
            off += 1
            continue
        out.append(rec)
        off += n
    return out, stats


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dump")
    ap.add_argument("--region", action="store_true", help="input is a partition dump with sector headers")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    data = open(a.dump, "rb").read()
    records, stats = decode_region(data) if a.region else decode_stream(data)
    if a.json:
        json.dump({"records": records, "stats": stats}, sys.stdout, indent=1)
        print()
    else:
        keys = ["seq", "t_ms", "type"]
        for r in records:
            keys += [k for k in r if k not in keys]
        w = csv.DictWriter(sys.stdout, fieldnames=keys)
        w.writeheader()
        w.writerows(records)
    print("records=%d %s" % (len(records), stats), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
