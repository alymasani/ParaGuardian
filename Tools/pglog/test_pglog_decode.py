"""Decoder tests against the golden vector the firmware test also checks (cross-language schema pin)."""
import os
import struct
import sys
import unittest
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pglog_decode as d  # noqa: E402

VEC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Tests", "vectors", "pglog_v1_records.bin")


def record(seq, ts, typ, payload):
    body = struct.pack("<BBBBII", 0xA5, typ, len(payload), 1, seq, ts) + payload
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


class DecoderTests(unittest.TestCase):
    def test_golden_vector(self):
        recs, stats = d.decode_stream(open(VEC, "rb").read())
        self.assertEqual(len(recs), 1)
        r = recs[0]
        self.assertEqual((r["type"], r["seq"], r["t_ms"], r["alt_cm"], r["vz_mmps"]), ("Navigation", 7, 0x01020304,
                                                                                         12345, -1500))
        self.assertIsNone(r["az_mmps2"])   # INT32_MIN sentinel -> invalid, not a number

    def test_region_order_torn_page_and_bad_sector(self):
        sector = bytearray(b"\xff" * 4096 * 3)

        def header(base, seq):
            h = struct.pack("<IBBHI", 0x534C4750, 1, 1, 0, seq)
            sector[base:base + 16] = h + struct.pack("<I", zlib.crc32(h) & 0xFFFFFFFF)
        header(4096, 5)                       # newer sector stored first in the ring
        header(0, 6)
        nav = struct.pack("<BiiiiI", 2, 1, 2, 3, 4, 0)
        a = record(10, 100, 3, nav)
        sector[4096 + 16:4096 + 16 + len(a)] = a
        b = record(11, 200, 3, nav)
        sector[16:16 + len(b)] = b
        torn = bytearray(record(12, 300, 3, nav))
        torn[20] ^= 0x55
        sector[16 + len(b):16 + len(b) + len(torn)] = torn
        sector[8192:8208] = b"garbage-sector!!"   # interrupted erase: no valid header
        recs, stats = d.decode_region(bytes(sector))
        self.assertEqual([r["seq"] for r in recs], [10, 11])
        self.assertEqual(stats["torn_or_corrupt_pages"], 1)
        self.assertEqual(stats["sectors"], 2)


if __name__ == "__main__":
    unittest.main()
