#!/usr/bin/env python3
"""Writes golden.json: r1can.py's decode of every SAMPLE_EVERY-th frame of each ID (all of them for IDs with <= 100).

usage: make_golden.py <r1_vendor/decode dir> <candump.log> > golden.json
"""
import collections, json, sys

sys.path.insert(0, sys.argv[1])
import r1can  # noqa: E402

SAMPLE_EVERY = 25

frames = collections.defaultdict(list)
for line_number, (stamp, can_id, flags, data) in enumerate(r1can.parse_candump(sys.argv[2])):
    frames[can_id].append((line_number, data))

golden = {}
for can_id, rows in sorted(frames.items()):
    # low-rate IDs (versions, errors, BMS, temperatures) keep every frame
    picked = range(len(rows)) if len(rows) <= 100 else sorted(set(range(0, len(rows), SAMPLE_EVERY)) | {len(rows) - 1})
    golden[f"0x{can_id:03X}"] = {
        "count": len(rows),
        "samples": [{"line": rows[i][0], "decoded": r1can.decode_frame(can_id, rows[i][1])} for i in picked],
    }
json.dump(golden, sys.stdout, separators=(",", ":"))
