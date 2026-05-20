"""
Shared wire-format helpers for the IsaacSim5G OAI data plane.

Both ``oai_bridge/ue_teleop_bridge.py`` (UE side) and the dashboard's
``oai-real`` mode parse and produce these envelopes.

Downlink (operator → UE → /cmd_vel):
    Plain JSON UDP datagrams. Format is the same one the dashboard
    already used for its simulated-OAI broker mode:

        {
            "linear_x": ..., "linear_y": ..., "linear_z": ...,
            "angular_z": ...,
            "emergency_stop": bool,
            "source": "dashboard",
            "seq": int, "stamp_ms": int
        }

    The UE-side bridge also accepts an envelope-encoded version (see
    KIND_POSE) so the same machinery can serve both directions.

Uplink (UE → operator, carrying pose + camera frames):
    A 12-byte binary header followed by a JSON metadata blob and a
    binary payload:

        uint32_le  magic     = 0x49535435  ("IST5")
        uint8      kind      = 1 (pose) or 2 (camera)
        uint8      reserved
        uint16_le  header_len
        uint32_le  body_len
        bytes      header_json  (UTF-8 JSON dict)
        bytes      body         (camera: JPEG bytes; pose: empty)

    For pose, all the data lives in the header. For camera, the header
    describes the topic / dimensions / stamp and the body is the JPEG
    bytes.
"""
from __future__ import annotations

import json
import struct


UPLINK_MAGIC = 0x49535435           # b"IST5" little-endian
KIND_POSE = 1
KIND_CAMERA = 2

UPLINK_HDR = "<IBBHI"
UPLINK_HDR_SIZE = struct.calcsize(UPLINK_HDR)
assert UPLINK_HDR_SIZE == 12


def pack_uplink(kind: int, header: dict, body: bytes = b"") -> bytes:
    hdr_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    return struct.pack(
        UPLINK_HDR,
        UPLINK_MAGIC, int(kind), 0,
        len(hdr_bytes), len(body),
    ) + hdr_bytes + body


def unpack_uplink(buf: bytes):
    """Reverse of pack_uplink. Returns (kind, header_dict, body_bytes) or None."""
    if len(buf) < UPLINK_HDR_SIZE:
        return None
    magic, kind, _r, hlen, blen = struct.unpack(UPLINK_HDR, buf[:UPLINK_HDR_SIZE])
    if magic != UPLINK_MAGIC:
        return None
    end = UPLINK_HDR_SIZE + hlen + blen
    if len(buf) < end:
        return None
    try:
        header = json.loads(buf[UPLINK_HDR_SIZE: UPLINK_HDR_SIZE + hlen])
    except json.JSONDecodeError:
        return None
    body = buf[UPLINK_HDR_SIZE + hlen: end]
    return int(kind), header, body
