"""Adapt already decoded RM2027 frames to the existing ROS envelope."""
from dataclasses import dataclass

from .rm_protocol import RefereeFrame
from ..vendor.rm2027.protocol import crc8_rm, crc16_rm


@dataclass(frozen=True)
class RM2027Frame(RefereeFrame):
    reference_decoded: dict

    def to_dict(self):
        result = super().to_dict()
        result['rm2027_decoded'] = self.reference_decoded
        return result


def adapt_reference_frame(frame):
    # The reference decoder has already verified both CRCs and decoded the
    # business payload. Reconstruct raw bytes only for recording/ROS consumers.
    header = bytes((0xA5, len(frame.payload) & 255, len(frame.payload) >> 8, frame.sequence))
    raw = header + bytes((crc8_rm(header),)) + frame.command.to_bytes(2, 'little') + frame.payload
    raw += crc16_rm(raw).to_bytes(2, 'little')
    return RM2027Frame(frame.command, frame.payload, frame.sequence, raw, frame.decoded)
