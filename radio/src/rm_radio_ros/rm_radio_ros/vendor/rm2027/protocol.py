from dataclasses import dataclass
from typing import Any
from .shark_core.wire2027 import AIR_LENGTHS, decode_air


def crc8_rm(data: bytes) -> int:
    crc = 0xFF

    for value in data:
        crc ^= value

        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) & 0xFF if crc & 1 else crc >> 1

    return crc


def crc16_rm(data: bytes) -> int:
    crc = 0xFFFF

    for value in data:
        crc ^= value

        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8408) & 0xFFFF if crc & 1 else crc >> 1

    return crc


def u16(data: bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def u32(data: bytes, offset: int) -> int:
    return (
        data[offset]
        | (data[offset + 1] << 8)
        | (data[offset + 2] << 16)
        | (data[offset + 3] << 24)
    )


@dataclass
class RadarFrame:
    command: int
    sequence: int
    payload: bytes
    decoded: dict[str, Any]


class RadarProtocolDecoder:
    """
    输入连续的射频分段数据（每次 15 字节）。
    输出经过 A5 / CRC8 / CRC16 校验的雷达无线链路帧。
    """

    def __init__(self) -> None:
        self.buffer = bytearray()

        self.crc8_errors = 0
        self.crc16_errors = 0
        self.valid_frames = 0

    def reset(self) -> None:
        self.buffer.clear()
        self.crc8_errors = 0
        self.crc16_errors = 0
        self.valid_frames = 0

    def feed_fragment(self, fragment: bytes) -> list[RadarFrame]:
        self.buffer.extend(fragment)
        frames: list[RadarFrame] = []

        while True:
            try:
                start = self.buffer.index(0xA5)
            except ValueError:
                self.buffer.clear()
                return frames

            if start:
                del self.buffer[:start]

            if len(self.buffer) < 5:
                return frames

            payload_len = self.buffer[1] | (self.buffer[2] << 8)

            # 雷达无线链路的业务数据最大为 47 B；128 是防噪声保护。
            if payload_len > 128:
                del self.buffer[0]
                continue

            header = bytes(self.buffer[:5])

            if crc8_rm(header[:4]) != header[4]:
                self.crc8_errors += 1
                del self.buffer[0]
                continue

            total_len = 5 + 2 + payload_len + 2

            if len(self.buffer) < total_len:
                return frames

            raw = bytes(self.buffer[:total_len])
            expected_crc16 = raw[-2] | (raw[-1] << 8)

            if crc16_rm(raw[:-2]) != expected_crc16:
                self.crc16_errors += 1
                del self.buffer[0]
                continue

            command = raw[5] | (raw[6] << 8)
            payload = raw[7:-2]

            if AIR_LENGTHS.get(command) != payload_len:
                del self.buffer[:total_len]
                continue
            if command == 0x0A06 and not (payload.isascii() and payload.isalnum()):
                del self.buffer[:total_len]
                continue

            frame = RadarFrame(
                command=command,
                sequence=raw[3],
                payload=payload,
                decoded=self.decode_payload(command, payload),
            )

            self.valid_frames += 1
            frames.append(frame)

            del self.buffer[:total_len]

    decode_payload = staticmethod(decode_air)
