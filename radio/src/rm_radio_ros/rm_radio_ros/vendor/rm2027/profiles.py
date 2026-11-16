"""2026 PHY for laboratory compatibility only; not the unpublished 2027 RF profile."""
from dataclasses import dataclass


SAMPLE_RATE = 1_000_000
SPS = 47
BT = 0.35

INFO_ACCESS = bytes.fromhex("2F 6F 4C 74 B9 14 49 2E")
INTERFERENCE_ACCESS = bytes.fromhex("16 E8 D3 77 15 1C 71 2D")

PHY_HEADER = bytes.fromhex("00 0F 00 0F")
PHY_FRAGMENT_BYTES = 15


@dataclass(frozen=True)
class RadioProfile:
    name: str
    frequency_hz: int
    bandwidth_hz: int
    sensitivity: float
    access_code: bytes
    kind: str


PROFILES = {
    "红方信息波": RadioProfile(
        "红方信息波", 433_200_000, 540_000,
        1.5628, INFO_ACCESS, "info"
    ),
    "红方一级干扰": RadioProfile(
        "红方一级干扰", 432_200_000, 940_000,
        2.8194, INTERFERENCE_ACCESS, "interference"
    ),
    "红方二级干扰": RadioProfile(
        "红方二级干扰", 432_500_000, 860_000,
        2.5681, INTERFERENCE_ACCESS, "interference"
    ),
    "红方三级干扰": RadioProfile(
        "红方三级干扰", 432_800_000, 250_000,
        0.6517, INTERFERENCE_ACCESS, "interference"
    ),
    "蓝方信息波": RadioProfile(
        "蓝方信息波", 433_920_000, 540_000,
        1.5628, INFO_ACCESS, "info"
    ),
    "蓝方一级干扰": RadioProfile(
        "蓝方一级干扰", 434_920_000, 940_000,
        2.8194, INTERFERENCE_ACCESS, "interference"
    ),
    "蓝方二级干扰": RadioProfile(
        "蓝方二级干扰", 434_620_000, 860_000,
        2.5681, INTERFERENCE_ACCESS, "interference"
    ),
    "蓝方三级干扰": RadioProfile(
        "蓝方三级干扰", 434_320_000, 250_000,
        0.6517, INTERFERENCE_ACCESS, "interference"
    ),
}
