"""比赛台与决策节点共用的用户级装甲策略配置。"""
import json
import os
from pathlib import Path

from .adaptive_armor import ArmorConfig
from .match_recording import atomic_write_json


def config_path():
    return Path(os.environ.get(
        'RM_RADIO_ARMOR_CONFIG', '~/.config/shark-radar/adaptive-armor.json'
    )).expanduser().resolve()


def load_config(path=None):
    path = Path(path) if path is not None else config_path()
    return ArmorConfig.from_dict(json.loads(path.read_text())) if path.exists() else ArmorConfig()


def save_config(config, path=None):
    atomic_write_json(Path(path) if path is not None else config_path(), config.to_dict())
