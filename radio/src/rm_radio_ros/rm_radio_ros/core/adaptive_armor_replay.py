"""离线 JSONL 回放，始终将输入标记为 replay，不创建 ROS 节点或发送请求。"""
from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path
import sys

from .adaptive_armor import (
    AdaptiveArmorDecision, ArmorConfig, CONFIG_TOPIC, EVENT_TOPIC, number,
)


def replay(rows, config=None):
    decision = AdaptiveArmorDecision(config)
    referee = {}
    previous_time = -1
    for row in rows:
        now = number(row.get('timestamp'), '接收时间')
        if now < previous_time:
            raise ValueError('回放接收时间必须单调；源事件允许乱序，但需保留原始到达顺序')
        previous_time = now
        data, kind = row.get('data', {}), row.get('kind')
        topic = row.get('topic')
        if kind == 'event' or topic == EVENT_TOPIC:
            event = dict(data)
            if event.get('kind') == 'session':
                event['origin'] = 'replay'
            decision.ingest(event, now)
        elif kind == 'config' or topic == CONFIG_TOPIC:
            decision.configure(ArmorConfig.from_dict(data.get('config', data)))
        elif kind == 'referee' or topic == '/rm_referee_serial_node/status':
            referee = data
        elif topic == '/rm_radar_integration/status':
            referee = data.get('referee_serial', referee)
        elif kind != 'tick':
            continue
        # Side belongs to the adapter session in offline playback, never a guessed red default.
        decision.evaluate(now, referee, decision.statistics.side)
        yield dict(decision.status)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, help='JSONL / JSONL.gz 文件，或 - 从 stdin 读取')
    parser.add_argument('--config', type=Path, help='策略 JSON 配置；缺省为固定居中、等待几何')
    parser.add_argument('--output', type=Path, help='逐次建议与状态 JSONL；缺省为 stdout')
    args = parser.parse_args()
    config = ArmorConfig.from_dict(json.loads(args.config.read_text())) if args.config else None
    if args.input == '-':
        stream = sys.stdin
    elif args.input.endswith('.gz'):
        stream = gzip.open(args.input, 'rt', encoding='utf-8')
    else:
        stream = open(args.input, encoding='utf-8')
    output = args.output.open('w', encoding='utf-8') if args.output else sys.stdout
    try:
        rows = (json.loads(line) for line in stream if line.strip())
        for status in replay(rows, config):
            output.write(json.dumps(status, ensure_ascii=False, allow_nan=False, sort_keys=True) + '\n')
    finally:
        if stream is not sys.stdin:
            stream.close()
        if output is not sys.stdout:
            output.close()


if __name__ == '__main__':
    main()
