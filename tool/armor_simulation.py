"""模拟事件构造；所有会话固定标记 synthetic，不构造裁判指令。"""
import random
import uuid

from rm_radio_ros.core.adaptive_armor import EVENT_SCHEMA, FRAME, integer


class SimulatedMatch:
    source = 'qt-armor-simulator'

    def __init__(self, width, height, side, now):
        self.width = integer(width, 'width', 1, 5000)
        self.height = integer(height, 'height', 1, 5000)
        if side not in ('red', 'blue'):
            raise ValueError('己方阵营无效')
        self.side = side
        self.match_id = 'sim-' + uuid.uuid4().hex
        self.started_at = now - 120
        self.sequence = 0
        self.shots = {'outgoing': 0, 'incoming': 0}

    def event(self, kind, now, **values):
        return dict(schema=EVENT_SCHEMA, kind=kind, timestamp=now, coordinate_frame=FRAME,
                    match_id=self.match_id, source=self.source, **values)

    def session(self):
        return self.event('session', self.started_at, origin='synthetic', side=self.side,
                          complete_from_start=True, weight_basis='count')

    def heartbeat(self, now):
        return self.event('shots', now, own_shots=self.shots['outgoing'],
                          opponent_shots=self.shots['incoming'], data_complete=True)

    def hit(self, direction, x, y, now, shots=5):
        if direction not in self.shots:
            raise ValueError('命中方向无效')
        x = integer(x, 'x_mm', 0, self.width)
        y = integer(y, 'y_mm', 0, self.height)
        shots = integer(shots, 'shots', 1, 1000)
        self.sequence += 1
        self.shots[direction] += shots
        return self.event('hit', now, direction=direction, confirmed=True, x_mm=x, y_mm=y,
                          event_id=f'{self.match_id}:module0:boot0:{self.sequence}')

    def example(self, now, x, y, spread, seed=2027):
        """Explicit historical samples let the existing holdout algorithm run immediately."""
        rng = random.Random(seed)
        events = []
        for direction, count in (('outgoing', 40), ('incoming', 100)):
            for i in range(count):
                cx, cy = (self.width/2, self.height/2) if direction == 'outgoing' else (x, y)
                sigma = min(self.width, self.height)/10 if direction == 'outgoing' else spread
                px = min(self.width, max(0, round(rng.gauss(cx, sigma))))
                py = min(self.height, max(0, round(rng.gauss(cy, sigma))))
                timestamp = (now-75 + i*35/70 if direction == 'outgoing' or i < 70
                             else now-10 + (i-70)*9/30)
                events.append(self.hit(direction, px, py, max(self.started_at, timestamp)))
        return events
