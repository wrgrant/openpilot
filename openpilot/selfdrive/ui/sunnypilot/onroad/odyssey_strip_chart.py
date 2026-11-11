"""Odyssey steering history. Raw subscriptions retain samples between UI frames; no new wire schema."""
import time

import raylib_nb as rl

from openpilot.cereal import messaging
from opendbc.can.parser import CANParser

HISTORY_SECONDS = 2.0
TORQUE_SCALE = .15
WHEEL_SCALE = 5.0
CAPACITY = 2048  # over two seconds of carState + both actuator publications at 100 Hz


def scaled(value: float, full_scale: float) -> float:
  return min(1., max(-1., value / full_scale))


class FineAngle:
  def __init__(self):
    self.t = None
    self.angle = 0.

  def update(self, t: float, coarse: float, rate: float | None) -> float:
    dt = t - self.t if self.t is not None else 0.
    if rate is None or not 0 < dt <= .1:
      self.angle = coarse
    else:
      self.angle = min(coarse + .05, max(coarse - .05, self.angle + rate * dt))
    self.t = t
    return self.angle


class StripHistory:
  def __init__(self, capacity=CAPACITY):
    self.capacity = capacity
    self.times = [0.] * capacity
    self.torques = [0.] * capacity
    self.angles = [0.] * capacity
    self.head = self.count = 0

  def append(self, t: float, torque: float, angle: float):
    if self.count and t < self.times[(self.head - 1) % self.capacity]:
      if self.times[(self.head - 1) % self.capacity] - t <= .1:
        return  # late delivery across sockets does not erase the visible window
      self.head = self.count = 0
    self.times[self.head], self.torques[self.head], self.angles[self.head] = t, torque, angle
    self.head = (self.head + 1) % self.capacity
    self.count = min(self.count + 1, self.capacity)

  def recent(self, now: float, seconds=HISTORY_SECONDS):
    for n in range(self.count):
      i = (self.head - n - 1) % self.capacity
      t = self.times[i]
      if now - t > seconds + 1e-9:
        break
      if t <= now:
        yield t, self.torques[i], self.angles[i]


class OdysseyStripChart:
  def __init__(self):
    self.history = StripHistory()
    self.fine_angle = FineAngle()
    self.socks = None
    self.parser = None
    self.rate_t = self.output_t = -1e9
    self.rate = self.torque = 0.
    self.angle = None
    self.fine = False
    self.output_source = False
    self.latest = self.received_at = 0.

  def update(self, honda_odyssey: bool):
    if self.socks is None:
      self.socks = {s: messaging.sub_sock(s, conflate=False) for s in ('can', 'carState', 'carControl', 'carOutput')}
    if honda_odyssey and self.parser is None:
      self.parser = CANParser('honda_odyssey_exl_2018_generated', [('STEER_STATUS', 0)], 0)
    # Bounded drain: a stalled UI never spends an unbounded frame catching up. Normal frames have ~20 events.
    events = []
    for sock in self.socks.values():
      for _ in range(256):
        raw = sock.receive(non_blocking=True)
        if raw is None:
          break
        events.append(messaging.log_from_bytes(raw))
    events.sort(key=lambda m: m.logMonoTime)
    for m in events:
      self.ingest(m)
    if events:
      self.received_at = time.monotonic()

  def ingest(self, m):
    """Consume in logMonoTime order, also used by recorded-drive render checks."""
    t, service = m.logMonoTime / 1e9, m.which()
    if service == 'carState' and self.latest and t < self.latest - .1:
      self.history = StripHistory()
      self.fine_angle = FineAngle()
      self.rate_t = self.output_t = -1e9
      self.angle = None
    if service == 'carState':
      self.latest = t
    if not m.valid:
      return
    if service == 'can':
      if self.parser is not None:
        frames = [(c.address, bytes(c.dat), c.src) for c in m.can if c.address == 399 and c.src == 0]
        if frames and 399 in self.parser.update([(m.logMonoTime, frames)]):
          self.rate = self.parser.vl['STEER_STATUS']['STEER_ANGLE_RATE']
          self.rate_t = t
      return
    if service == 'carOutput':
      self.torque = m.carOutput.actuatorsOutput.torque
      self.output_t = t
      self.output_source = True
    elif service == 'carControl':
      # A replay's live controlsd has a different clock from its recorded carState/carOutput.
      if not self.latest or abs(t - self.latest) > 1.:
        return
      if 0 <= t - self.output_t < .2:
        return
      self.torque = m.carControl.actuators.torque
      self.output_source = False
    elif service == 'carState':
      self.fine = 0 <= t - self.rate_t < .1
      self.angle = self.fine_angle.update(t, m.carState.steeringAngleDeg, self.rate if self.fine else None)
    if self.angle is not None:
      self.history.append(t, self.torque, self.angle)

  def draw(self, rect, font):
    orange, cyan = rl.Color(255, 155, 45, 230), rl.Color(80, 230, 255, 255)
    grid = rl.Color(255, 255, 255, 60)
    rl.draw_rectangle_rec(rect, rl.Color(0, 0, 0, 155))
    labels = ((f'torque +/-{TORQUE_SCALE * 100:g}%', orange),
              (f'wheel +/-{WHEEL_SCALE:g}°', cyan),
              (f'{HISTORY_SECONDS:g} s / {"fine" if self.fine else "coarse"}', rl.WHITE),
              ('sent' if self.output_source else 'requested', orange))
    for n, (label, color) in enumerate(labels):
      rl.draw_text_ex(font, label, rl.Vector2(rect.x + 8, rect.y + 5 + 27 * n), 24, 0, color)
    x0, x1 = rect.x + 8, rect.x + rect.width - 8
    center, half = (x0 + x1) / 2, (x1 - x0) / 2
    top, bottom = rect.y + 118, rect.y + rect.height - 8
    height = bottom - top
    for x in (x0, center, x1):
      rl.draw_line(int(x), int(top), int(x), int(bottom), grid)
    for n in range(5):
      y = top + height * n / 4
      rl.draw_line(int(x0), int(y), int(x1), int(y), grid)
    # Traverse the fixed ring directly, without copying a frame-sized history array.
    now = self.latest + max(0., time.monotonic() - self.received_at)
    previous = None
    for t, torque, angle in self.history.recent(now):
      y = top + (now - t) / HISTORY_SECONDS * height
      x = center + scaled(torque, TORQUE_SCALE) * half
      rl.draw_line_ex(rl.Vector2(center, y), rl.Vector2(x, y), 3., orange)
      if abs(torque) > TORQUE_SCALE:
        rl.draw_circle(int(x), int(y), 3., rl.WHITE)
      wheel_x = center + scaled(angle, WHEEL_SCALE) * half
      if previous is not None and previous[2] - t < .1:
        rl.draw_line_ex(rl.Vector2(previous[0], previous[1]), rl.Vector2(wheel_x, y), 2., cyan)
      previous = wheel_x, y, t
