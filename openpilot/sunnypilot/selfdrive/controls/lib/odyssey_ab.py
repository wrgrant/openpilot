"""Odyssey road A/B tests: rotate between arms in fixed blocks of the monotonic clock (the clock behind logMonoTime), so
one drive compares them on the same roads and tools/odyssey_wander.py --ab can tell afterwards which arm ran when."""
AB_BLOCK_S = 60.0  # each block holds three or four of the 15-20 s swings the wander score measures

WANDER_BLOCK_S = 180.0  # wander needs long blocks: score the last 150 s of each (the wander worker's A/B plan)

DEADBAND_ARMS = ("off", "kick", "bump")  # SteerDeadbandCompAB
NNLC_ARMS = ("on", "off")  # NnlcAB
WANDER_ARMS = ("off", "inverse")  # OdysseyWanderAB
# NnlcModelAB: stock HONDA_ODYSSEY NNLC vs the van-specific models (report_nnlc_smooth.md), WANDER_BLOCK_S blocks
NNLC_MODEL_ARMS = ("stock", "S3", "S5")
# OdysseyLaneCenterAB: lane-centring correction off/on (odyssey_lane.py), WANDER_BLOCK_S blocks. With NnlcModelAB also on,
# block k runs model arm k % 3 and lane arm k % 2, so every 6 blocks cover all six combinations once.
LANE_ARMS = ("off", "on")
# SteerLowSpeedAngleAB: low-speed angle steering's error gain, K_ERR vs K_ERR_FIRM (low_speed_angle.py), AB_BLOCK_S blocks.
# It acts below 18 mph and the lane and model tests are scored above 25 mph, so they don't disturb each other.
LOWSPEED_ARMS = ("base", "firm")
NNLC_MODEL_FILES = {"S3": "HONDA_ODYSSEY_WILLIAM_S3.json", "S5": "HONDA_ODYSSEY_WILLIAM_S5.json"}


def ab_arm(mono_s: float, arms: tuple[str, ...], block_s: float = AB_BLOCK_S) -> str:
  return arms[int(mono_s // block_s) % len(arms)]


# The historical model clock above remains readable for old drives; manual tests exclude S5.
EXPERIMENTS = {"lane": LANE_ARMS, "bump": DEADBAND_ARMS, "nnlc_model": ("stock", "S3"), "lowspeed": LOWSPEED_ARMS}
TARGET_STRAIGHT, TARGET_CURVE = 300, 60
PARAMS_POLL_S = 0.2  # how often the worker re-reads the experiment and arm: a tap takes effect within this
TOTALS_SAVE_S = 10.0  # totals go to flash this often; the live status carries them to the screen in between


def mem_params(params):
  """The live status is rewritten every second, so it lives in memory (the idiom sunnypilot's map code uses)."""
  import platform
  from openpilot.common.params import Params
  return Params("/dev/shm/params") if platform.system() != "Darwin" else params


def selection(experiment, arm, manual, mono_s):
  if experiment not in EXPERIMENTS:
    return "off"
  arms = EXPERIMENTS[experiment]
  if manual:
    return arms[arms.index(arm)] if arm in arms else arms[0]
  return ab_arm(mono_s, arms, AB_BLOCK_S if experiment in ("bump", "lowspeed") else WANDER_BLOCK_S)


def good_data(experiment, *, active, speed, blinker=False, lane_change=False, since_switch=0.,
              pressed_s=0., lane_probs=(0., 0.), desired_accel=0.):
  """One frame's eligibility, shared by controls and log analysis. Durations are in seconds."""
  import math
  kind = "straight" if abs(desired_accel) < .5 else "curve"
  reason = ""
  if experiment not in EXPERIMENTS:
    reason = "no experiment"
  elif not active:
    reason = "steering inactive"
  elif not all(math.isfinite(x) for x in (speed, since_switch, pressed_s, desired_accel, *lane_probs)):
    reason = "invalid data"
  elif blinker:
    reason = "blinker"
  elif lane_change:
    reason = "lane change"
  elif experiment == "lowspeed" and not 0 < speed < 18 * .44704:
    reason = "outside 0–18 mph"
  elif experiment != "lowspeed" and speed <= 25 * .44704:
    reason = "under 25 mph"
  elif since_switch < 5:
    reason = "5 s after switch"
  elif pressed_s >= 1:
    reason = "you're steering"
  elif experiment in ("lane", "nnlc_model") and min(lane_probs) <= .5:
    reason = "lane lines lost"
  return not reason, reason or "counting", kind


def validate_totals(raw):
  import json
  import math
  try:
    data = json.loads(raw or "{}")
    return {e: {a: {k: float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and 0 <= v < 1e9 else 0.
                    for k in ("straight", "curve") for v in [data.get(e, {}).get(a, {}).get(k, 0)]}
                for a in arms} for e, arms in EXPERIMENTS.items()}
  except (ValueError, TypeError, AttributeError):
    return validate_totals("{}")


def parse_record(raw):
  import json
  try:
    d = json.loads(raw)
    d = d.get("msg", d)
    if d.get("event") != "odyssey_ab":
      return None
    if d.get("experiment") == "" and d.get("arm") == "" and d.get("good") is False:
      return d
    if d.get("experiment") not in EXPERIMENTS:
      return None
    if d.get("arm") not in EXPERIMENTS[d["experiment"]] or type(d.get("good")) is not bool:
      return None
    return d
  except (ValueError, TypeError, AttributeError):
    return None


# A single atomic snapshot is read by controllers. Only the worker touches params/log I/O.
_config = ("", False, "")
_model_arms = {"stock"}


def register_model_arms(arms):
  global _model_arms
  _model_arms = set(arms)


def manual_experiment(name):
  return bool(_config[0]) and _config[1] and _config[0] == name


def clock_enabled(name, enabled):
  return (enabled or _config[0] == name) and not (_config[0] and _config[1])


def runtime_arm(name, mono_s, arms, block_s=AB_BLOCK_S):
  arm = selection(name, _config[2], _config[1], mono_s) if _config[0] == name else ab_arm(mono_s, arms, block_s)
  return "stock" if name == "nnlc_model" and arm not in _model_arms else arm


class LiveAB:
  def __init__(self, params):
    import queue
    import threading
    self.params = params
    self.mem = mem_params(params)
    self.broken = False
    self.stop = threading.Event()
    self.queue = queue.Queue(maxsize=256)
    self.status = None
    self.last = None
    self.switched = 0.
    self.pressed = None
    self.thread = threading.Thread(target=self._worker, daemon=True, name="odyssey-ab")
    self.thread.start()

  def close(self):
    self.stop.set()
    self.thread.join(timeout=.5)

  def _worker(self):
    import json
    import queue
    from openpilot.common.swaglog import cloudlog
    global _config
    import time
    totals = validate_totals(self.params.get("OdysseyABTotals"))
    last_record, last_save, last_log, last_poll = None, 0., 0., -1.
    while not self.stop.is_set():
      if time.monotonic() - last_poll >= PARAMS_POLL_S:  # not once per queued frame: these are file reads
        last_poll = time.monotonic()
        e = self.params.get("OdysseyABExperiment") or ""
        # Invalid nonempty experiments select the safe first arm of lane.
        arm = self.params.get("OdysseyABArm") or ""
        manual = self.params.get_bool("OdysseyABManual")
        if e and e not in EXPERIMENTS:
          e, arm, manual = "lane", "", True
        _config = (e, manual, arm)
        if self.params.get_bool("OdysseyABReset"):
          totals = validate_totals("{}")
          self.params.put_bool("OdysseyABReset", False)
          self.params.put("OdysseyABTotals", json.dumps(totals))
      try:
        record, dt = self.queue.get(timeout=.1)
      except queue.Empty:
        continue
      if record["good"]:
        totals[record["experiment"]][record["arm"]][record["kind"]] += dt
      key = tuple(record[k] for k in ("experiment", "arm", "manual", "good", "reason", "kind"))
      now = record["mono_s"]
      if key != last_record or now - last_log >= 1:
        cloudlog.event("odyssey_ab", **record)
        self.mem.put("OdysseyABStatus", json.dumps({**record, "totals": totals.get(record["experiment"], {})}))
        last_record, last_log = key, now
      if now - last_save >= TOTALS_SAVE_S or not record["experiment"]:
        self.params.put("OdysseyABTotals", json.dumps(totals))
        last_save = now

  def update(self, now, cs, md, active, desired_accel):
    """Called from the 100 Hz controls loop. Counting is bookkeeping: a fault here must never stop steering, so it
    stops the counting instead. Arm selection does not depend on it."""
    if self.broken:
      return
    try:
      self._update(now, cs, md, active, desired_accel)
    except Exception:
      self.broken = True
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception("odyssey_ab: counting stopped")

  def _update(self, now, cs, md, active, desired_accel):
    import queue
    e, manual, arm = _config
    if not e:
      self.last = None
      if self.status is not None and self.status["experiment"]:
        self.status = {"experiment": "", "arm": "", "manual": False, "good": False,
                       "reason": "no experiment", "kind": "straight", "mono_s": now}
        try:
          self.queue.put_nowait((self.status, 0.))
        except queue.Full:
          pass
      return
    arm = runtime_arm(e, now, EXPERIMENTS[e], AB_BLOCK_S if e in ("bump", "lowspeed") else WANDER_BLOCK_S)
    previous = self.status
    if previous is None or (e, arm, manual) != (previous["experiment"], previous["arm"], previous["manual"]):
      self.switched = now
    self.pressed = (now if self.pressed is None else self.pressed) if cs.steeringPressed else None
    probs = md.laneLineProbs
    good, reason, kind = good_data(e, active=active, speed=cs.vEgo, blinker=cs.leftBlinker or cs.rightBlinker,
                                  lane_change=str(md.meta.laneChangeState) not in ("off", "0"), since_switch=now-self.switched,
                                  pressed_s=now-self.pressed if self.pressed is not None else 0.,
                                  lane_probs=(probs[1], probs[2]) if len(probs) == 4 else (0., 0.), desired_accel=desired_accel)
    dt = now-self.last if self.last is not None and 0 < now-self.last <= .02 else 0.
    self.last = now
    self.status = {"experiment": e, "arm": arm, "manual": manual, "good": good, "reason": reason, "kind": kind, "mono_s": now}
    try:
      self.queue.put_nowait((self.status, dt))
    except queue.Full:
      pass  # Missing telemetry is never credited as good data.
