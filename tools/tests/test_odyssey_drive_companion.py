import io
import json
from pathlib import Path
import tempfile
import unittest

from tools.odyssey_drive_companion import LocalDevice, run


class TestCompanion(unittest.TestCase):
  def test_pass_and_drop(self):
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp)
      route = root/'source'/'route'
      for n in range(3):
        p = route/f'route--{n}'/'rlog.zst'
        p.parent.mkdir(parents=True)
        p.write_bytes(b'synthetic placeholder')
      device = LocalDevice(root/'source', statuses=[{}, {}, {}, {'state': 'done'}], drop_at={2})
      def analysis(paths):
        from tools.odyssey_eps_report import summarize
        s = summarize([])
        s['coverage'][0]['good'] = len(list(Path(paths[0]).glob('**/rlog.zst')))
        return s
      output = io.StringIO()
      result = run(device, root/'out', poll_seconds=0, analyzer=analysis, stream=output)
      self.assertEqual(result['exit_reason'], 'pass complete')
      self.assertEqual(len(list((root/'out').glob('**/rlog.zst'))), 3)
      self.assertGreaterEqual(len(set(device.notes)), 3)
      self.assertIn('link unavailable', output.getvalue())

  def test_fault(self):
    with tempfile.TemporaryDirectory() as tmp:
      device = LocalDevice(Path(tmp), statuses=[{'aborts': {'frame gap': 5}}])
      result = run(device, Path(tmp)/'out', poll_seconds=0, stream=io.StringIO())
      self.assertEqual(result['exit_reason'], 'fault pattern: frame gap')
      self.assertEqual(device.params['OdysseyPulsePaused'], '1')
      json.dumps(result, allow_nan=False)


def synthetic_drive():
  """Known independent plant outputs for the actual 26-shot input plan."""
  from dataclasses import asdict
  import numpy as np
  from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_pulse import session, LEAD_S, TAIL_S
  records, samples = [], []
  for number, (name, c) in enumerate(session()):
    start = 4*number+1.
    duration = round(LEAD_S+c.duration+TAIL_S, 2)
    t = np.arange(round(duration*100))*.01
    u = np.array([c.command(x) for x in t])
    motor = 112*np.maximum(abs(u)-.028, 0)
    rate = np.zeros(len(t))
    if c.kind == 'ramp':
      rate[abs(u) >= .05-1e-9] = c.direction*2
    elif c.kind == 'pair':
      rate[u*c.direction > 0] = c.direction*2
      rate[u*c.direction < 0] = -c.direction*.8
    elif c.kind == 'chirp':
      for k in range(2, len(t)):
        rate[k] = 1.7910074975*rate[k-1]-.81*rate[k-2]+.02*u[k]
    elif c.kind == 'train':
      rate = u*5
      avg = c.height*c.duty
      desired = 112*max(avg-.028, 0)*.5
      active = (t >= .2-1e-9) & (t < .2+c.duration-1e-9)
      motor[active] = desired
    angle = np.cumsum(rate)*.01
    accel = angle*.02
    s = np.column_stack((t+start, np.round(angle, 1), rate, motor, accel, u, t*0, t*0, u))
    samples.extend(s)
    records.append({'id': '0:'+name, 'start': start, 'end': start+duration, 'completed': True,
                    'speed': 25., 'band': '45-70', 'condition': asdict(c)})
  return records, np.asarray(samples)


def write_synthetic_route(root, fault=False):
  """Write real compressed cereal logs through the production reader seam."""
  import numpy as np
  import zstandard as zstd
  from opendbc.can import CANPacker
  from openpilot.cereal import log
  from tools.odyssey_eps_report import DBC
  records, samples = synthetic_drive()
  packer = CANPacker(DBC)
  buffers = [bytearray() for _ in range(3)]
  def emit(which, now, values, part):
    msg = log.Event.new_message()
    msg.logMonoTime = round(now*1e9)
    if which == 'logMessage':
      msg.logMessage = json.dumps({'event': 'odyssey_pulse', **values})
    else:
      msg.init(which)
      getattr(msg, which).from_dict(json.loads(json.dumps(values)))
    buffers[part].extend(msg.to_bytes())
  for k, record in enumerate(records):
    part = min(k//9, 2)
    emit('logMessage', record['start'], {**record, 'phase': 'start'}, part)
    s = samples[(samples[:, 0] >= record['start']) & (samples[:, 0] < record['end'])]
    for t, angle, rate, motor, accel, u, sensor, _offset, _request in s:
      emit('carControl', t, {'angularVelocity': [0., 0., accel/25.], 'actuators': {'torque': u}}, part)
      emit('carOutput', t, {'actuatorsOutput': {'torque': u}}, part)
      address, dat, bus = packer.make_can_msg('STEER_MOTOR_TORQUE', 0, {'MOTOR_TORQUE': round(motor)})
      msg = log.Event.new_message()
      msg.logMonoTime = round(t*1e9)
      msg.init('can', 1)
      msg.can[0] = {'address': address, 'dat': dat, 'src': bus}
      buffers[part].extend(msg.to_bytes())
      emit('carState', t, {'vEgo': 25., 'steeringAngleDeg': angle, 'steeringRateDeg': rate, 'steeringTorque': sensor}, part)
    emit('logMessage', record['end'], record, part)
  if fault:
    records = records[-5:]
    for r in records:
      r = {**r, 'completed': False, 'abort': 'frame gap'}
      emit('logMessage', r['end']+1, {**r, 'id': 'fault:'+r['id'], 'start': r['start']+200, 'end': r['end']+200}, 2)
  for i, data in enumerate(buffers):
    p = Path(root)/f'synthetic--{i}'/'rlog.zst'
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_bytes(zstd.ZstdCompressor().compress(data))
  return len(np.asarray(samples))


class TestRecordedCompanion(unittest.TestCase):
  def test_actual_rlogs_pass_with_dropped_link(self):
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp)
      write_synthetic_route(root/'source')
      device = LocalDevice(root/'source', drop_at={2})
      result = run(device, root/'out', poll_seconds=0, stream=io.StringIO(), max_minutes=1)
      self.assertEqual(result['exit_reason'], 'pass complete')
      self.assertEqual(result['clean'], 26)
      self.assertEqual(result['pulled_segments'], 3)
      self.assertGreaterEqual(len(set(device.notes)), 3)

  def test_actual_rlogs_fault(self):
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp)
      write_synthetic_route(root/'source', fault=True)
      device = LocalDevice(root/'source')
      result = run(device, root/'out', poll_seconds=0, stream=io.StringIO(), max_minutes=1)
      self.assertEqual(result['exit_reason'], 'fault pattern: frame gap')
      self.assertEqual(device.params['OdysseyPulsePaused'], '1')

  def test_deadlines_and_write_allowlist(self):
    from tools.odyssey_drive_companion import validate_write
    class Clock:
      now = 0.
      def time(self):
        return self.now
      def sleep(self, seconds):
        self.now += seconds
    with tempfile.TemporaryDirectory() as tmp:
      c = Clock()
      result = run(LocalDevice(tmp), Path(tmp)/'out', poll_seconds=60, clock=c.time, sleep=c.sleep, stream=io.StringIO(), wait_minutes=10)
      self.assertEqual(result['exit_reason'], 'no drive started', "with no drive on the device it waits, then says so")
      self.assertGreaterEqual(c.now, 600)
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp)
      (root/'source'/'route--0').mkdir(parents=True)
      (root/'source'/'route--0'/'rlog.zst').write_bytes(b'x')
      c = Clock()
      from tools.odyssey_eps_report import summarize
      result = run(LocalDevice(root/'source'), root/'out', poll_seconds=60, clock=c.time, sleep=c.sleep, stream=io.StringIO(),
                   analyzer=lambda paths: summarize([]))
      self.assertEqual(result['exit_reason'], 'nothing new for three minutes', "once a drive is being followed, silence ends the run")
      self.assertTrue((root/'out'/'local_route'/'route--0'/'rlog.zst').exists(), "logs land in <dongle>_<route>/<segment>/")
    with tempfile.TemporaryDirectory() as tmp:
      c = Clock()
      result = run(LocalDevice(tmp), Path(tmp)/'out', max_minutes=.1, poll_seconds=3,
                   clock=c.time, sleep=c.sleep, stream=io.StringIO())
      self.assertEqual(result['exit_reason'], 'max minutes reached')
    for name, value in [('OdysseyPulsePaused', '0'), ('OtherParam', '1'), ('OdysseyPulseQueue', '[{}]')]:
      with self.assertRaises((ValueError, TypeError)):
        validate_write(name, value)

  def test_queue_never_overwrites_outstanding_and_restart_does_not_duplicate(self):
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp)
      write_synthetic_route(root/'source')
      device = LocalDevice(root/'source')
      run(device, root/'out', queue=True, poll_seconds=0, stream=io.StringIO(), max_minutes=1)
      queued = device.params.get('OdysseyPulseQueue')
      self.assertIsNotNone(queued)
      self.assertLessEqual(len(json.loads(queued)), 6)
      self.assertTrue((root/'out'/'companion-state.json').exists())
      again = LocalDevice(root/'source')
      run(again, root/'out', queue=True, poll_seconds=0, stream=io.StringIO(), max_minutes=1)
      self.assertNotIn('OdysseyPulseQueue', again.params)
