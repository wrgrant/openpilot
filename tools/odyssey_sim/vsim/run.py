"""Real-model replay, consistent perturbations and research-only free-running closure."""

import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import time
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from tools.odyssey_sim.vsim.video import ModernFrameReader
from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive.modeld.modeld import get_action_from_model
from openpilot.cereal import log
from tools.odyssey_sim.vsim.geometry import homography
from tools.odyssey_sim.vsim.model import DefaultModel, prepare
from tools.odyssey_sim.vsim.perturbation import injection
from types import SimpleNamespace as NS
from tools.odyssey_sim.vsim.metrics import perception_metrics, closure_metrics, curve_scores, lane_scores, logged_lane_mask


def records(route):
  """Snapshot causal SubMaster-like inputs; encode indexes use explicit segmentId."""
  state, rows, indexes, counts, params = {}, [], {'narrow': {}, 'wide': {}}, Counter(), {}
  for seg in sorted(route.glob('*--*'), key=lambda p: int(p.name.rsplit('--', 1)[-1])):
    p = seg / 'rlog.zst'
    if not p.exists():
      continue
    for m in LogReader(str(p)):
      k = m.which()
      counts[k] += 1
      if k == 'initData' and not params:
        params = {e.key: bytes(e.value) for e in m.initData.params.entries}
      if k in ('narrowRoadEncodeIdx', 'wideRoadEncodeIdx'):
        i = getattr(m, k)
        cam = 'narrow' if k.startswith('narrow') else 'wide'
        indexes[cam][i.frameId] = (seg, i.segmentId, i.timestampEof)
      if k == 'modelV2':
        rows.append((m.logMonoTime * 1e-9, m.modelV2, state.copy()))
      elif k in (
        'extrinsicsCalibration',
        'deviceState',
        'narrowRoadCameraState',
        'carState',
        'lateralDelay',
        'carParams',
        'driverMonitoringState',
        'drivingModelData',
        'carControl',
        'controlsState',
      ):
        state[k] = getattr(m, k)
  return rows, indexes, counts, params


def agreement(a, b):
  a, b = np.asarray(a), np.asarray(b)
  return {
    'n': len(a),
    'r': float(np.corrcoef(a.ravel(), b.ravel())[0, 1]) if np.std(a) > 0 and np.std(b) > 0 else None,
    'rmse': float(np.sqrt(np.mean((a - b) ** 2))),
    'logged_rms': float(np.sqrt(np.mean(b**2))),
  }


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--route', required=True, type=Path)
  ap.add_argument('--controller', default='logged', help='logged for gate 1; module:factory reserved for gated closure')
  ap.add_argument('--window', default='0:60', help='seconds from first model output; replay always warms from start')
  ap.add_argument('--dy', type=float, default=0.0)
  ap.add_argument('--dpsi', type=float, default=0.0, help='degrees, positive right')
  ap.add_argument('--output', type=Path, help='optional evidence JSON and frame traces')
  ap.add_argument('--model', type=Path, default=Path('openpilot/selfdrive/modeld/models/driving_supercombo.onnx'))
  ap.add_argument('--period', type=float, default=10.0)
  ap.add_argument('--shape', choices=['sine', 'ramp', 'step'])
  ap.add_argument('--scratch', type=Path)
  ap.add_argument('--sensitivity', choices=['gain', 'friction', 'lag'])
  ap.add_argument('--scale', type=float, default=1.0)
  args = ap.parse_args()
  if args.period <= 0:
    ap.error('period must be positive')
  if args.controller != 'logged' and (args.scratch is None or args.shape):
    ap.error('closure requires --scratch and cannot combine with external injections')
  if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
  if args.controller != 'logged' and ':' not in args.controller:
    ap.error('controller must be logged or module:factory')
  lo, hi = map(float, args.window.split(':'))
  if not 0 <= lo < hi:
    ap.error('window must be 0 <= start < end')
  os.environ['PATH'] = '/opt/homebrew/bin:' + os.environ['PATH']
  start = time.perf_counter()
  rows, indexes, counts, params = records(args.route)
  selected_params = {
    k: v.decode(errors='replace')
    for k, v in params.items()
    if k
    in (
      'CameraOffset',
      'LagdToggle',
      'LagdValueCache',
      'NeuralNetworkLateralControl',
      'NnlcModelOverride',
      'NnlcModelAB',
      'NnlcAB',
      'ModelRunnerTypeCache',
      'ModelManager_ActiveBundle',
      'ModelManager_ActiveBundleChestnut',
    )
  }
  if any(params.get(k, b'') not in (b'', b'null', b'{}') for k in ('ModelManager_ActiveBundle', 'ModelManager_ActiveBundleChestnut')):
    ap.error('custom bundle needs its own model adapter')
  if float(params.get('CameraOffset', b'0')) != 0:
    ap.error('nonzero CameraOffset needs the logged shear; refusing silent mismatch')
  if not rows:
    ap.error('route has no modelV2')
  loop = None
  if args.controller != 'logged':
    from tools.odyssey_sim.vsim.closed_loop import ClosedLoop

    loop = ClosedLoop(args.route, args.controller, args.scratch, (args.sensitivity, args.scale) if args.sensitivity else None, rows[0][0] + lo)
  model = DefaultModel(args.model)
  readers, traces = {}, []
  timers = Counter()
  skipped = Counter()
  previous_action = log.ModelDataV2.Action()
  last_frame = None
  consecutive = 0
  origin = rows[0][0]
  failure = None
  for timestamp, md, state in rows:
    elapsed = timestamp - origin
    if elapsed >= hi:
      break
    if elapsed < max(0, lo - 15):
      continue
    pair = [indexes[c].get(fid) for c, fid in [('narrow', md.frameId), ('wide', md.frameIdExtra)]]
    if any(p is None for p in pair):
      skipped['missing_encode_index'] += 1
      continue
    if any(not (p[0] / name).exists() for p, name in zip(pair, ['fcamera.hevc', 'ecamera.hevc'], strict=True)):
      skipped['missing_video'] += 1
      continue
    if any(k not in state for k in ('extrinsicsCalibration', 'deviceState', 'narrowRoadCameraState', 'carState', 'lateralDelay', 'carParams')):
      skipped['missing_context'] += 1
      continue
    speed = state['carState'].vEgo
    dy, dpsi = args.dy, np.deg2rad(args.dpsi)
    if args.shape:
      dy, dpsi = injection(elapsed, speed, args.dy, args.period, args.shape)
    if loop:
      loop_start = time.perf_counter()
      try:
        loop.advance(timestamp)
      except ValueError as e:
        failure = str(e)
        break
      timers['controller_plant'] += time.perf_counter() - loop_start
      dy, dpsi = loop.pose.dy, loop.pose.dpsi
    calib = state['extrinsicsCalibration']
    dc = DEVICE_CAMERAS[(str(state['deviceState'].deviceType), str(state['narrowRoadCameraState'].sensor))]
    images = []
    for j, (p, name, cam) in enumerate(zip(pair, ['fcamera.hevc', 'ecamera.hevc'], [dc.narrow_road, dc.wide_road], strict=True)):
      path = p[0] / name
      t = time.perf_counter()
      if path not in readers:
        readers[path] = ModernFrameReader(str(path))
      reader = readers[path]
      raw = reader.get(p[1])
      timers['decode'] += time.perf_counter() - t
      t = time.perf_counter()
      warp = get_warp_matrix(np.array(calib.rpyCalib), cam.intrinsics, bool(j))
      if dy or dpsi:
        # Conjugate the calibrated-level view transform into the sensor view.
        a = view_frame_from_device_frame @ rot_from_euler(calib.rpyCalib) @ view_frame_from_device_frame.T
        h = homography(cam.intrinsics, a, calib.height[0], dy, dpsi)
        warp = np.linalg.inv(h) @ warp
      images.append(prepare(raw, reader.w, reader.h, warp))
      timers['warp'] += time.perf_counter() - t
    consecutive = consecutive + 1 if last_frame is not None and 0 < md.frameId - last_frame <= 3 else 0
    last_frame = md.frameId
    speed = state['carState'].vEgo
    delay = state['lateralDelay'].lateralDelay
    desire = 0
    if 'drivingModelData' in state and str(state['drivingModelData'].meta.laneChangeState) in ('laneChangeStarting', 'laneChangeFinishing'):
      desire = 3 if str(state['drivingModelData'].meta.laneChangeDirection) == 'left' else 4
    rhd = bool(state['driverMonitoringState'].isRHD) if 'driverMonitoringState' in state else False
    output = model.run(np.stack(images), desire, rhd, [delay + 0.075, state['carParams'].longitudinalActuatorDelay + 0.375])
    if 'carControl' in state and not state['carControl'].latActive and 'controlsState' in state:
      previous_action.desiredCurvature = state['controlsState'].curvature
    action = get_action_from_model(output, previous_action, delay + 0.075, state['carParams'].longitudinalActuatorDelay + 0.375, speed)
    previous_action = action
    if loop:
      plan = output['plan'][0]
      loop.target = action.desiredCurvature
      loop.plan = NS(acceleration=NS(y=plan[:, 7]), orientation=NS(x=plan[:, 9], y=plan[:, 10]))
    if elapsed >= lo:
      traces.append(
        {
          't': elapsed,
          'dy': float(dy),
          'dpsi': float(dpsi),
          'frame': md.frameId,
          'extra': md.frameIdExtra,
          'speed': speed,
          'warm_frames': consecutive,
          'timestamp_error_ms': (pair[0][2] - md.timestampEof) / 1e6,
          'curvature': action.desiredCurvature,
          'logged_curvature': md.action.desiredCurvature,
          'lane_prob': output['lane_lines_prob'][0, 1::2].tolist(),
          'logged_lane_prob': list(md.laneLineProbs),
          'lane': output['lane_lines'][0, 1:3, :, 0].tolist(),
          'logged_lane': [list(md.laneLines[i].y) for i in (1, 2)],
          'path': output['plan'][0, :, 1].tolist(),
          'logged_path': list(md.position.y),
        }
      )
    if len(traces) % 200 == 0 and traces:
      print(f'{len(traces)} frames, t={elapsed:.1f}s', flush=True)
  if not traces and loop:
    result = {'closure_pass': False, 'failure': failure, 'controller_identity': loop.identity}
    if args.output:
      args.output.write_text(json.dumps(result, indent=2))
      args.output.with_suffix('.motion.json').write_text(json.dumps(loop.history))
    print(json.dumps(result, indent=2))
    return
  if not traces:
    ap.error(f'no paired frames; skipped={dict(skipped)}')
  eligible = [r for r in traces if r['speed'] > 5 and r['warm_frames'] >= 200]
  metrics = {k: agreement([r[k] for r in traces], [r['logged_' + k] for r in traces]) for k in ('curvature', 'lane', 'path')}
  moving_metrics = {k: agreement([r[k] for r in eligible], [r['logged_' + k] for r in eligible]) for k in ('curvature', 'lane', 'path')} if eligible else None
  result = {
    'moving_metrics': moving_metrics,
    'route': str(args.route),
    'window': [lo, hi],
    'params': selected_params,
    'model_sha256': hashlib.sha256(args.model.read_bytes()).hexdigest(),
    'checkpoint': model.metadata.get('model_checkpoint'),
    'injection': {'dy': args.dy, 'dpsi_deg': args.dpsi, 'shape': args.shape, 'period': args.period},
    'controller': args.controller,
    'controller_identity': loop.identity if loop else None,
    'metrics': metrics,
    'gate1_pass': loop is None
    and not (args.dy or args.dpsi or args.shape)
    and moving_metrics is not None
    and moving_metrics['curvature']['r'] is not None
    and moving_metrics['curvature']['r'] > 0.99,
    'ranking_authorized': False,
    'closure_failure': failure,
    'closure_metrics': closure_metrics(traces, loop.history) if loop else None,
    'closure_pass': False if loop else None,
    'timers_s': dict(timers, model=model.seconds, total=time.perf_counter() - start),
    'model_fps': model.frames / model.seconds,
    'processed_frames': model.frames,
    'pipeline_fps': model.frames / (sum(timers.values()) + model.seconds),
    'perception_diagnostics': perception_metrics(traces),
    'skipped': dict(skipped),
    'message_counts': dict(counts),
  }
  if loop and traces and loop.history:
    result['matched_curve_metrics'] = curve_scores(traces, loop.history, loop.d, origin)
    if len(traces) >= 3:
      key = args.route.name.split('_')[1].split('--')[0]
      result['canonical_lane_metrics'] = lane_scores(traces, loop.history, logged_lane_mask(traces, key, origin))
    result['legacy_closure_metrics_note'] = 'closure_metrics is the legacy unmasked diagnostic; canonical_lane_metrics applies visibility/continuity gates.'
  if loop and args.output:
    args.output.with_suffix('.motion.json').write_text(json.dumps(loop.history))
  if args.output:
    args.output.write_text(json.dumps(result, indent=2))
    args.output.with_suffix('.traces.json').write_text(json.dumps(traces))
  print(json.dumps(result, indent=2))


if __name__ == '__main__':
  main()
