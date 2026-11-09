"""Native 100Hz curvature clipping check, causal logs and generated frame IDs."""
import argparse
import json
from pathlib import Path
from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from tools.odyssey_sim.vsim.run import agreement


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--route', type=Path, required=True)
  ap.add_argument('--traces', type=Path, required=True)
  ap.add_argument('--output', type=Path, required=True)
  args = ap.parse_args()
  frames = {r['frame']: r['curvature'] for r in json.loads(args.traces.read_text())}
  current = None
  speed = roll = 0.
  active = False
  baseline = sim = None
  x, y, z, ts = [], [], [], []
  for seg in sorted(args.route.glob('*--*'), key=lambda p: int(p.name.rsplit('--', 1)[-1])):
    for m in LogReader(str(seg / 'rlog.zst')):
      k = m.which()
      if k == 'carState':
        speed = m.carState.vEgo
      elif k == 'vehicleParameters':
        roll = m.vehicleParameters.roll
      elif k == 'carControl':
        active = m.carControl.latActive
      elif k == 'modelV2':
        md = m.modelV2
        current = (frames[md.frameId], md.action.desiredCurvature) if md.frameId in frames else None
      elif k == 'controlsState':
        cs = m.controlsState
        if current is None or not active:
          baseline = sim = cs.desiredCurvature
          continue
        if baseline is None:
          baseline = sim = cs.desiredCurvature
        baseline, _ = clip_curvature(speed, baseline, current[1], roll)
        sim, _ = clip_curvature(speed, sim, current[0], roll)
        x.append(sim)
        y.append(cs.desiredCurvature)
        z.append(baseline)
        ts.append(m.logMonoTime * 1e-9)
  if not x:
    raise ValueError('no active controls samples matched the traces')
  result = {'generated_vs_logged_controlsd': agreement(x, y), 'logged_model_vs_logged_controlsd': agreement(z, y),
            'generated_vs_logged_model_clip': agreement(x, z), 'duration_s': ts[-1] - ts[0],
            'note': 'Causal log delivery approximation; baseline measures scheduling/maneuver-plan mismatch.'}
  args.output.write_text(json.dumps(result, indent=2))
  print(json.dumps(result, indent=2))


if __name__ == '__main__':
  main()
