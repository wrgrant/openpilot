"""Process-local vsim adapters; never edits vsim or production code."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
from tools.odyssey_sim.plan.compensation import Compensation, inner_lane_confidence


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--mode', choices=['stock', 'lead', 'lane', 'smooth', 'path'], required=True)
  ap.add_argument('--strength', type=float, default=1.)
  ap.add_argument('--reference', type=Path)
  ap.add_argument('--portable-model', type=Path)
  ap.add_argument('--refine', type=Path)
  args, rest = ap.parse_known_args()
  from tools.odyssey_sim.vsim import run
  if args.portable_model:
    from tools.odyssey_sim.plan.models import PortableModel
    run.DefaultModel = PortableModel
    rest += ['--model', str(args.portable_model)]
  if args.mode == 'path':
    from tools.odyssey_wander import bandpass
    route = Path(rest[rest.index('--route')+1])
    captured = run.records(route)
    logged = captured[0]
    origin = logged[0][0]
    t = np.array([ts-origin for ts, _, _ in logged])
    off = -np.array([np.mean([md.laneLines[i].y[0] for i in (1, 2)]) for _, md, _ in logged])
    grid = np.arange(t[0], t[-1], .05)
    shift = -bandpass(np.interp(grid, t, off))
    if args.refine:
      prior = json.loads(args.refine.read_text())
      pt = np.array([r['t'] for r in prior])
      pe = -np.array([np.mean(np.array(r['lane'])[:, 0]) for r in prior])
      freq = np.fft.rfftfreq(len(pt), .05)
      freq = freq[(freq >= 1/20) & (freq <= 1/7)]
      design = np.column_stack([np.ones(len(pt)), pt-pt.mean()] +
                               [fn(2*np.pi*f*pt) for f in freq for fn in (np.sin, np.cos)])
      co = np.linalg.lstsq(design, pe, rcond=None)[0][2:]
      correction = np.column_stack([fn(2*np.pi*f*grid) for f in freq for fn in (np.sin, np.cos)])@co
      shift -= correction
    rate = np.gradient(shift, .05)
    run.records = lambda _: captured
    def path_injection(elapsed, speed, *_):
      return float(np.interp(elapsed, grid, shift)), float(np.interp(elapsed, grid, rate)/max(speed, .1))
    run.injection = path_injection
    rest += ['--shape', 'sine', '--dy', '.01']
  elif args.mode != 'stock':
    original_model = run.DefaultModel
    original_action = run.get_action_from_model
    from tools.odyssey_sim.vsim import closed_loop
    original_loop = closed_loop.ClosedLoop
    comp = Compensation(args.mode, args.strength)
    context = {}
    class Model(original_model):
      def run(self, images, desire, rhd, action_t):
        context['lane_change'] = desire != 0
        return super().run(images, desire, rhd, action_t)
    def action(output, previous, lat_delay, long_delay, speed):
      result = original_action(output, previous, lat_delay, long_delay, speed)
      lane = output['lane_lines'][0, 1:3, :, 0].mean(axis=0)
      x = 192*(np.arange(33)/32)**2
      keep = x <= 25
      co = np.polynomial.polynomial.polyfit(x[keep], lane[keep], 3)
      confidence = inner_lane_confidence(output['lane_lines_prob'][0])
      value = comp.update(result.desiredCurvature*speed**2, speed, -co[0], -np.arctan(co[1]), confidence,
                          .05, enabled=True, lane_change=context['lane_change'])
      context['compensated'] = value/max(speed**2, .01)
      return result
    class Loop(original_loop):
      @property
      def target(self):
        return self._target

      @target.setter
      def target(self, value):
        self._target = context.get('compensated', value) if value is not None else None

    closed_loop.ClosedLoop = Loop
    run.DefaultModel, run.get_action_from_model = Model, action
  sys.argv = ['vsim-plan']+rest
  run.main()


if __name__ == '__main__':
  main()
