"""OFFLINE ONLY: invert rejected plant as a diagnostic, never as a road forecast."""
import json
import numpy as np
from scipy import optimize
from tools.odyssey_sim.shake.analyze import OUT
from tools.odyssey_sim.shake.physics import simulate


def main():
  p = np.array(json.loads((OUT/'fit.json').read_text())['parameters'])
  rows = []
  for hz in (.3, .5, 1., 1.3, 2., 3.):
    t = np.arange(3000)*.01
    carrier = np.sin(2*np.pi*hz*t)

    def force(amplitude, carrier=carrier, t=t, hz=hz):
      y = simulate(amplitude*carrier, np.full(len(t), 26.82), p, np.zeros(8))
      # Harmonic peak rather than total RMS, excluding startup.
      z = y[1000:, 1]
      basis = np.column_stack([np.sin(2*np.pi*hz*t[1000:]), np.cos(2*np.pi*hz*t[1000:]), np.ones(len(z))])
      c = np.linalg.lstsq(basis, z, rcond=None)[0]
      return float(80*np.hypot(c[0], c[1]))

    for target in (.25, .5, 1., 2., 4., 7., 9., 14., 20.):
      # Grid brackets avoid pretending a nonlinear stick-slip response is monotonic.
      grid = np.linspace(0, 1, 41)
      values = np.array([force(x) for x in grid])
      hits = np.flatnonzero((values[:-1]-target)*(values[1:]-target) <= 0)
      amplitude = None
      if len(hits):
        j = hits[0]
        amplitude = float(optimize.brentq(lambda x, target=target: force(x)-target, grid[j], grid[j+1], xtol=1e-5))
      rows.append({'times_per_second': hz, 'peak_newtons_on_80kg': target, 'peak_m_s2': target/80,
                   'unvalidated_command_amplitude': amplitude})
  (OUT/'ladder_diagnostic.json').write_text(json.dumps({'authorized_for_road': False, 'reason': 'Both validation gates failed.',
                                                     'speed_mph': 60, 'rows': rows}, indent=2))
  print(rows, flush=True)


if __name__ == '__main__':
  main()
