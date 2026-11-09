"""Fit a stable PD counterexample to the injected response, without natural regression."""
import numpy as np


def audit(rows):
  w = 2*np.pi/np.array([r['period'] for r in rows])
  h = np.array([r['plan_accel_per_offset']['gain']*np.exp(1j*np.deg2rad(r['plan_accel_per_offset']['phase_deg'])) for r in rows])
  k = -h.real.mean()
  c = -np.dot(w, h.imag)/np.dot(w, w)
  predicted = -k-1j*w*c
  return {'k_per_s2': float(k), 'c_per_s': float(c),
          'ideal_poles': [[float(p.real), float(p.imag)] for p in np.roots([1, c, k])],
          'relative_complex_error': float(np.linalg.norm(h-predicted)/np.linalg.norm(h)),
          'phase_relative_to_restoring_deg': np.angle(-h, deg=True).tolist(),
          'warning': 'Stable ideal PD is a counterexample to raw-phase-as-delay inference, not a validated vehicle-loop model.'}
