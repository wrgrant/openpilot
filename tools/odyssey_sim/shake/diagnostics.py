"""Training-only roll observability and earlier-plant counterfactuals."""
import json
import numpy as np
from scipy import signal, optimize
from tools.odyssey_sim.shake.analyze import OUT, windows, band
from tools.odyssey_sim.shake.extract import SCRATCH
from tools.odyssey_sim.shake.physics import simulate


def main():
  d = np.load(SCRATCH/'000001ac.npz')
  ts = np.arange(d['t'][0]+1, d['t'][-1]-1, .01)
  phi = np.interp(ts, d['orientation_raw'][:, 0], d['orientation_raw'][:, 1])
  ay = np.interp(ts, d['t'], d['v']*d['yaw'])
  v = np.interp(ts, d['t'], d['v'])
  active = np.interp(ts, d['t'], d['lat']) > .5
  phi = band(phi)
  q = signal.savgol_filter(phi, 31, 3, deriv=1, delta=.01)
  qdot = signal.savgol_filter(phi, 31, 3, deriv=2, delta=.01)
  mask = active & (v > 11.17)
  x = np.column_stack([band(ay), -q, -phi])[mask]
  y = qdot[mask]
  unconstrained = np.linalg.lstsq(x, y, rcond=None)[0]
  fit = optimize.lsq_linear(x, y, bounds=([0., .01, (2*np.pi*.5)**2], [10., 50., (2*np.pi*3)**2]))
  beta = fit.x
  hz = np.sqrt(beta[2])/(2*np.pi)
  zeta = beta[1]/(2*np.sqrt(beta[2]))
  f, coh = signal.coherence(band(ay)[mask], phi[mask], fs=100, nperseg=2048)
  result = {'roll_proxy_fit': {'source': 'carControl.orientationNED[0], not an independently calibrated seat/roll sensor',
            'unconstrained_drive_damping_stiffness': unconstrained.tolist(), 'bounded_coefficients': beta.tolist(),
            'natural_times_per_second': float(hz), 'damping_ratio': float(zeta),
            'variance_explained': float(1-np.var(x@beta-y)/np.var(y)),
            'coherence_1_3_per_second': float(coh[np.argmin(abs(f-1.3))]),
            'status': 'diagnostic, not substituted into frozen gate plant; road-bank and frame confounded'}}
  w = max((w for w in windows('000001ac') if w['block'] == 2), key=lambda w: np.std(band(w['truth'])[:, 1]))
  p = np.array(json.loads((OUT/'fit.json').read_text())['parameters'])
  from tools.odyssey_sim.calib.plant import Plant
  old = Plant()
  old.reset(w['truth'][0, 1])
  ys = []
  for i, u in enumerate(w['u']):
    old.step(u, w['v'][i], 0.)
    ys.append(old.body)
  variants = {'full': p.copy(), 'no_friction': p.copy(), 'fast_motor': p.copy(), 'short_relaxation': p.copy(), 'no_roll': p.copy()}
  variants['no_friction'][2] = 0.
  variants['fast_motor'][7] = .024
  variants['short_relaxation'][8] = .05
  variants['no_roll'][11] = 0.
  result['ablation'] = {}
  for key, parameters in variants.items():
    sim = simulate(w['u'], w['v'], parameters, w['initial'])
    result['ablation'][key] = {'yaw_rms': float(np.std(band(sim[:, 1])[500:-100])),
                              'seat_rms': float(np.std(band(sim[:, 5])[500:-100]))}
  result['ablation']['old_scalar_plant'] = {'yaw_rms': float(np.std(band(np.array(ys))[500:-100]))}
  result['ablation']['road'] = {'yaw_rms': float(np.std(band(w['truth'][:, 1])[500:-100]))}
  result['training_window_start'] = float(w['t'][0])
  (OUT/'diagnostics.json').write_text(json.dumps(result, indent=2))
  print(result)


if __name__ == '__main__':
  main()
