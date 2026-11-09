"""Independent raw wheel-to-yaw steady transfer, with explicit understeer and roll."""
import json
import numpy as np
from tools.odyssey_sim.calib.identify import S, TRAIN, TEST, align


def main():
  from scipy.optimize import least_squares
  rows = {}
  axes = {}
  for key in TRAIN+TEST:
    d = dict(np.load(S/'calib'/f'{key}_aligned.npz'))
    good = d['ok']&(abs(d['rate'])<1)&(abs(d['act'])>.15)
    ix = np.where(good)[0][::5]
    rows[key] = np.array([d[k][ix] for k in ['wheel','v','roll','body']])
    cc = np.load(S/'calib'/f'{key}_body.npz')['cc']
    axes[key] = [float(np.corrcoef(d['v'][good]*align(d['t'],cc,j,.15)[good],d['act'][good])[0,1]) for j in (1,2,3)]
  a = np.concatenate([rows[k] for k in TRAIN],axis=1)
  def pred(p,a):
    return a[0]/(1+p[0]*a[1]**2)+p[1]*a[2]+p[2]
  p = least_squares(lambda p:pred(p,a)-a[3],[.0006,3,0],bounds=([0,-30,-1],[.01,30,1]),loss='soft_l1',f_scale=.1).x
  result = {'understeer_coefficient_per_mps2':float(p[0]),'roll_gain':float(p[1]),'bias':float(p[2]),
            'axis_correlations_roll_pitch_yaw':axes,
            'heldout_rmse':{k:float(np.sqrt(np.mean((pred(p,rows[k])-rows[k][3])**2))) for k in TEST}}
  (S/'calib'/'body_fit.json').write_text(json.dumps(result,indent=2))
  print(json.dumps(result,indent=2))


if __name__ == '__main__':
  main()
