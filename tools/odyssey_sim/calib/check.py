"""Numerical API checks, frozen-output held-out validation and lane closure audit."""
import json
import numpy as np
from tools.odyssey_sim.calib.identify import S, TEST, windows
from tools.odyssey_sim.calib.plant import Plant
from tools.odyssey_sim.calib.evaluate import records, pieces


def main():
  result = {'api':{},'lane_closure':[]}
  for speed in (5,10,20,30,35):
    for command in (-1,-.04,0,.04,1):
      plant = Plant()
      for _ in range(2000):
        plant.step(command,speed,0)
      assert np.isfinite([plant.y,plant.body,plant.u]).all()
      assert abs(plant.y)<10 and plant.y*command>=-.05
  result['api']['finite_bounded_grid'] = True
  for key in TEST:
    d = dict(np.load(S/'calib'/f'{key}_aligned.npz'))
    es,eb = [],[]
    for w in windows(d):
      plant = Plant()
      plant.reset(w[3,0],w[4,0],w[0,0])
      for j in range(1,100):
        for _ in range(10):
          plant.step(w[0,j-1],w[1,j-1],w[2,j-1])
        es.append(plant.y-w[3,j])
        eb.append(plant.body-w[4,j])
    result['api'][key] = {'steering_10s_rmse':float(np.sqrt(np.mean(np.square(es)))),
                         'body_10s_rmse':float(np.sqrt(np.mean(np.square(eb))))}
  for route,d in records():
    for j,r in enumerate(pieces(d)):
      # Low-frequency closure in exactly the wander meter's band; numerical
      # differentiation there avoids noisy frame-to-frame camera differences.
      e = d['offset'][r]
      freq = np.fft.rfftfreq(len(r),.1)
      def band(x, freq=freq):
        x = x-np.polyval(np.polyfit(np.arange(len(x)),x,1),np.arange(len(x)))
        z = np.fft.rfft(x)
        z[(freq<1/20)|(freq>1/7)] = 0
        return z
      lane_dd = -(2*np.pi*freq)**2*band(e)
      inertial_dd = band(d['road'][r]-d['body'][r])
      residual = np.fft.irfft(inertial_dd-lane_dd,len(r))
      result['lane_closure'].append({'route':route,'window':j,
        'road_minus_body_vs_lane_dd_rmse':float(np.std(residual)),
        'lane_dd_rms':float(np.std(np.fft.irfft(lane_dd,len(r)))),
        'accel_residual_equiv_wander_cm':float(100*np.std(np.fft.irfft(
          np.divide(inertial_dd-lane_dd,-(2*np.pi*freq)**2,out=np.zeros_like(lane_dd),where=freq>0),len(r))))})
  (S/'calib'/'checks.json').write_text(json.dumps(result,indent=2))
  print(json.dumps(result,indent=2))


if __name__ == '__main__':
  main()
