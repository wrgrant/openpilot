"""Free-run wheel prediction driven by measured signed CAN motor torque."""
import json
import numpy as np
from tools.odyssey_sim.shake.identify2 import route, pieces, save
from tools.odyssey_sim.shake.loop2 import Campaign, step
from tools.odyssey_sim.shake.analyze import OUT, band


def main():
  c = Campaign()
  fits = json.loads((OUT/'column2.json').read_text())['fits']
  rows = []
  for name in ('000001ad', '000001ae'):
    for w in pieces(route(name)):
      for fit in fits:
        state = np.zeros(9)
        state[0], state[3] = w['theta'][0], w['yaw'][0]
        angles = []
        for i in range(2000):
          motor = .05*np.sign(w['cmd'][i])*w['motor'][i]
          state = step(0., w['v'][i], 0., state, c.yaw[w['speed']], np.array(fit['parameters']), c.ep,
                       fit['kind'], fit['d'], fit['vs'], motor)
          angles.append(state[0])
        a, b = band(w['theta'])[500:1900], band(np.array(angles))[500:1900]
        rows.append({'route': name, 'speed': w['speed'], 'friction': fit['label'],
                     'wheel_corr': float(np.corrcoef(a, b)[0, 1]), 'rms_ratio': float(np.std(b)/np.std(a)),
                     'wheel_rmse_deg': float(np.degrees(np.sqrt(np.mean((a-b)**2))))})
  save('column_check2.json', {'rows': rows})


if __name__ == '__main__':
  main()
