"""Route-held-out, free-run identification; scipy-only offline entry point."""
import json
import numpy as np
from pathlib import Path
S = Path("/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad")

TRAIN = ['00000174', '0000018c', '0000019b', '000001a3', '000001a7']
TEST = ['0000019c', '000001aa', '000001ab']


def align(t, a, col, max_age):
  if np.any(np.diff(a[:, 0]) < 0):
    raise ValueError('Alignment requires sorted timestamps')
  ix = np.searchsorted(a[:, 0], t, side='right') - 1
  valid = (ix >= 0) & (t - a[np.maximum(ix, 0), 0] <= max_age)
  return np.where(valid, a[np.maximum(ix, 0), col], np.nan)


def load(key):
  raw = dict(np.load(S/'plant_data'/f'{key}.npz'))
  ix = np.r_[0, np.where(np.diff(np.floor(raw['t']*10)) > 0)[0]+1]
  d = {k: v[ix] for k, v in raw.items()}
  b = np.load(S/'calib'/f'{key}_body.npz')
  for name, col in [('roll', 1), ('angle_offset', 2), ('sr', 3), ('stiffness', 4)]:
    d[name] = align(d['t'], b['vp'], col, 0.5)
  for name, col in [('yaw', 3), ('pitch', 5), ('output', 7)]:
    d[name] = align(d['t'], b['cc'], col, 0.15)
  for name, col in [('des', 1), ('act', 2), ('logoutput', 3), ('sat', 4)]:
    d[name] = align(d['t'], b['torque'], col, 0.15)
  d['body'] = d['v']*d['yaw']
  d['wheel'] = -np.radians(d['angle']-d['angle_offset'])*d['v']**2/(d['sr']*2.9)
  d['ok'] = (d['active'] > 0)&(d['pressed'] == 0)&(abs(d['drv']) < 500)&(d['blinker'] == 0)&(d['v'] > 5)
  d['ok'] &= np.isfinite(np.array([d[k] for k in ['body','roll','wheel','act']])).all(axis=0)
  return d


def windows(d):
  out = []
  # Nonoverlapping 10 s predictions with 2 s continuously eligible context.
  for i in range(20, len(d['t'])-100, 100):
    sl = slice(i-20, i+100)
    if d['ok'][sl].all() and np.max(np.diff(d['t'][sl])) < .16 and len(set(d['seg'][sl])) == 1:
      out.append(np.array([d[k][i:i+100] for k in ('cmd','v','roll','act','body','wheel')]))
  return np.array(out)


def steering(p, w):
  u,v,r,y = w[:, :4].transpose(1,0,2)
  gain = np.interp(v,[10,20,30],p[:3])
  friction = np.interp(v,[10,20,30],p[4:7])
  eff = np.sign(u)*np.maximum(abs(u)-.028,0)
  a = y[:,0].copy()
  pred = [a]
  for j in range(1,100):
    push = gain[:,j-1]*eff[:,j-1]+p[7]*r[:,j-1]+p[8]-a
    a = a+np.sign(push)*np.maximum(abs(push)-friction[:,j-1],0)*(-np.expm1(-.1/p[3]))
    pred.append(a)
  return np.array(pred).T


def body(p, w, y=None):
  v,r = w[:,1],w[:,2]
  y = w[:,3] if y is None else y
  g = np.interp(v,[10,20,30],p[:3])
  a = w[:,4,0].copy()
  pred = [a]
  for j in range(1,100):
    a = a+(g[:,j-1]*y[:,j-1]+p[4]*r[:,j-1]+p[5]-a)*(-np.expm1(-.1/p[3]))
    pred.append(a)
  return np.array(pred).T


def main():
  from scipy.optimize import least_squares
  ds = {k:load(k) for k in TRAIN+TEST}
  ws = {k:windows(d) for k,d in ds.items()}
  w = np.concatenate([ws[k] for k in TRAIN if len(ws[k])])
  w = w[np.random.default_rng(42).choice(len(w),min(len(w),1600),replace=False)]
  ps = least_squares(lambda p:(steering(p,w)[:,1:]-w[:,3,1:]).ravel(),
                     [1.6,2.5,3.7,.87,.4,.4,.4,0,0],
                     bounds=([.1,.1,.1,.04,0,0,0,-30,-1],[10,10,10,3,1.5,1.5,1.5,30,1]),
                     loss='soft_l1',f_scale=.2,max_nfev=100).x
  pb = least_squares(lambda p:(body(p,w)[:,1:]-w[:,4,1:]).ravel(),
                     [1,1,1,.05,0,0], bounds=([.2,.2,.2,.005,-30,-1],[2,2,2,1,30,1]),
                     loss='soft_l1',f_scale=.1,max_nfev=80).x
  result = {'steering_parameters':ps.tolist(),'body_parameters':pb.tolist(),'routes':{}}
  for key,d in ds.items():
    z = ws[key]
    steady = d['ok'] & (abs(d['rate']) < 1) & (abs(d['act']) > .15)
    bands = {}
    for lo,hi in [(5,15),(15,25),(25,40)]:
      m = steady&(d['v']>=lo)&(d['v']<hi)
      x = np.c_[d['wheel'][m], d['roll'][m], np.ones(m.sum())]
      coef = np.linalg.lstsq(x,d['body'][m],rcond=None)[0]
      bands[f'{lo}-{hi}'] = {'n':int(m.sum()),'correlation':float(np.corrcoef(d['act'][m],d['body'][m])[0,1]) if m.sum()>2 else None,
                            'wheel_gain_roll_bias':coef.tolist(),'rmse':float(np.sqrt(np.mean((x@coef-d['body'][m])**2)))}
    a = steering(ps,z) if len(z) else np.empty((0,100))
    b = body(pb,z,a) if len(z) else a
    result['routes'][key] = {'windows':len(z),'roll_std':float(np.nanstd(d['roll'])), 'body_finite_pct':float(np.mean(np.isfinite(d['body']))*100),
      'steady_bands':bands,
      'steering_10s_rmse':float(np.sqrt(np.mean((a[:,1:]-z[:,3,1:])**2))) if len(z) else None,
      'body_10s_rmse':float(np.sqrt(np.mean((b[:,1:]-z[:,4,1:])**2))) if len(z) else None,
      'body_given_logged_steer_rmse':float(np.sqrt(np.mean((body(pb,z)[:,1:]-z[:,4,1:])**2))) if len(z) else None}
    np.savez_compressed(S/'calib'/f'{key}_aligned.npz',**d)
  (S/'calib'/'plant_v2.json').write_text(json.dumps({'schema':2,'steering':ps.tolist(),'body':pb.tolist(),'train':TRAIN,'heldout':TEST},indent=2))
  (S/'calib'/'identification.json').write_text(json.dumps(result,indent=2))
  print(json.dumps(result,indent=2),flush=True)


if __name__ == '__main__':
  main()
