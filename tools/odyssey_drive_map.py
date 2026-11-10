#!/usr/bin/env python3
"""Map recorded GPS, arm and shared eligibility. OSM tiles and Leaflet require network access."""
import argparse
import itertools
import json
from pathlib import Path

import numpy as np

from tools.odyssey_wander import load, good_frames, logged_arm, bandpass
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import EXPERIMENTS, NNLC_MODEL_ARMS


def road_matched(points):
  """Pair 50 m cells, heading and speed; resample drive-pair means rather than adjacent GPS fixes."""
  bins = {}
  lon_scale = 111320 * np.cos(np.radians(np.mean([p["lat"] for p in points]))) if points else 111320
  for p in points:
    if not p['good'] or not np.isfinite(p['push']):
      continue
    key = (round(p['lat'] * 111320 / 50), round(p['lon'] * lon_scale / 50), round(p['heading'] / 30) % 12,
           int(p['speed'] // 5), p.get('experiment', ''))
    bins.setdefault(key, {}).setdefault(p['arm'], {}).setdefault(p['route'], []).append(p['push'])
  arms = sorted({p['arm'] for p in points})
  out = []
  rng = np.random.default_rng(0)
  for a, b in itertools.combinations(arms, 2):
    clusters, cells = {}, 0
    for v in bins.values():
      if a not in v or b not in v:
        continue
      cells += 1
      for ra, va in v[a].items():
        for rb, vb in v[b].items():
          clusters.setdefault(tuple(sorted((ra, rb))), []).append(np.mean(vb) - np.mean(va))
    delta = np.array([np.mean(v) for v in clusters.values()])
    interval = None
    if len(delta) >= 2:
      boot = np.mean(rng.choice(delta, (2000, len(delta))), axis=1)
      interval = np.percentile(boot, [2.5, 97.5]).tolist()
    out.append({'pair': f'{b} minus {a}', 'cells': cells, 'drive_pairs': len(delta),
                'difference_N': float(delta.mean()) if len(delta) else None, 'interval_N': interval})
  return out


def points_for(route, experiment):
  d = load(str(route))
  experiment = next((r['experiment'] for _, r in d['records'] if r['experiment'] in EXPERIMENTS), experiment)
  arms = NNLC_MODEL_ARMS if experiment == 'nnlc_model' and not d['records'] else EXPERIMENTS[experiment]
  block = 60 if experiment in ('bump', 'lowspeed') else 180
  good = good_frames(d, experiment, arms, block)
  t = d['t']
  finite = np.isfinite(d['body'])
  body = np.interp(t, t[finite], d['body'][finite]) if finite.any() else np.zeros(len(t))
  fast = bandpass(body, (.3, 3.))
  rows = []
  for i in range(0, len(t), 20):
    if not np.isfinite(d['latitude'][i]) or not np.isfinite(d['longitude'][i]):
      continue
    w = fast[max(0, i-50):i+50]
    # Typical peak over five seconds, on an 80 kg person.
    push = float(80 * np.percentile(np.abs(w), 95))
    changes = float(np.count_nonzero(np.diff(np.sign(w))) / (len(w) / 20))
    ok, reason, kind = good[i]
    rows.append({'experiment': experiment, 'route': route.name, 't': float(t[i]), 'lat': float(d['latitude'][i]), 'lon': float(d['longitude'][i]),
                     'heading': float(d['heading'][i]), 'speed': float(d['v'][i]*2.23694), 'arm': logged_arm(d, t[i], arms, block),
                     'good': ok, 'reason': reason, 'kind': kind, 'push': push, 'changes': changes})
  return rows


def render(points):
  summary = {}
  for p in points:
    a = summary.setdefault(p['arm'], {'seconds': 0, 'straight': 0, 'curve': 0, 'pushes': [], 'changes': []})
    a['seconds'] += 1
    if p['good']:
      a[p['kind']] += 1
    a['pushes'].append(p['push'])
    a['changes'].append(p['changes'])
  table = ''.join(f"<tr><td>{arm}</td><td>{v['seconds']/60:.1f}</td><td>{v['straight']}</td><td>{v['curve']}</td>"
                  + f"<td>{np.median(v['pushes']):.1f}</td><td>{max(v['pushes']):.1f}</td><td>{np.mean(v['changes']):.2f}</td></tr>"
                  for arm, v in sorted(summary.items()))
  html = '''<!doctype html><meta charset="utf-8"><title>Odyssey drive evidence</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.css">
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.js"></script>
<style>body{font:18px system-ui;margin:20px;background:#151b24;color:white}#map{height:70vh}td,th{padding:10px}select{font:inherit}</style>
<h1>Odyssey drive evidence</h1><label>Colour: <select id="mode"><option>arm</option><option>push</option>
<option>speed</option><option>counted</option></select></label>
<span id="legend"></span><div id="map"></div><p>Tap a road segment for arm, push, speed and exclusion reason.
Push is the 95th percentile of |0.3–3 Hz lateral acceleration| over 5 s × 80 kg.</p>
<table><tr><th>Arm</th><th>Minutes mapped</th><th>Straight s</th><th>Curve s</th>
<th>Typical N</th><th>Worst N</th><th>Changes/s</th></tr>''' + table + '''
</table><h2>Road matched push differences</h2><pre>''' + json.dumps(road_matched(points), indent=2) + '''
</pre><script>
const pts=DATA;const map=L.map('map');L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{attribution:'© OpenStreetMap contributors'}).addTo(map);
const layer=L.layerGroup().addTo(map),colors=['#4dd0e1','#ffb74d','#ce93d8','#81c784'];
const arms=[...new Set(pts.map(p=>p.arm))];
function draw(){layer.clearLayers();let mode=document.getElementById('mode').value;
document.getElementById('legend').textContent=mode==='arm'?arms.map((a,i)=>a+' '+colors[i%4]).join(' | '):
mode==='push'?'green <10 N · amber 10–20 N · red ≥20 N':
mode==='speed'?'blue <18 · green 18–25 · amber 25–45 · red ≥45 mph':'green counting · grey excluded';
pts.forEach((p,i)=>{let q=pts[i+1];if(!q||q.route!==p.route||q.t-p.t>2)return;
let color=mode==='arm'?colors[arms.indexOf(p.arm)%4]:
mode==='push'?(p.push<10?'#81c784':p.push<20?'#ffb74d':'#ef5350'):
mode==='speed'?(p.speed<18?'#4dd0e1':p.speed<25?'#81c784':p.speed<45?'#ffb74d':'#ef5350'):(p.good?'#81c784':'#888');
L.polyline([[p.lat,p.lon],[q.lat,q.lon]],{color,weight:5})
.bindPopup(p.route+'<br>'+p.arm+' · '+p.speed.toFixed(0)+' mph · '+p.push.toFixed(1)+' N<br>'+p.reason+' · '+p.kind).addTo(layer);});}
if(pts.length)map.fitBounds(pts.map(p=>[p.lat,p.lon]));
document.getElementById('mode').onchange=draw;draw();</script>'''
  return html.replace('DATA', json.dumps(points, allow_nan=False))


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('routes', nargs='+', type=Path)
  parser.add_argument('-o', required=True, type=Path)
  parser.add_argument('--experiment', choices=EXPERIMENTS, default='nnlc_model')
  args = parser.parse_args()
  points = [p for route in args.routes for p in points_for(route, args.experiment)]
  args.o.write_text(render(points))
  print(json.dumps({'points': len(points), 'road_matched': road_matched(points)}, indent=2))


if __name__ == '__main__':
  main()
