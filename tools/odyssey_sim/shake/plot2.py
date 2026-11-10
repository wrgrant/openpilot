"""Static report figure; plots rejected-model diagnostics, not road forecasts."""
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from tools.odyssey_sim.shake.analyze import OUT


def main():
  s = json.loads((OUT/'summary2.json').read_text())
  fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
  keys = ['crossings_s', 'after_crossing', 'command_rms', 'wheel_rms_deg', 'body_rms']
  labels = ['Crossings/s', 'Size +0.1 s', 'Command RMS', 'Wheel RMS', 'Body RMS']
  for r in s['closed_loop']:
    if r['scope'] == 'development':
      ax[0].plot(range(5), [r['ratios'][k] for k in keys], 'o-', label=['stock', 'S3', 'S5'][r['model']])
  ax[0].axhspan(.75, 1.25, color='green', alpha=.2, label='Required range')
  ax[0].set_yscale('log')
  ax[0].set_xticks(range(5), labels, rotation=20)
  ax[0].set_ylabel('Simulation / road, median paired windows')
  ax[0].set_title('Closed-loop gate: all three fail')
  ax[0].legend(fontsize=8)
  r = json.loads((OUT/'factorial2.json').read_text())['rows']
  for model in (0, 2):
    x, y = [], []
    for gain in (1, 2):
      z = [a['stats']['body_rms'] for a in r if a['model'] == model and a['gain'] == gain and a['noise'] and a['disturbance'] and a['lane']]
      x.append(gain)
      y.append(np.median(z))
    ax[1].plot(x, y, 'o-', label='stock' if model == 0 else 'S5')
  ax[1].set_xticks([1, 2])
  ax[1].set_xlabel('Shaper error gain; all excitation factors on')
  ax[1].set_ylabel('Simulated body RMS (m/s²)')
  ax[1].secondary_yaxis('right', functions=(lambda x: x*80, lambda x: x/80)).set_ylabel('Equivalent RMS push on 80 kg (N)')
  ax[1].set_title('Factorial: rejected plant over-amplifies S5')
  ax[1].legend()
  for a in ax:
    a.grid(alpha=.2)
  fig.tight_layout()
  fig.savefig(OUT/'gates2.png', dpi=160)


if __name__ == '__main__':
  main()
