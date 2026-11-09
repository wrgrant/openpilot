# Odyssey EPS preparation (offline only)

`rwd.Image.decode(raw)` / `.encode()` validate all three checksums, the THR-A020 identity, keys, layout, torque axes and six table copies. `rwd.build(raw, outputs=..., limits=(...), identity=...)` or `scale=...` writes only known table/identity locations and recomputes checksums. Eight outputs exclude the leading zero knot. Cipher permutation is from nrdr-derived `check_rwd.py`; no downloaded module is imported.

Run from the repository with its Python:

```
.venv/bin/python tools/test_runner.py tools/odyssey_eps opendbc_repo/opendbc/sunnypilot/car/honda/test_honda.py
.venv/bin/python -m tools.odyssey_eps.fit --stock STOCK_NPZ_DIR --modified MODIFIED_NPZ_DIR --scale 2.5 --out NEW_OUTPUT_DIR
```

For firmware inputs use isolated Python (`-I`) and explicitly insert this trusted repository into sys.path before importing these modules. `prepare.main()` regenerates candidates only under `~/comma-logs/odyssey-eps/`. The binaries stay outside git. Reference tests skip when external files are absent; synthetic codec tests remain.

## Controller configuration

Modified Odyssey only (EPS identity `39990-THR,A020`). The `OdysseyEpsMap` param (JSON, registered in `params_keys.h`) is read once when the car controller is built; change it offroad. Example:

```json
{"firmware_scale":1.5,"authority_ceiling":1.0,"speed_bp":[0,40],"speed_gain":[1,1],"torque_bp":[0,1],"torque_v":[0,1]}
```

The firmware's torque table output acts as a virtual torque on the wheel, which then goes through the normal assist curve: across the command range and above 18 mph, the hand torque that produces the same motor torque is a constant 2400-2700 sensor units times the table output. So a modified image is matched to stock by inverting its table, and `opendbc.sunnypilot.car.honda.eps_mod.OdysseyEpsMap` does that:

1. The controller's torque goes through `torque_bp`/`torque_v` (or `torque_rows`, one row per speed breakpoint) to a stock-equivalent command. Values above 1 ask for more than stock can give, e.g. bp `[0,.8,1]`, v `[0,.8,1.5]` keeps the normal range as stock and adds authority only near the top.
2. That goes through the stock table to the virtual torque stock would produce, relative to stock's maximum, and is multiplied by `speed_gain` (to undo the low-speed fade).
3. It is capped at `authority_ceiling` (default 1 = stock's maximum, at most 2) and at what the flashed image holds.
4. The command sent is the one whose modified-table output equals that virtual torque.

With the defaults, a modified EPS receives the table output a stock one would, so it should drive like stock; that still has to be confirmed on the van. All images share one identity, so `firmware_scale` (1.5, 2 or 2.5) must match the flashed image. A missing or invalid config assumes 2.5, which sends the smallest commands.

The normalized rate limit stays 3/s. A second bound of `4096 * 0.03 / scale` counts per frame applies after the map, so speed gain and lookup slope cannot raise the command rate. An unmodified Odyssey's commands are unchanged, and no panda safety limit is relaxed.

## Offline characterization

Use actual CAN command caches (`cmd`, `motor`, `v`, `active`, `pressed`, `drv`, 100 Hz). `fit` keeps 1-second steady blocks, hands-off and |driver|<100, reports counts and 10th/90th percentile motor per speed/command bin, then inverts the measured response. It rejects insufficient coverage, plateaus, large monotonicity violations and out-of-range targets; it emits a candidate JSON only with at least two populated speed bins. No post-flash data exists yet. A short drive can fail to provide enough coverage; do not extrapolate missing data or force saturation to fill bins. The existing suite's small highway pulses measure onset and delay, not full-range authority or low-speed fade.

The fit pools motor magnitude across steering directions; review left/right held pulses separately with `tools/odyssey_eps_report.py` before accepting a symmetric map. Curve fits are observational, not proof of steering dynamics or override thresholds. Treat bin-to-bin interpolation as provisional and keep operation in covered speeds.

## Map of this addition

`rwd.py`: codec and exact rebuild seam. `prepare.py`: external candidate writer / calibration inventory. `fit.py`: cache characterization / conservative inverse-map seam. Tests exercise these seams and the Honda controller's actual transmitted torque bytes. Runtime mapping lives in the existing `opendbc.sunnypilot.car.honda.eps_mod` extension and is called after Honda's original rate limit.
