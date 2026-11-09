# Brief: a closed-loop lane simulator that runs the real driving model on reprojected video

Read `BRIEF_COMMON.md` in this scratch directory first. Its goals, rules, data paths and conventions all apply. This brief adds your task.

## Why this, and why now

Every steering idea so far (van-trained NNLC models, friction assistance, curve look-ahead, low-speed angle steering) can be ranked on how the steering responds, but not on whether the van stays centred in its lane. The lane loop is the problem. Three calibration passes (`report_calib.md`, `report_calib3.md`, `report_wander.md`) tried to model openpilot's planner as a function of lane offset and heading, and all failed to close: 0 of 40 behaviour checks pass, and the best fitted planner reaches R² of only 0.44–0.64.

The planner is a neural network looking at the camera, so a fitted formula can't stand in for it. William's view, which I share, is that a simulator good enough to rank lane-keeping ideas opens the real frontier: once it works, we can test ideas freely instead of one road drive at a time.

## The idea

Close the loop with the real model on real video. The van's recorded road and wide camera video now exists on the Studio for several drives. To simulate the van sitting `dy` metres to the side of, and `dpsi` radians off, the heading it actually drove, warp each recorded frame to the view from that pose. Then run openpilot's own driving model on the warped frames, and its own controlsd lateral math and controller on the model output. Our calibrated plant turns the controller's torque into motion, and that motion updates `dy` and `dpsi` for the next frame.

- **Rotation (`dpsi`) is exact:** it is a pure-rotation homography.
- **Lateral translation (`dy`) is exact for points on the road surface:** use the plane-induced homography. Vertical objects will be slightly wrong, which is acceptable at wander-scale offsets (0–0.5 m).
- **comma trains its models with the same kind of reprojection.** Search for and cite public descriptions, for example comma's blog posts and talks on the "learned simulator" and on pose augmentation.

openpilot's MetaDrive simulator (`tools/sim`) uses synthetic graphics and isn't calibrated to the van, so it isn't the answer here. Say so if you find otherwise.

## Pieces to reuse

- **Real controller and plant.** `tools/odyssey_sim/replay/` is the validated harness (r≈0.999): the real `LatControlTorqueV0` and NNLC at 100 Hz. `tools/odyssey_sim/calib/` holds plant v2, the evaluator and its controller interface (`--controller module:factory`), and the yaw-validated body motion.
  - **Body lateral accel:** vEgo × carControl.angularVelocity[2].
- **Metrics.** Use the wander metrics (`tools/odyssey_wander.py`, `tools/odyssey_sim/wander/`), the curve metrics (`tools/odyssey_sim/curves/characterize.py`) and the perception metric (`tools/odyssey_sim/perception/metric.py`).
- **Driving model.** The van runs sunnypilot's model stack under `openpilot/sunnypilot/modeld_v2/`, and `openpilot/selfdrive/modeld/` is the upstream original.
  - **Identifying the model.** The van has no custom model selected, so it should be the repo's default bundle. Confirm from the rlogs (initData params, and model metadata in modelV2 / modelDataV2SP).
  - **Recompiling.** The `*_tinygrad.pkl` files are compiled for the comma's Qualcomm GPU. Compile or run from the ONNX for the Mac with tinygrad (Metal or CPU); see `compile_modeld*.py`, `get_model_metadata.py` and `selfdrive/test/process_replay/model_replay.py`.
- **Inputs to reproduce.** The model is recurrent and takes both cameras. Reproduce its inputs exactly:
  - the frame pairing, via roadEncodeIdx and wideRoadEncodeIdx;
  - the warp from liveCalibration;
  - desire, the traffic convention, the lateral-control params and the previous desired curvature;
  - the feature buffer.
- **Video.** Decode with `tools/lib/framereader` (ffmpeg is at /opt/homebrew/bin/ffmpeg).
- **controlsd lateral math.** This covers desired curvature from the model plan with the learned lateralDelay, clipping, and the torque controller. Mirror what `selfdrive/controls/controlsd.py` does on this branch, and import rather than copy where you can.

## Data

Road camera (`fcamera.hevc`), wide camera (`ecamera.hevc`) and `rlog.zst` per segment, in `~/comma-logs/04b44e4c4fe44232_<route>/<route>--<seg>/`.

The download from the comma is running now, in this order: 000001a3, 000001a7, 000001aa, 000001ab, then 0000019f, 000001a0, 000001a2 (those last three are new to you and have rlogs too).

- **1a3 arrives first:** start with it.
- **A route is complete** when the next route's folder starts filling, or when its file sizes stop changing.
- **Video exists for no other routes.** The older ones were already deleted on the device.

The split carries over from BRIEF_COMMON:

| Role | Routes |
|---|---|
| Train / tune | 1a3, 1a7 |
| Held out | 1aa, 1ab |
| Extra unseen checks | 19f, 1a0, 1a2 (check what driving each contains first) |

Never tune on the held-out routes.

## Gates, in order

Each gate is a measurement, not a hope. If one fails, find out why, fix it, and log the generation. Question the gate itself if you can show it is wrong, but don't weaken it to pass.

1. **Open-loop fidelity.** The model on unwarped recorded video, at logged timing, reproduces the logged modelV2 outputs: lane lines, path, and the desired curvature controlsd derived. Target correlation >0.99 on desired curvature. If encoding or frame timing makes that impossible, quantify the gap and show it is small next to the wander.
2. **Reprojection fidelity.**
   - **Injected shifts:** shift by known `dy` (±0.1–0.5 m) and `dpsi` (±0.5–2°). The model's perceived lane position and heading should shift by the injected amount (slope 1 ± 0.1), and the plan should correct in the right direction with a plausible magnitude.
   - **Natural experiment:** compare against moments in real logs where the van was genuinely off-centre.
3. **Zero-injection stability.** With the logged controller output replayed as-is, the simulated pose stays near the logged path for 60 s windows.
   - **If it drifts:** that is plant or integration error. Measure it before blaming the model.
4. **Closure: the gate everything else waits on.** Free-running with the controller that actually ran on each held-out drive (read it from the logs: stock torque or NNLC, and which model file), the simulator must reproduce that drive's real behaviour:
   - the 7–20 s lane-offset RMS and spectrum;
   - the wander period;
   - how wide the van runs mid-curve;
   - the curve-exit release timing.

   Agree within about 25%, on held-out 1aa/1ab, without tuning on them. Report per window, not just medians.
5. **Speed.** Report frames per second on the Studio. Aim for faster than real time. If it is slower, find where the time goes (decode, warp or model) and fix the biggest.

## Payoff, once gate 4 passes

Rank on the held-out drives, with paired windows and uncertainty:

- stock NNLC
- van models V1/S3/S5 (in `nnlc/odyssey_models/`, and all candidates in `~/comma-logs/odyssey-nnlc-models/`)
- friction assistance (`odyssey_wander.FrictionAssist`)
- curve look-ahead (`tools/odyssey_sim/curves/transition.py`, preview mode)
- the low-speed angle mode, where coverage allows

Lead with tracking and takeover-proxy metrics, then body motion. Then propose the next generation of ideas the simulator makes testable.

## Deliverables

- Code in `tools/odyssey_sim/vsim/`:
  - a README;
  - a one-command CLI, `python -m tools.odyssey_sim.vsim.run --route <dir> --controller module:factory [--window …]`, that prints the same metrics as the wander and curve tools;
  - unit tests for the reprojection geometry and the pose integration.
- Report `scratchpad/report_vsim.md`:
  - a plain-language summary for William on top;
  - the generation log;
  - per-gate evidence, with sources cited.

If you can't reach gate 4, deliver the furthest gate you passed, the measured reason the next one fails, and the most promising next step.
