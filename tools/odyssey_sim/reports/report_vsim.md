# Video simulator research report

The real driving model now runs on the Studio against the van’s recorded two-camera video. On a moving 55-second window, its steering request matches the log at **r = 0.99983**. Near-camera lane position differs by about **1 cm RMS**. The finished pipeline processes **31.7 frames/s including log loading**, faster than its 20 Hz input rate. No comma access was needed.

**This is not yet a simulator that can rank steering ideas.** The furthest sequential gate passed is gate 1. Lateral image shifts work within the required tolerance, but small heading shifts produce only 60–71% of the expected perceived heading near the camera. Separately, the existing plant drifts roughly 160–190 m over one minute when fed the actual recorded commands. Logged yaw through the same pose integrator has zero drift. A plant accurate enough for short acceleration comparisons is not accurate enough for minute-long position integration.

My recommendation is to keep this real-model replay as the perception test bed, investigate the small-yaw perception discrepancy without scaling the injected angle, and make 60-second logged-command drift the next plant acceptance test. Validate any plant correction on untouched held-out windows before free-running controllers. Candidate rankings and road settings are not justified yet.

## Delivered and deliberately incomplete

Code is in `~/Code/odyssey-pilot/tools/odyssey_sim/vsim/`, checkout `odyssey-sim`, starting HEAD `cba631852`. README gives exact commands. There is a one-command gate CLI, geometry and exact relative-pose integration, a portable default-model runner, native NV12 preprocessing, upstream FrameReader compatibility, wander diagnostics, a controlsd curvature audit, and an unchanged-plant drift audit. Eleven tests and Ruff pass.

The requested arbitrary `module:factory` free-running controller mode is **not implemented**: the CLI rejects it explicitly until the camera and plant gates are satisfied. Curve width, takeover, curve-release and body-motion comparison fields remain explicitly unavailable. These are material missing parts of the requested final simulator, not completed work hidden behind a warning. The brief’s permitted fallback—furthest passed gate, measured failures and next step—is the delivered outcome.

All authored code is confined to `vsim/`; evidence, temporary dependencies and helper scripts are confined to this scratch folder. No commit, push, branch switch, device connection or runtime parameter mutation was performed. Existing unrelated `msgq_repo` dirt was preserved. The repository has no `ARCHITECTURE.md` or local agent file; the local README supplies this prototype’s map without changing files outside the allowed scope. As the assigned research child, I performed the reviews myself and did not spawn further agents.

## Generation log

| Generation | Measurement / change | Result and interpretation |
|---|---|---|
| G0: identify actual stack | Read ODYSSEY, runtime dispatch, initData and carParamsSP | No active custom bundle; default path is upstream `selfdrive/modeld/modeld.py`. Contrary to the brief’s assumption, `modeld_v2` requires an explicit bundle and is not the default runner. |
| G1: decoder baseline | Tried the existing FrameReader unchanged | Failed before inference: installed ffmpeg rejects removed `-vsync` option. `g1.log` records the exact exception. Added a decoder adapter only inside vsim, retaining upstream indexing and caching. |
| G2: first video replay | First 60 s, 1,200 model frames | Paths r=0.99661, RMS 0.00986 m. Speed was 0 throughout. Curvature comparison fails and is not a valid driving test; it is dominated by stationary curvature-hold initialization. Added moving/warm eligibility instead of claiming success. |
| G3: moving replay | 1a3, 610–650 s, 800 scored frames | Desired curvature r=0.999770, RMS 1.8275e-5 m⁻¹. Lane/path r>0.9998. Encoder EOF timestamps match exactly. Gate 1 passes on this training window. |
| G3b: controls audit | Imported actual `clip_curvature`, 4,000 native 100 Hz samples | Generated versus logged controlsd r=0.996377; RMS 7.0017e-5 m⁻¹. Even logged model requests replayed through clipping have RMS 6.6043e-5 from causal log/scheduling/other-plan mismatch. Generated versus logged-model clipping r=0.999771. |
| G4: warm-start/longer replay | Start 15 s before score window, require 200 continuous frames, extend score to 610–665 s | 1,100 moving frames, desired curvature r=0.999832, RMS 1.8006e-5 m⁻¹. Gaps count as lost continuity; replay across a gap cannot satisfy warm eligibility immediately. |
| G4b: injection sweep | Fixed dy ±0.1/±0.5 m; heading ±0.5/±2°, same paired 800-frame window | Translation slopes 0.907–0.952 pass. Near-camera heading slopes 0.600–0.890 fail. Corrections overwhelmingly point the right way. No arbitrary warp amplification was applied. |
| G4c: challenge heading estimator | Vary polynomial fit extent 10–60 m, evaluate heading at 0 and 10 m | Small-yaw slopes at 10 m remain about 0.74–0.80; the failure persists. Larger ±2° injections reach roughly 0.92–0.98 at 10 m. Did not select a fit range to manufacture a pass. |
| Independent plant diagnostic | Three uninterrupted autonomous ~60 s training windows, unchanged calibrated plant, logged CAN commands | Position drift 157–186 m. Logged-yaw oracle stays at exactly zero. Gate 3 would fail even if perception passed. No plant tuning was performed. |
| G5: full metrics/timing | Same 55 s replay, count warm frames in throughput | 1,374 processed frames; model 147 fps; decode dominates. CLI reports actual existing wander metric, with unavailable closure metrics explicit. |
| G6: decode optimization | Benchmark 2/4/8 software threads and VideoToolbox; select 4-thread hardware decode | Decoding 1.32× faster over the full run. All generated curvature, lane and path values are exactly identical to software decode. End-to-end 43.38 s for 1,374 frames. |
| Final review/tests | Physical-projection, sign, clamping, packing, recurrence and integration tests; Ruff | 11 tests pass, Ruff clean. An initial recurrence test checked too early: the graph has history queues, so successive identical first frames can give identical predictions. A 20-frame warm sequence correctly detects recurrence. |

## Gate 1: open-loop fidelity — passes on measured training windows

The driving ONNX is the repository’s actual default `driving_supercombo.onnx`, not a substitute policy. SHA-256:

`65a08adc31d5c456219687d99b7bf5e44d61dae2d49ea67850e76105c7248cce`

Embedded checkpoint:

`6a7d09ad-bcc9-43bc-916d-29287e60cee2/200/a27b3122-733e-4a65-938b-acfebebbe5e8/100`

The graph has `new_img`, desire, traffic convention, `action_t`, image history, desire history and feature history inputs. It returns all next recurrent states; these feed the next invocation directly. This default graph has no separate `prev_desired_curv` or `lateral_control_params` input. Those are inputs of other model generations; adding invented inputs here would be incorrect. Delay enters the two action horizons. I import the actual model action extraction and parsing functions.

`initData` has CameraOffset=0, LagdToggle=1, ModelRunnerTypeCache=2, NeuralNetworkLateralControl=1, no selected bundle, and no NNLC override/AB entry. carParamsSP explicitly names `HONDA_ODYSSEY`, path `/data/openpilot/openpilot/sunnypilot/neural_network_data/neural_network_lateral_control/HONDA_ODYSSEY.json`. This establishes stock NNLC for the tested training route. Held-out initial params also show NNLC enabled; a full per-drive model/AB history audit remains required before gate 4.

The replay pairs narrow and wide by modelV2 frameId/frameIdExtra through their encode indices and segmentId. EOF timestamp error is zero on every scored frame. Both camera transforms use causal extrinsicsCalibration and sensor intrinsics. Preprocessing samples NV12 directly, with nearest-neighbour clamping and native Y00/Y10/Y01/Y11/U/V packing. There is no RGB conversion or double warp. Desire pulses are reconstructed from logged lane-change state/direction; exact DesireHelper replay through lane-change transitions remains unvalidated. CameraOffset≠0 and custom bundles fail explicitly.

| Scored 1a3 window, seconds from first modelV2 | n | Desired curvature r | Curvature RMS error, m⁻¹ | Lane y r, all horizons | Path y r, all horizons |
|---|---:|---:|---:|---:|---:|
| 610–650 | 800 | 0.999770 | 0.00001827 | 0.999879 | 0.999892 |
| 610–665 | 1,100 | 0.999832 | 0.00001801 | 0.999882 | 0.999903 |

The second window overlaps the first; it is a duration/warmup check, not independent statistical replication. Speed is 24.69–25.29 m/s. Curvature error corresponds to desired-acceleration RMS error of 0.01124 m/s², against logged wander-band desired-acceleration RMS 0.12784 m/s². All-horizon lane/path RMS errors are 0.100/0.145 m, including distant predictions; near-camera lane-centre RMS error is 0.00958 m. Do not confuse the distant-prediction aggregate with near-camera tracking error.

The native controlsd audit exceeds the >0.99 correlation target too, but retains a scheduling limitation: log delivery order approximates SubMaster timing, and the audit does not replay the complete lateralManeuverPlan validity lifecycle. Its logged-model baseline quantifies that gap rather than hiding it. Full asynchronous process replay, transition desire inputs and other speeds/routes remain unverified.

Evidence: `vsim/g3.json`, `g4.json`, `g6.json`, their trace files, `controls_audit.json`, `fidelity_detail.json`, `controller_identity.json`.

## Gate 2: reprojection — lateral subtest passes; heading/natural gate incomplete

Coordinate convention: calibrated device x forward, y right, z down; positive dy and heading move/turn the virtual vehicle right. The view-coordinate ground normal is +y and its distance is the logged calibration height. The homography is conjugated by camera calibration and combined with model preprocessing before a single sampling operation. Pure rotation applies to all depths; translation assumes ground points.

Eight geometry/integration tests independently check literal physical examples, arbitrary tilted-camera 3D ground projection, positive-shift sign, off-plane rotation, invalid height, identity, equal-yaw curved-path closure and a known constant-yaw arc. Thus geometry has stronger evidence than merely observing a model respond in the expected direction.

Perception is the centre of the two inner lane lines. A cubic fit over fixed model x coordinates through 30 m gives offset and heading at x=0. Signs are vehicle-relative: minus lane-centre y and minus lane heading. Same-frame unshifted replay is the comparator; all 800 frame pairs are retained.

| Injected pose | Perceived offset change, m | Perceived heading change, ° | Relevant slope | Plan corrects toward original pose |
|---|---:|---:|---:|---:|
| dy −0.1 m | −0.0952 | −0.0153 | 0.952 | 97.6% |
| dy +0.1 m | +0.0907 | +0.0106 | 0.907 | 95.9% |
| dy −0.5 m | −0.4572 | −0.0851 | 0.914 | 100% |
| dy +0.5 m | +0.4612 | +0.1066 | 0.922 | 98.6% |
| yaw −0.5° | −0.0021 | −0.2999 | 0.600 | 100% |
| yaw +0.5° | +0.0211 | +0.3542 | 0.708 | 99.9% |
| yaw −2° | +0.0297 | −1.6434 | 0.822 | 100% |
| yaw +2° | +0.0122 | +1.7800 | 0.890 | 95.8% |

For ±0.1 m shifts, mean desired lateral acceleration changes by about ∓0.033 m/s²; for ±0.5 m, about ∓0.19–0.21 m/s². Heading corrections are nonlinear and asymmetric, especially +2°. These are measured model requests, not proof of closed-loop plausibility.

The small-yaw failure is robust to fit extent. At x=10 m, ±0.5° slopes remain roughly 0.74–0.80. Pure rotation introduces no road-plane approximation; possible causes include learned/temporal pose priors and perception extrapolation, rather than a simple sign or ground-height error. The present evidence does not distinguish those causes conclusively. Inflating injected yaw would change the virtual camera pose and is not a valid fix.

**Questioning the gate without weakening it:** requiring perfect predicted heading equivariance also tests the network’s perception accuracy, not only rendered geometry. A real network can systematically under-report heading in a correct camera view. Keep this gate failed for now; use an independently measured natural pose experiment to decide whether the gate or renderer is wrong. Do not simply waive it because the controller corrects in the expected direction.

Natural consistency check: replay versus logged model lane offset has r=0.99959 and 1.14 cm RMS on 699 frames whose logged offset exceeds 10 cm; on 372 frames above 20 cm it has r=0.99956 and 1.44 cm RMS. **Logged model estimates are not independent pose truth.** This is useful consistency evidence, not the requested causal natural-experiment validation.

Evidence: signed `dy_*.json`/`yaw_*.json` and traces; `injection_summary.json`; `heading_metric_sensitivity.json`; `natural_check.json`.

## Gate 3: unchanged plant fails an independent diagnostic

This was measured as an isolated downstream diagnostic, not counted as passing out of sequence. Read-only extraction caches supply the logged applied Honda CAN command, speed, roll, steering-equivalent acceleration and validated body acceleration (`vEgo × carControl.angularVelocity[2]`). Initialize the existing calibrated plant from each window’s logged state and command, then hold each 10 Hz cached input over ten integration substeps, using actual timestamp intervals. No recentering or fit change occurs.

Relative motion integrates both forward and lateral displacement, expresses them in the rotating logged frame, and integrates relative heading from simulated versus logged yaw. The independent oracle feeds logged yaw to both sides. That remains exactly at zero even on curves.

| Route | Absolute log t0, s | Duration, s | Final dy, m | dy RMS, m | Final heading error, ° | Body accel RMS error, m/s² |
|---|---:|---:|---:|---:|---:|---:|
| 1a3 | 915.6075 | 59.9002 | 163.48 | 69.22 | 14.16 | 0.1560 |
| 1a3 | 975.6076 | 59.8999 | 157.48 | 69.92 | 12.95 | 0.1083 |
| 1a3 | 1035.6078 | 59.9022 | 185.66 | 77.62 | 17.13 | 0.1397 |

These are all three eligible nonoverlapping windows on the audit’s fixed 60-second grid, not hand-picked failures. The yaw oracle is zero in all three. This diagnostic uses 10 Hz applied-command caches, not a newly extracted native 100 Hz command stream. Repeating with native timing is the exact next discrimination test before attributing the full error to model structure. Even an acceleration error of 0.1 m/s² can integrate to order 180 m over a minute; a short-window acceleration-fit score cannot certify lane-position stability.

Do not feed those offsets into the image model: they are far outside the small-shift renderer’s useful range. Do not subtract each window’s future mean bias or continuously anchor the simulated pose to make this gate pass. A logged-baseline residual plant could be a useful counterfactual research model, but its zero-injection pass would be true by construction and would need separate perturbation validation.

Evidence: `plant_audit.json`; implementation `vsim/plant_audit.py`; inherited plant parameters remain unchanged.

## Gate 4: not attempted; no rankings

Gate 2 has not passed and the independent plant diagnostic fails. In addition, at final inventory neither 1a7 nor held-out 1aa/1ab has any complete local two-camera segment. 19f/1a0/1a2 folders are absent. Only 1a3 contains paired video so far: segments 0, 1 and 10–18. Lexicographic arrival explains the gaps; the route is not presumed complete. No download was started or manipulated by this worker.

No held-out route was used to tune anything. No stock/V1/S3/S5, friction-assist, preview or low-speed controller ranking is offered. The complete simulator remains unavailable even if the downloads finish, because its geometry-to-model and plant acceptance issues still need resolution.

Evidence: initial `inventory.json`, final `inventory_final.json`.

## Gate 5: measured faster than real time

Final G6: 1,374 processed model frames (warmup included), 1,100 scored. Total wall time 43.38 s: **31.67 frames/s**, or 1.58× the model’s real-time rate. Model execution alone is 148.27 fps. Decode+warp+model is 40.98 fps; the difference includes log/context loading, indexing and Python/report overhead.

| Component | Software decode G5, s | Hardware decode G6, s |
|---|---:|---:|
| Decode | 26.93 | 20.37 |
| Warp | 3.89 | 3.89 |
| Model | 9.35 | 9.27 |
| Total wall | 49.93 | 43.38 |

On a representative 30-frame GOP, software 2/4/8 threads took 0.287/0.292/0.298 s; VideoToolbox 2/4/8 took 0.242/0.218/0.224 s. All six raw NV12 hashes were identical. The full optimized 55-second replay then produced **exactly identical curvature, lane and path arrays** to the software baseline. No fidelity tradeoff was accepted for speed.

The existing wander calculation reports 12.38 cm replayed versus 12.30 cm logged, both with 13.75 s dominant period. These describe this unmasked open-loop perception/request window; they do **not** constitute gate-4 lane-motion reproduction. This replay has not profiled a complete free-running controller/plant loop.

Evidence: `g5.json`, `g6.json`, `decode_benchmark.json`, `final_speed.json`.

## Public sources and local source map

- [comma: End-to-end lateral planning](https://blog.comma.ai/end-to-end-lateral-planning/) describes real-video planar warping, its small-deviation utility, and the risk that models exploit warp artifacts. It supports this experiment’s basic renderer, not an assumption that every network’s pose estimates are perfectly equivariant.
- [comma: Learning to Drive from a World Model](https://blog.comma.ai/mlsim/) distinguishes depth-based reprojection and a learned world model. Its learned simulator accepts six-degree-of-freedom pose conditioning. The current prototype is a plane renderer, not a recreation of that learned simulator.
- [comma: openpilot 0.11](https://blog.comma.ai/011release/) shows explicit expected-versus-observed pose-deviation evaluation. That is a useful pattern for the missing independent natural-pose validation.
- [comma: openpilot 0.9.5](https://blog.comma.ai/095release/) introduces MetaDrive as the default synthetic simulator. Local `tools/sim` is not the recorded-van camera/plant loop requested here; no evidence was found that it supplies the missing Odyssey calibration.
- Local runtime truth: `ODYSSEY.md`; `openpilot/sunnypilot/models/helpers.py` (`get_active_bundle`/runner selection); `openpilot/selfdrive/modeld/modeld.py`; `fill_model_msg.py`; `parse_model_outputs.py`; `openpilot/selfdrive/controls/controlsd.py`; `drive_helpers.py`.
- Local preprocessing truth: `tinygrad_repo/examples/openpilot/compile_warp.py`, `openpilot/common/transformations/model.py`, `camera.py`, `openpilot/tools/lib/framereader.py`.
- Reused plant and metrics: `tools/odyssey_sim/calib/plant.py`, `calib/identify.py`, `tools/odyssey_wander.py`. The actual-controller replay remains the established implementation in `replay/`; this prototype does not replace it with copied controller code.

## Exact continuation

1. Replay native 100 Hz applied commands into the unchanged plant on the three listed windows. Compare acceleration bias, yaw residual and integrated dy to isolate cache resampling from plant mismatch. Fit only on 1a3/1a7, freeze, then test other windows.
2. For small-yaw perception, compare projected lane coordinates and model lane predictions against an independently estimated natural camera/vehicle heading change. Keep the pose injection geometrically exact. Determine whether the residual is network perception bias or a renderer/context defect.
3. Once those pass and held-out video is local, wire `module:factory` through the existing real controller adapter with isolated scratch Params, native 100 Hz clipping/control, dynamic dy/dpsi reprojection and per-window closure metrics. Validate the **logged controller identity and runtime modes** on each held-out drive first.
4. Only after held-out gate-4 agreement, run paired-window controller rankings and uncertainty. Further ideas then worth testing include offset/heading bias estimation, speed-dependent compensation for measured EPS stiction, and curve-entry/exit asymmetry. They remain hypotheses, not recommendations for the vehicle.
