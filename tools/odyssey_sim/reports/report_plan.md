# Plan-side wander research

William: **the small lane-centring correction is the promising plan-side road-test candidate.** It reduced simulated held-out wander by **1.74cm RMS** across seven observable windows; a shared lane reference plus simulated pose gives **1.87cm**. Mean body jerk increased **0.014m/s³** across 13 held-out windows. The simulator has failed held-out closure checks, so this is a testable indication, not a validated road improvement or centimetre-level lane holding.

**Recommendation:** keep the default driving model and prioritize a reviewed stock-versus-lane-correction road A/B. Leave the standalone prototype disabled until that integration/test is authorized. The tested lead filter worsened held-out wander by **1.34cm RMS**; residual-proxy smoothing showed no reliable benefit.

The inherited phase interpretation needs correction: **−120° includes the restoring sign**, and is compatible with stable offset-plus-heading damping rather than 120° of delay. A refined training replay reduced perceived offset wander from **13.6cm to 0.84cm**, while the request's slow-band variation persisted. That supports investigating upstream forcing, but does not separate true road curvature from intrinsic model variation.

Cheddar and Terrible Super Fantastic Do Over now execute on the Mac and are compared on the same 19-window population. Their input contracts, source hashes, request residual proxies, transfer responses, physical-path indications and curve timings are retained below. Neither alternative showed a held-out mean improvement using the shared lane reference. Model-specific lane estimates can create apparent improvements; the shared-reference comparison is essential. No simulator ranking is licensed.

## Scope and evidence

Checkout `/Users/williamgrant/Code/odyssey-pilot`, branch `odyssey-sim`, base `e6e9b55bd`. Authored code is confined to `tools/odyssey_sim/plan/`; evidence is in this report's adjacent `plan/` directory. No device access, production edits, commits, pushes or forecast changes. A concurrent scorer owns changes to `tools/odyssey_wander.py`; this worker did not edit it.

The default-model population is the prior exhaustive native-eligibility/video inventory: 3 windows on 1a3, 3 on 1a7, 6 on 1aa, 7 on 1ab. Settings were fixed before held-out scoring. Every eligible window is retained, including low-confidence lane windows whose canonical wander is unavailable. Five of six training windows contain external steering probes; held-out windows contain none. New open-loop replay measures fidelity on these videos rather than assuming the original 1a3 check generalizes.

Alternate-model discovery started with 1a3_01, 1a7_02, 1aa_01, 1ab_00 and 1ab_03, including matched curve coverage. Final evaluation expands both models unchanged to the same full 19-window population; the early subset does not determine inclusion in final scores. The four-period injected-response experiment uses the existing 1a3 610–665s video and the same model-specific unwarped baseline. It is training evidence only.

All closed-loop values remain indications. The inherited simulator passed only 3/7 held-out spectrum comparisons and misses some curve-width checks. It carries logged motion forward through a differential plant. Camera-derived road curvature is not independent surveyed road truth.

## Generation log

| Generation | Experiment / result |
|---|---|
| G0 | Read all requested reports, real-model runner, controller adapter, geometry, model selection and van sign conventions. No architecture map exists; the scoped README provides the research-code map without modifying the repo root. |
| G1 | Initial adapters ran, then self-review caught compensation entering the previous-action smoother and a path warp lacking dynamic warmup. Stopped owned jobs; preserved outputs/logs under `plan/g1/`; no G1 result is a final comparison. |
| G2 | Applied compensation only to the loop target after unchanged action extraction. Warped the full logged path through warmup and scoring. Ran all six training windows; initial training mean changes were lead +0.150cm, lane −0.173cm and smooth −0.291cm under the later-rejected outer-lane gate. Froze modest gains before held-out evaluation. |
| G3 | Tested the restoring-sign interpretation of injected phase. Stable PD reproduces the four complex responses with ~10% error. Added a harmonic path-refinement attempt on training 1a3_01; retained its actual lane-offset reduction rather than asserting perfect-line replay. |
| G4 | Audited all 99 current bundles and historical catalogs; bundle files alone contain no ONNX. Source-hash lookup found 30 retrievable ONNX sources (11 QCOM, 19 Chestnut). Downloaded and hash-checked all 11 retrievable QCOM sources and recorded their four ABI families. WD40 CPU probes produced nonfinite raw outputs; ten other zero-state probes were finite. Adapted native/legacy recurrent video execution for the two selected models. |
| G5 | Measured both alternate models on fixed matched videos, plus four injected periods, then expanded both unchanged to all 19 windows. Added a shared logged-lane-plus-relative-pose metric after discovering perception differences could masquerade as improved path. Added training/held-out curve windows explicitly. |
| G6 | Scored all held-out default-model compensation windows, preserved unavailable visibility and pose-domain failures, compared jerk and matched curve timing. No held-out gain tuning. |
| G7 | Found that compensation confidence used all four lane existence probabilities rather than the inner two. A failing test reproduced the missing contract; preserved the over-gated runs under `plan/g2_outer_lane_gate/` and reran every training/held-out compensation window with the correct inner-lane gate, without changing gains. Correct-gate training changes are −0.040/−0.406/+0.354cm; held-out changes are +1.337/−1.744/+0.250cm for lead/lane/smooth. |
| Final self-review | Reviewed integration placement/sign/history, identification/visibility/cross terms, then dropout/runtime/scope. Eleven public-interface/recurrent-model tests and scoped Ruff checks are the verification gates. Missing-camera inputs initially raised TypeError; a failing public-interface test reproduced it and the final fallback handles missing values. No child agents were started, following this research-worker brief. |

## 1. What the request decomposition can establish

`analysis.json` applies the **measured injected** gain and phase, interpolated over frequency, to the observed offset. It does not use the attenuated natural regression. Road is the cubic 25m model-lane curvature times speed squared. Residual is request minus road proxy minus injected-response proxy.

Numbers below are **displacement equivalents**: ideal double integration of each acceleration spectrum in the 7–20s band, in cm RMS. They are not independently identified contributions to observed lane position. Full covariance matrices are in `analysis.json`; the component RMS values must not be added. The summed request includes large cancellation between components. Heading is represented only through the injected derivative-consistent weave transfer, which need not transport to natural heading/offset when geometric closure is poor.

“Perfect execution” holds the recorded plan and road fixed, adding the acceleration tracking-gap displacement to the recorded lane spectrum. It omits replanning and assumes the camera lane estimate is correct. It is a sensitivity calculation, not how many centimetres a real perfect controller would remove. The causal removable-wander field remains null.

| Window | Lane confidence | Road equiv cm | Pose-response equiv cm | Residual equiv cm | Observed cubic lane cm | Fixed-plan ideal-execution lane cm |
|---|---:|---:|---:|---:|---:|---:|
| 000001a3_00 | 0.87 | 29.505 | 54.031 | 41.630 | 14.034 | 15.363 |
| 000001a3_01 | 0.90 | 59.499 | 62.585 | 42.803 | 13.471 | 13.997 |
| 000001a3_02 | 0.87 | 21.900 | 40.779 | 28.849 | 11.228 | 15.354 |
| 000001a7_00 | 0.99 | 13.695 | 20.370 | 29.980 | 4.858 | 14.128 |
| 000001a7_01 | 1.00 | 58.072 | 40.707 | 28.349 | 10.548 | 18.526 |
| 000001a7_02 | 0.97 | 41.037 | 28.150 | 52.149 | 8.362 | 11.184 |
| 000001aa_00 | 0.02 | 61.992 | 20.002 | 28.035 | 5.678 | 15.745 |
| 000001aa_01 | 0.94 | 65.017 | 78.022 | 61.393 | 21.301 | 15.393 |
| 000001aa_02 | 0.98 | 63.104 | 29.100 | 35.832 | 8.660 | 7.943 |
| 000001aa_03 | 0.70 | 18.685 | 43.849 | 37.030 | 15.046 | 16.522 |
| 000001aa_04 | 0.93 | 19.485 | 27.045 | 25.666 | 10.103 | 11.301 |
| 000001aa_05 | 0.79 | 35.155 | 28.354 | 31.382 | 9.467 | 9.828 |
| 000001ab_00 | 0.87 | 33.458 | 16.283 | 26.051 | 7.905 | 17.753 |
| 000001ab_01 | 0.84 | 83.666 | 23.493 | 35.133 | 7.426 | 28.256 |
| 000001ab_02 | 0.82 | 48.142 | 33.510 | 23.733 | 9.447 | 17.410 |
| 000001ab_03 | 0.90 | 105.623 | 35.773 | 37.872 | 10.126 | 17.714 |
| 000001ab_04 | 0.25 | 20.323 | 12.592 | 16.562 | 5.084 | 10.994 |
| 000001ab_05 | 0.17 | 60.217 | 23.499 | 17.883 | 5.803 | 7.766 |
| 000001ab_06 | 0.00 | 11.429 | 18.363 | 11.369 | 5.449 | 11.607 |

Low-confidence rows are unmasked diagnostics; they cannot support lane-wander attribution. Cubic-origin and near-camera lane metrics can differ materially. The historical independent lane/road closure failure is a reason to retain these as proxies.

### Direct smoothed-path replay

The virtual camera removes the logged near-lane 7–20s offset component, with yaw equal to shift derivative divided by speed, throughout recurrent warmup. Actual output perception below tests whether this achieved the intended line. Remaining request includes road forcing; even subtracting the model's road proxy cannot identify an independent intrinsic residual. Warps outside 1m or 0.1rad are explicitly invalid for interpretation. A fixed one-step harmonic correction was selected on training 1a3_01 and applied as `default_refined` wherever prior replay existed; it adjusts the warp through warmup, not a controller. The full source lane-confidence fraction is in `summary.json`; low-confidence camera estimates can contaminate the warp itself.

| Window | Model | Request RMS before / after m/s² | Perceived offset before / after cm | Heading before / after ° | Maximum warp m | Within pose domain |
|---|---|---:|---:|---:|---:|---|
| 000001a3_00 | default | 0.047 / 0.113 | 13.761 / 4.256 | 0.085 / 0.140 | 0.399 | True |
| 000001a3_00 | default_refined | 0.047 / 0.118 | 13.761 / 0.360 | 0.085 / 0.136 | 0.386 | True |
| 000001a3_00 | TSFDOM | 0.040 / 0.088 | 9.509 / 8.339 | 0.061 / 0.133 | 0.399 | True |
| 000001a3_00 | CHEDDAR | 0.049 / 0.104 | 13.806 / 5.824 | 0.051 / 0.122 | 0.399 | True |
| 000001a3_01 | default | 0.093 / 0.095 | 13.597 / 4.918 | 0.094 / 0.090 | 0.289 | True |
| 000001a3_01 | default_refined | 0.093 / 0.111 | 13.597 / 0.836 | 0.094 / 0.096 | 0.359 | True |
| 000001a3_01 | TSFDOM | 0.099 / 0.099 | 9.388 / 6.628 | 0.077 / 0.092 | 0.289 | True |
| 000001a3_01 | CHEDDAR | 0.104 / 0.108 | 12.843 / 5.530 | 0.076 / 0.069 | 0.289 | True |
| 000001a3_02 | default | 0.025 / 0.088 | 11.336 / 6.319 | 0.079 / 0.135 | 0.281 | True |
| 000001a3_02 | default_refined | 0.025 / 0.109 | 11.336 / 0.918 | 0.079 / 0.164 | 0.378 | True |
| 000001a3_02 | TSFDOM | 0.033 / 0.075 | 7.606 / 4.409 | 0.056 / 0.143 | 0.281 | True |
| 000001a3_02 | CHEDDAR | 0.034 / 0.093 | 11.415 / 6.334 | 0.049 / 0.111 | 0.281 | True |
| 000001a7_00 | default | 0.048 / 0.095 | 4.640 / 6.579 | 0.045 / 0.108 | 0.253 | True |
| 000001a7_00 | default_refined | 0.048 / 0.053 | 4.640 / 0.774 | 0.045 / 0.056 | 0.152 | True |
| 000001a7_00 | TSFDOM | 0.058 / 0.090 | 4.014 / 6.757 | 0.033 / 0.115 | 0.253 | True |
| 000001a7_00 | CHEDDAR | 0.052 / 0.086 | 4.539 / 6.724 | 0.033 / 0.099 | 0.253 | True |
| 000001a7_01 | default | 0.060 / 0.074 | 10.702 / 6.084 | 0.124 / 0.127 | 0.359 | True |
| 000001a7_01 | default_refined | 0.060 / 0.094 | 10.702 / 0.404 | 0.124 / 0.149 | 0.505 | True |
| 000001a7_01 | TSFDOM | 0.079 / 0.078 | 8.440 / 4.093 | 0.145 / 0.121 | 0.359 | True |
| 000001a7_01 | CHEDDAR | 0.064 / 0.069 | 11.379 / 5.977 | 0.127 / 0.115 | 0.359 | True |
| 000001a7_02 | default | 0.045 / 0.067 | 8.239 / 6.519 | 0.044 / 0.053 | 0.337 | True |
| 000001a7_02 | default_refined | 0.045 / 0.059 | 8.239 / 0.861 | 0.044 / 0.034 | 0.274 | True |
| 000001a7_02 | TSFDOM | 0.053 / 0.046 | 7.005 / 6.727 | 0.060 / 0.047 | 0.337 | True |
| 000001a7_02 | CHEDDAR | 0.052 / 0.054 | 7.878 / 6.513 | 0.053 / 0.044 | 0.337 | True |
| 000001aa_00 | default | 0.122 / 0.135 | 5.409 / 6.596 | 0.081 / 0.121 | 0.379 | True |
| 000001aa_00 | default_refined | 0.122 / 0.138 | 5.409 / 1.935 | 0.081 / 0.128 | 0.283 | True |
| 000001aa_00 | TSFDOM | 0.122 / 0.129 | 5.732 / 6.207 | 0.101 / 0.097 | 0.379 | True |
| 000001aa_00 | CHEDDAR | 0.118 / 0.130 | 6.178 / 5.810 | 0.088 / 0.079 | 0.379 | True |
| 000001aa_01 | default | 0.054 / 0.124 | 20.973 / 12.401 | 0.124 / 0.188 | 0.330 | True |
| 000001aa_01 | default_refined | 0.054 / 0.161 | 20.973 / 5.688 | 0.124 / 0.153 | 0.627 | True |
| 000001aa_01 | TSFDOM | 0.046 / 0.099 | 16.397 / 12.734 | 0.165 / 0.232 | 0.330 | True |
| 000001aa_01 | CHEDDAR | 0.054 / 0.109 | 21.643 / 14.257 | 0.140 / 0.196 | 0.330 | True |
| 000001aa_02 | default | 0.047 / 0.133 | 8.454 / 6.985 | 0.055 / 0.163 | 0.652 | True |
| 000001aa_02 | default_refined | 0.047 / 0.102 | 8.454 / 0.854 | 0.055 / 0.123 | 0.548 | True |
| 000001aa_02 | TSFDOM | 0.045 / 0.112 | 5.046 / 6.472 | 0.036 / 0.161 | 0.652 | True |
| 000001aa_02 | CHEDDAR | 0.046 / 0.114 | 8.498 / 7.390 | 0.039 / 0.138 | 0.652 | True |
| 000001aa_03 | default | 0.034 / 0.108 | 15.204 / 5.677 | 0.143 / 0.275 | 0.299 | True |
| 000001aa_03 | default_refined | 0.034 / 0.140 | 15.204 / 0.708 | 0.143 / 0.334 | 0.404 | True |
| 000001aa_03 | TSFDOM | 0.034 / 0.077 | 9.271 / 4.033 | 0.131 / 0.278 | 0.299 | True |
| 000001aa_03 | CHEDDAR | 0.039 / 0.119 | 13.778 / 5.617 | 0.100 / 0.206 | 0.299 | True |
| 000001aa_04 | default | 0.078 / 0.107 | 10.088 / 5.191 | 0.138 / 0.167 | 0.261 | True |
| 000001aa_04 | default_refined | 0.078 / 0.146 | 10.088 / 0.614 | 0.138 / 0.219 | 0.344 | True |
| 000001aa_04 | TSFDOM | 0.089 / 0.093 | 4.134 / 7.391 | 0.102 / 0.155 | 0.261 | True |
| 000001aa_04 | CHEDDAR | 0.096 / 0.105 | 5.972 / 5.814 | 0.100 / 0.153 | 0.261 | True |
| 000001aa_05 | default | 0.081 / 0.100 | 9.334 / 4.039 | 0.139 / 0.119 | 0.316 | True |
| 000001aa_05 | default_refined | 0.081 / 0.101 | 9.334 / 0.971 | 0.139 / 0.111 | 0.306 | True |
| 000001aa_05 | TSFDOM | 0.083 / 0.092 | 8.730 / 4.002 | 0.134 / 0.144 | 0.316 | True |
| 000001aa_05 | CHEDDAR | 0.089 / 0.093 | 8.996 / 4.317 | 0.119 / 0.116 | 0.316 | True |
| 000001ab_00 | default | 0.126 / 0.119 | 7.936 / 2.301 | 0.112 / 0.124 | 0.207 | True |
| 000001ab_00 | default_refined | 0.126 / 0.132 | 7.936 / 0.260 | 0.112 / 0.127 | 0.252 | True |
| 000001ab_00 | TSFDOM | 0.122 / 0.107 | 6.957 / 2.554 | 0.079 / 0.117 | 0.207 | True |
| 000001ab_00 | CHEDDAR | 0.114 / 0.099 | 7.861 / 3.434 | 0.076 / 0.111 | 0.207 | True |
| 000001ab_01 | default | 0.144 / 0.131 | 7.397 / 6.485 | 0.121 / 0.095 | 0.197 | True |
| 000001ab_01 | default_refined | 0.144 / 0.156 | 7.397 / 1.064 | 0.121 / 0.100 | 0.208 | True |
| 000001ab_01 | TSFDOM | 0.139 / 0.126 | 5.172 / 5.497 | 0.097 / 0.063 | 0.197 | True |
| 000001ab_01 | CHEDDAR | 0.148 / 0.136 | 6.947 / 6.628 | 0.080 / 0.058 | 0.197 | True |
| 000001ab_02 | default | 0.091 / 0.105 | 9.350 / 3.548 | 0.066 / 0.102 | 0.240 | True |
| 000001ab_02 | default_refined | 0.091 / 0.120 | 9.350 / 0.436 | 0.066 / 0.116 | 0.292 | True |
| 000001ab_02 | TSFDOM | 0.091 / 0.090 | 5.033 / 4.093 | 0.049 / 0.117 | 0.240 | True |
| 000001ab_02 | CHEDDAR | 0.101 / 0.102 | 8.981 / 3.803 | 0.045 / 0.097 | 0.240 | True |
| 000001ab_03 | default | 0.158 / 0.163 | 9.523 / 4.587 | 0.102 / 0.110 | 0.218 | True |
| 000001ab_03 | default_refined | 0.158 / 0.183 | 9.523 / 0.665 | 0.102 / 0.132 | 0.294 | True |
| 000001ab_03 | TSFDOM | 0.168 / 0.168 | 8.868 / 7.830 | 0.082 / 0.131 | 0.218 | True |
| 000001ab_03 | CHEDDAR | 0.174 / 0.173 | 12.189 / 6.188 | 0.066 / 0.108 | 0.218 | True |
| 000001ab_04 | default | 0.062 / 0.073 | 4.838 / 1.499 | 0.063 / 0.123 | 0.365 | True |
| 000001ab_04 | default_refined | 0.062 / 0.073 | 4.838 / 0.633 | 0.063 / 0.116 | 0.360 | True |
| 000001ab_04 | TSFDOM | 0.075 / 0.078 | 5.812 / 2.793 | 0.061 / 0.109 | 0.365 | True |
| 000001ab_04 | CHEDDAR | 0.070 / 0.074 | 6.325 / 3.955 | 0.053 / 0.078 | 0.365 | True |
| 000001ab_05 | default | 0.059 / 0.070 | 5.636 / 3.354 | 0.076 / 0.091 | 0.186 | True |
| 000001ab_05 | default_refined | 0.059 / 0.075 | 5.636 / 0.877 | 0.076 / 0.086 | 0.206 | True |
| 000001ab_05 | TSFDOM | 0.068 / 0.071 | 6.175 / 4.066 | 0.088 / 0.109 | 0.186 | True |
| 000001ab_05 | CHEDDAR | 0.068 / 0.076 | 6.131 / 3.800 | 0.081 / 0.085 | 0.186 | True |
| 000001ab_06 | default | 0.046 / 0.046 | 5.874 / 5.534 | 0.153 / 0.095 | 0.323 | True |
| 000001ab_06 | default_refined | 0.046 / 0.059 | 5.874 / 1.251 | 0.153 / 0.110 | 0.271 | True |
| 000001ab_06 | TSFDOM | 0.046 / 0.042 | 5.279 / 4.786 | 0.160 / 0.127 | 0.323 | True |
| 000001ab_06 | CHEDDAR | 0.051 / 0.055 | 5.037 / 4.920 | 0.125 / 0.091 | 0.323 | True |

Refined-replay decomposition below measures pose removal as the **paired unwarped-minus-refined real request**, rather than assuming the training transfer transports perfectly. Road remains the unwarped model-derived proxy; residual is refined request minus that same road proxy. These are correlated displacement equivalents, not additive causal lane-wander shares. Full covariance is retained in `summary.json`.

| Window | Road equiv cm | Direct pose-removal equiv cm | Remaining residual equiv cm | Refined offset cm |
|---|---:|---:|---:|---:|
| 000001a3_00 | 28.331 | 54.317 | 40.547 | 0.360 |
| 000001a3_01 | 58.774 | 64.191 | 42.489 | 0.836 |
| 000001a3_02 | 21.923 | 49.360 | 39.645 | 0.918 |
| 000001a7_00 | 10.548 | 22.775 | 30.249 | 0.774 |
| 000001a7_01 | 56.022 | 52.845 | 30.303 | 0.404 |
| 000001a7_02 | 39.722 | 25.375 | 49.222 | 0.861 |
| 000001aa_00 | 62.074 | 21.595 | 28.831 | 1.935 |
| 000001aa_01 | 62.437 | 84.134 | 53.669 | 5.688 |
| 000001aa_02 | 61.803 | 33.756 | 40.513 | 0.854 |
| 000001aa_03 | 20.672 | 47.818 | 38.004 | 0.708 |
| 000001aa_04 | 19.692 | 30.838 | 26.615 | 0.614 |
| 000001aa_05 | 35.341 | 31.234 | 33.853 | 0.971 |
| 000001ab_00 | 34.678 | 22.863 | 24.801 | 0.260 |
| 000001ab_01 | 82.440 | 22.111 | 35.922 | 1.064 |
| 000001ab_02 | 47.287 | 44.122 | 21.457 | 0.436 |
| 000001ab_03 | 106.766 | 38.545 | 41.925 | 0.665 |
| 000001ab_04 | 20.620 | 11.152 | 15.395 | 0.633 |
| 000001ab_05 | 61.037 | 20.722 | 20.492 | 0.877 |
| 000001ab_06 | 10.039 | 19.212 | 12.488 | 1.251 |

Road prediction: if motion-generated feedback dominates, the request/offset/heading relationship should change predictably in an independently measured small-offset maneuver; merely improving actuator execution will not erase the model's request spectrum. If substantial intrinsic residual exists, it should persist on a demonstrably constant-offset, constant-heading straight after independent road curvature is removed. The present replay does not satisfy all those conditions.

## 2. Alternate driving models

`catalog_audit.json` records names, refs, runner, selector versions and artifact URLs for every bundle. `source_audit.json` records every source-hash lookup and HTTP outcome. Current branch requires **exact selector version 21**, not “at least 21.” The active QCOM catalog has 78 compatible tinygrad bundles; 21 Chestnut bundles require different hardware and are not comma-3X-only choices. Metadata compatibility does not prove an installation or driving quality.

The 11 retrievable QCOM source graphs are: WD40 (April 09, 2024); North Dakota v2 (April 29, 2024); Notre Dame (July 01, 2024); Rebel Legion Model (July 22, 2026); Get Your Hopes Up Model (July 25, 2026); Rebellious Hope Model (July 27, 2026); RDF Model (August 05, 2026); RDF Model V2 (August 11, 2026); Terrible Super Fantastic Model (August 12, 2026); Terrible Super Fantastic Do Over Model (August 15, 2026); Cheddar Model (November 05, 2026). The other 69 bundle source-hash URLs returned HTTP 404; no lookup timed out.

Both exercised candidates are QCOM, generation 12, tinygrad, selector 21, 20Hz. They are eligible for this branch's Model Manager; no device installation was attempted. Their source ONNX hashes match the adjacent published bundle metadata:

- Cheddar: `5520d198bbbe2736b7fbbc432c2d7a97e306db74214aa1677fed70521b9b9eb4`.
- TSFDOM: `baa30b01cc980a2e616bca5c94f64b64cccce5acfd2456c48bd29da274f4d78b`.

Cheddar uses `new_img` uint8 `[2,6,128,256]`, float32 desire `[8]`, traffic `[1,2]`, action horizons `[1,2]`, and graph-owned image/desire/feature states. TSFDOM uses narrow/wide uint8 `[1,12,128,256]`, float16 desire history `[1,33,8]`, traffic/action `[1,2]` and features `[1,32,512]`. Its adapter mirrors this branch's 5-frame image queue sampled every four, max-pooled 132-frame desire queue, 128-frame feature queue sampled every four, and previous output hidden-state feedback. Full input/output contracts and checkpoint metadata are in `*_contract.json`. Recurrent smoke tests pass, but the legacy adapter has not been compared against a running device's compiled model outputs.

All 11 retrievable QCOM sources were downloaded, hash-checked, loaded and invoked with ONNX Runtime CPU; `qcom_source_contracts.json` records each full input/output contract. Ten produced finite raw outputs in the zero-state smoke call. WD40 produced nonfinite values both at zero and with 25m/s lateral-control inputs, so its CPU usability is unresolved and it is not a video candidate. The remaining 19 retrievable sources are Chestnut graphs and were not downloaded because they are not comma-3X-only model choices. CPU graph execution is not a claim that every contract has a recurrent video adapter. Four QCOM ABI families appear: the 2024 models use float16 `input_imgs`/`big_input_imgs`, desire `[1,100,8]`, lateral control `[1,2]`, previous curvature `[1,100,1]` and features `[1,99,512]`; RLM/GYHUM/RHM/RDF use uint8 images with desire `[1,25,8]` and features `[1,24,512]`; RDFv2/TSF/TSFDOM use desire `[1,33,8]` and features `[1,32,512]`; Cheddar owns its recurrent state inside the graph. Full exact names/shapes/dtypes and model-to-contract mapping are retained in the JSON. The chosen pair spans native/default-like and legacy 20Hz contracts without inventing missing inputs.

### Injected request response

Gain is requested m/s² per metre offset. Positive phase means output leads the injected sinusoid; restoring feedback adds the essential minus sign. Measurements at the four periods use paired unwarped baselines, warm recurrence and derivative-consistent heading.

| Model | Period s | Gain | Raw phase ° | Phase relative to restoring ° |
|---|---:|---:|---:|---:|
| default | 7 | 1.180 | -118.448 | 61.552 |
| default | 10 | 0.892 | -119.109 | 60.891 |
| default | 15 | 0.678 | -128.511 | 51.489 |
| default | 20 | 0.569 | -128.736 | 51.264 |
| CHEDDAR | 7 | 1.137 | -117.991 | 62.009 |
| CHEDDAR | 10 | 0.893 | -125.022 | 54.978 |
| CHEDDAR | 15 | 0.737 | -132.836 | 47.164 |
| CHEDDAR | 20 | 0.677 | -138.413 | 41.587 |
| TSFDOM | 7 | 1.044 | -114.940 | 65.060 |
| TSFDOM | 10 | 0.802 | -118.146 | 61.854 |
| TSFDOM | 15 | 0.609 | -124.679 | 55.321 |
| TSFDOM | 20 | 0.540 | -130.408 | 49.592 |

For default, `a = -0.4435 e - 1.2088 e_dot` matches these complex responses with 9.98% relative error. Its ideal characteristic equation `s² + 1.2088 s + 0.4435 = 0` is stable. This counterexample invalidates raw −120° as sufficient evidence of 120° delay. It does not establish real-loop stability: actuator delay, speed changes, model dynamics and geometry still matter.

Road prediction: TSFDOM's injected response should remain close to default over this band, rather than removing a large alleged phase delay. Any claimed real path improvement must survive the same independent lane observer, not merely a change in the driving model's own lane estimates. Cheddar's measured response and curve behavior below supply the corresponding testable comparison; no unconditional “better model” claim follows.

![Paired simulator indications](plan/findings.png)

## 3. Closed-loop compensation and model indications

All three compensators are standalone, numpy-only, default-disabled APIs, bounded to ±0.15m/s² addition. Invalid input, stale camera >100ms, frame gaps, low confidence, lane-change state or speed below 11.2m/s resets history and returns the original request. They do not dead-reckon through camera dropout. Mac p99 update times are below 0.02ms; comma CPU timing remains unmeasured. The proposed off-by-default param and integration location are documented in the README; no production param was registered.

- Lead: modest fast-minus-slow request filter, 0.45s/3s time constants and 0.5 gain. This advances a 10s sinusoid only modestly; it does not attempt the unjustified 120° cancellation. Filtering acceleration is equivalent to curvature only at constant speed.
- Lane: `-.3*offset - 1.2*speed*heading`. Heading comes from the model lane fit, not a noisy numerical derivative of offset. It remains vulnerable to model heading bias.
- Smooth: half-strength smoothing of the high-passed request, 0.7s smoothing. This is a **residual proxy**, not smoothing of an identified causal road/pose-independent residual.

Shared-reference transport uses the same logged lane offset plus each run's relative dy. It removes model-specific lane-perception scaling from the comparison, but remains conditional on logged lane correctness and the perturbation approximation. Canonical model-lane wander is also retained. Negative deltas mean lower RMS; intervals resample windows, not drives or transport uncertainty.

| Split | Arm | Visible pairs | Canonical wander Δ cm [95% window CI] | Shared-reference wander Δ cm | Body jerk Δ m/s³ |
|---|---|---:|---|---:|---:|
| train | lead | 6 | -0.040 [-0.428, 0.402] | -0.012 | 0.008 |
| train | lane | 6 | -0.406 [-0.913, 0.121] | -0.500 | 0.020 |
| train | smooth | 6 | 0.354 [-0.261, 1.060] | 0.316 | 0.001 |
| train | TSFDOM | 6 | -1.900 [-3.574, -0.193] | -0.007 | 0.002 |
| train | CHEDDAR | 6 | -0.515 [-2.011, 0.762] | -0.311 | 0.002 |
| held-out | lead | 7 | 1.337 [0.451, 2.215] | 1.446 | 0.006 |
| held-out | lane | 7 | -1.744 [-2.182, -1.286] | -1.872 | 0.014 |
| held-out | smooth | 7 | 0.250 [-0.287, 0.793] | 0.223 | 0.002 |
| held-out | TSFDOM | 7 | 0.763 [-1.636, 2.961] | 1.397 | 0.004 |
| held-out | CHEDDAR | 7 | 1.089 [0.049, 2.236] | 0.641 | 0.006 |

Per-run pose departure and failures (do not rank partial/domain-exit runs):

| Window | Arm | Wander cm | Max pose m | Shared-reference cm | Failure |
|---|---|---:|---:|---:|---:|---|
| 000001a3_00 | lead | 14.482 | 0.207 | 14.422 | — |
| 000001a3_00 | lane | 14.796 | 0.261 | 15.013 | — |
| 000001a3_00 | smooth | 13.982 | 0.212 | 13.888 | — |
| 000001a3_00 | TSFDOM | 10.954 | 0.374 | 14.046 | — |
| 000001a3_00 | CHEDDAR | 13.692 | 0.265 | 13.858 | — |
| 000001a3_01 | lead | 14.366 | 0.169 | 13.853 | — |
| 000001a3_01 | lane | 14.417 | 0.183 | 13.781 | — |
| 000001a3_01 | smooth | 13.720 | 0.101 | 13.010 | — |
| 000001a3_01 | TSFDOM | 11.072 | 0.158 | 15.023 | — |
| 000001a3_01 | CHEDDAR | 10.746 | 0.196 | 12.819 | — |
| 000001a3_02 | lead | 3.651 | 0.110 | 3.713 | — |
| 000001a3_02 | lane | 3.599 | 0.144 | 3.296 | — |
| 000001a3_02 | smooth | 5.809 | 0.129 | 5.946 | — |
| 000001a3_02 | TSFDOM | 5.060 | 0.286 | 5.451 | — |
| 000001a3_02 | CHEDDAR | 4.380 | 0.189 | 3.789 | — |
| 000001a7_00 | lead | 4.043 | 0.201 | 4.305 | — |
| 000001a7_00 | lane | 3.823 | 0.157 | 4.207 | — |
| 000001a7_00 | smooth | 4.902 | 0.103 | 5.073 | — |
| 000001a7_00 | TSFDOM | 5.716 | 0.260 | 6.363 | — |
| 000001a7_00 | CHEDDAR | 5.567 | 0.274 | 5.806 | — |
| 000001a7_01 | lead | 11.793 | 0.160 | 11.877 | — |
| 000001a7_01 | lane | 9.700 | 0.168 | 9.410 | — |
| 000001a7_01 | smooth | 10.904 | 0.098 | 10.829 | — |
| 000001a7_01 | TSFDOM | 7.291 | 0.217 | 9.478 | — |
| 000001a7_01 | CHEDDAR | 11.905 | 0.190 | 10.821 | — |
| 000001a7_02 | lead | 6.574 | 0.163 | 6.675 | — |
| 000001a7_02 | lane | 6.377 | 0.240 | 6.211 | — |
| 000001a7_02 | smooth | 7.956 | 0.148 | 8.065 | — |
| 000001a7_02 | TSFDOM | 3.657 | 0.323 | 4.515 | — |
| 000001a7_02 | CHEDDAR | 5.768 | 0.207 | 5.958 | — |
| 000001aa_00 | lead | — | 0.202 | — | — |
| 000001aa_00 | lane | — | 0.148 | — | — |
| 000001aa_00 | smooth | — | 0.192 | — | — |
| 000001aa_00 | TSFDOM | — | 0.401 | — | — |
| 000001aa_00 | CHEDDAR | — | 0.219 | — | — |
| 000001aa_01 | lead | 19.119 | 0.117 | 19.530 | — |
| 000001aa_01 | lane | 17.392 | 0.183 | 17.852 | — |
| 000001aa_01 | smooth | 18.740 | 0.118 | 19.253 | — |
| 000001aa_01 | TSFDOM | 16.477 | 0.153 | 19.899 | — |
| 000001aa_01 | CHEDDAR | 19.121 | 0.145 | 18.528 | — |
| 000001aa_02 | lead | 8.854 | 0.164 | 9.009 | — |
| 000001aa_02 | lane | 7.700 | 0.360 | 7.424 | — |
| 000001aa_02 | smooth | 8.360 | 0.089 | 8.440 | — |
| 000001aa_02 | TSFDOM | 4.363 | 0.174 | 7.892 | — |
| 000001aa_02 | CHEDDAR | 8.758 | 0.196 | 9.206 | — |
| 000001aa_03 | lead | — | 0.161 | — | — |
| 000001aa_03 | lane | — | 0.129 | — | — |
| 000001aa_03 | smooth | — | 0.131 | — | — |
| 000001aa_03 | TSFDOM | — | 0.292 | — | — |
| 000001aa_03 | CHEDDAR | — | 0.179 | — | — |
| 000001aa_04 | lead | 3.947 | 0.253 | 3.760 | — |
| 000001aa_04 | lane | 2.888 | 0.182 | 3.033 | — |
| 000001aa_04 | smooth | 5.478 | 0.159 | 5.287 | — |
| 000001aa_04 | TSFDOM | 8.857 | 0.423 | 7.073 | — |
| 000001aa_04 | CHEDDAR | 5.487 | 0.260 | 4.607 | — |
| 000001aa_05 | lead | — | 0.290 | — | — |
| 000001aa_05 | lane | — | 0.237 | — | — |
| 000001aa_05 | smooth | — | 0.108 | — | — |
| 000001aa_05 | TSFDOM | — | 0.350 | — | — |
| 000001aa_05 | CHEDDAR | — | 0.332 | — | — |
| 000001ab_00 | lead | 7.748 | 0.297 | 7.825 | — |
| 000001ab_00 | lane | 3.727 | 0.314 | 3.336 | — |
| 000001ab_00 | smooth | 6.606 | 0.180 | 6.649 | — |
| 000001ab_00 | TSFDOM | 5.126 | 0.286 | 5.539 | — |
| 000001ab_00 | CHEDDAR | 5.391 | 0.281 | 5.487 | — |
| 000001ab_01 | lead | 7.881 | 0.183 | 8.703 | — |
| 000001ab_01 | lane | 3.283 | 0.311 | 3.443 | — |
| 000001ab_01 | smooth | 4.853 | 0.130 | 5.109 | — |
| 000001ab_01 | TSFDOM | 8.338 | 0.189 | 9.035 | — |
| 000001ab_01 | CHEDDAR | 8.075 | 0.292 | 8.269 | — |
| 000001ab_02 | lead | 6.679 | 0.252 | 6.924 | — |
| 000001ab_02 | lane | 2.747 | 0.160 | 2.792 | — |
| 000001ab_02 | smooth | 4.059 | 0.138 | 4.042 | — |
| 000001ab_02 | TSFDOM | 6.476 | 0.193 | 6.724 | — |
| 000001ab_02 | CHEDDAR | 4.719 | 0.231 | 4.833 | — |
| 000001ab_03 | lead | 10.890 | 0.385 | 11.556 | — |
| 000001ab_03 | lane | 5.813 | 0.295 | 6.201 | — |
| 000001ab_03 | smooth | 9.413 | 0.162 | 9.967 | — |
| 000001ab_03 | TSFDOM | 11.460 | 0.210 | 10.802 | — |
| 000001ab_03 | CHEDDAR | 11.829 | 0.337 | 10.741 | — |
| 000001ab_04 | lead | — | 0.193 | — | — |
| 000001ab_04 | lane | — | 0.201 | — | — |
| 000001ab_04 | smooth | — | 0.203 | — | — |
| 000001ab_04 | TSFDOM | — | 0.328 | — | — |
| 000001ab_04 | CHEDDAR | — | 0.228 | — | — |
| 000001ab_05 | lead | — | 0.204 | — | — |
| 000001ab_05 | lane | — | 0.202 | — | — |
| 000001ab_05 | smooth | — | 0.232 | — | — |
| 000001ab_05 | TSFDOM | — | 0.258 | — | — |
| 000001ab_05 | CHEDDAR | — | 0.186 | — | — |
| 000001ab_06 | lead | — | 0.059 | — | — |
| 000001ab_06 | lane | — | 0.059 | — | — |
| 000001ab_06 | smooth | — | 0.059 | — | — |
| 000001ab_06 | TSFDOM | — | 0.309 | — | — |
| 000001ab_06 | CHEDDAR | — | 0.777 | — | — |

### Curves and turn release

Matched events come only from the logged plan. Width is signed outside-directed mid-curve displacement and requires >80% visible middle support. Alternate-model widths inherit different lane perceptions and cannot independently establish physical path improvement. Body release is relative to the logged plan's 20% crossing; this is not necessarily the driving model's own exit time. Open-loop alternate-model entry/exit deltas below use fixed 20% logged-peak thresholds and retain missing crossings. Full curve-event records, including jerk, are in `summary.json`.

| Window | Arm | Event start s | Run-wide stock / candidate m | Body release stock / candidate s |
|---|---|---:|---:|---:|
| 000001a7_02 | lead | 1905.697 | -0.118 / -0.206 | 1.000 / 1.000 |
| 000001a7_02 | lane | 1905.697 | -0.118 / -0.139 | 1.000 / 1.000 |
| 000001a7_02 | smooth | 1905.697 | -0.118 / -0.077 | 1.000 / 0.900 |
| 000001a7_02 | TSFDOM | 1905.697 | -0.118 / -0.365 | 1.000 / 1.000 |
| 000001a7_02 | CHEDDAR | 1905.697 | -0.118 / -0.242 | 1.000 / 1.000 |
| 000001aa_00 | lead | 624.304 | — / — | 0.300 / 0.300 |
| 000001aa_00 | lane | 624.304 | — / — | 0.300 / 0.300 |
| 000001aa_00 | smooth | 624.304 | — / — | 0.300 / 0.300 |
| 000001aa_00 | TSFDOM | 624.304 | — / — | 0.300 / 0.600 |
| 000001aa_00 | CHEDDAR | 624.304 | — / — | 0.300 / 0.800 |
| 000001aa_02 | lead | 890.113 | 0.219 / 0.104 | — / — |
| 000001aa_02 | lead | 897.813 | 0.043 / 0.022 | — / — |
| 000001aa_02 | lane | 890.113 | 0.219 / 0.201 | — / — |
| 000001aa_02 | lane | 897.813 | 0.043 / 0.054 | — / — |
| 000001aa_02 | smooth | 890.113 | 0.219 / 0.209 | — / — |
| 000001aa_02 | smooth | 897.813 | 0.043 / 0.021 | — / — |
| 000001aa_02 | TSFDOM | 890.113 | 0.219 / 0.146 | — / — |
| 000001aa_02 | TSFDOM | 897.813 | 0.043 / 0.088 | — / — |
| 000001aa_02 | CHEDDAR | 890.113 | 0.219 / 0.127 | — / — |
| 000001aa_02 | CHEDDAR | 897.813 | 0.043 / -0.013 | — / — |
| 000001aa_05 | lead | 1629.142 | -0.226 / -0.269 | 0.600 / 0.100 |
| 000001aa_05 | lane | 1629.142 | -0.226 / -0.285 | 0.600 / 1.000 |
| 000001aa_05 | smooth | 1629.142 | -0.226 / -0.226 | 0.600 / 0.600 |
| 000001aa_05 | TSFDOM | 1629.142 | -0.226 / -0.098 | 0.600 / 0.300 |
| 000001aa_05 | CHEDDAR | 1629.142 | -0.226 / -0.430 | 0.600 / 0.300 |
| 000001ab_01 | lead | 229.463 | -0.278 / -0.333 | — / — |
| 000001ab_01 | lane | 229.463 | -0.278 / -0.182 | — / — |
| 000001ab_01 | smooth | 229.463 | -0.278 / -0.235 | — / — |
| 000001ab_01 | TSFDOM | 229.463 | -0.278 / -0.366 | — / — |
| 000001ab_01 | CHEDDAR | 229.463 | -0.278 / -0.297 | — / — |
| 000001ab_03 | lead | 644.930 | -0.242 / -0.365 | 0.200 / 0.000 |
| 000001ab_03 | lead | 696.730 | -0.063 / -0.334 | 0.300 / 0.300 |
| 000001ab_03 | lane | 644.930 | -0.242 / -0.205 | 0.200 / 0.000 |
| 000001ab_03 | lane | 696.730 | -0.063 / -0.084 | 0.300 / 0.000 |
| 000001ab_03 | smooth | 644.930 | -0.242 / -0.198 | 0.200 / 0.200 |
| 000001ab_03 | smooth | 696.730 | -0.063 / -0.101 | 0.300 / 0.300 |
| 000001ab_03 | TSFDOM | 644.930 | -0.242 / -0.164 | 0.200 / 0.200 |
| 000001ab_03 | TSFDOM | 696.730 | -0.063 / -0.039 | 0.300 / 0.300 |
| 000001ab_03 | CHEDDAR | 644.930 | -0.242 / -0.078 | 0.200 / 0.200 |
| 000001ab_03 | CHEDDAR | 696.730 | -0.063 / -0.130 | 0.300 / 0.300 |

Body entry/exit against the fixed logged 20% threshold (20Hz crossings; missing entry is retained):

| Window | Arm | Curve start s | Body entry Δ s | Body exit Δ s |
|---|---|---:|---:|---:|
| 000001a7_02 | lead | 1905.697 | — | 1.000 |
| 000001a7_02 | lane | 1905.697 | — | 0.650 |
| 000001a7_02 | smooth | 1905.697 | — | 0.650 |
| 000001a7_02 | TSFDOM | 1905.697 | — | 1.000 |
| 000001a7_02 | CHEDDAR | 1905.697 | — | 1.000 |
| 000001aa_00 | lead | 624.304 | 0.300 | 0.250 |
| 000001aa_00 | lane | 624.304 | 0.350 | 0.250 |
| 000001aa_00 | smooth | 624.304 | 0.250 | 0.250 |
| 000001aa_00 | TSFDOM | 624.304 | 0.300 | 0.550 |
| 000001aa_00 | CHEDDAR | 624.304 | 0.650 | 0.750 |
| 000001aa_02 | lead | 890.113 | 0.050 | — |
| 000001aa_02 | lead | 897.813 | — | — |
| 000001aa_02 | lane | 890.113 | 0.050 | — |
| 000001aa_02 | lane | 897.813 | — | — |
| 000001aa_02 | smooth | 890.113 | 0.450 | — |
| 000001aa_02 | smooth | 897.813 | — | — |
| 000001aa_02 | TSFDOM | 890.113 | -0.100 | — |
| 000001aa_02 | TSFDOM | 897.813 | — | — |
| 000001aa_02 | CHEDDAR | 890.113 | -0.100 | — |
| 000001aa_02 | CHEDDAR | 897.813 | — | — |
| 000001aa_05 | lead | 1629.142 | 1.150 | -0.150 |
| 000001aa_05 | lane | 1629.142 | -0.200 | 0.850 |
| 000001aa_05 | smooth | 1629.142 | 1.150 | 0.150 |
| 000001aa_05 | TSFDOM | 1629.142 | 1.300 | 0.150 |
| 000001aa_05 | CHEDDAR | 1629.142 | 1.150 | 0.150 |
| 000001ab_01 | lead | 229.463 | 1.150 | — |
| 000001ab_01 | lane | 229.463 | 1.150 | — |
| 000001ab_01 | smooth | 229.463 | 0.650 | — |
| 000001ab_01 | TSFDOM | 229.463 | 1.200 | — |
| 000001ab_01 | CHEDDAR | 229.463 | 1.200 | — |
| 000001ab_03 | lead | 644.930 | — | -0.000 |
| 000001ab_03 | lead | 696.730 | 0.450 | 0.250 |
| 000001ab_03 | lane | 644.930 | — | -0.000 |
| 000001ab_03 | lane | 696.730 | 0.450 | -0.000 |
| 000001ab_03 | smooth | 644.930 | — | 0.200 |
| 000001ab_03 | smooth | 696.730 | -0.050 | 0.300 |
| 000001ab_03 | TSFDOM | 644.930 | — | 0.150 |
| 000001ab_03 | TSFDOM | 696.730 | 0.450 | 0.250 |
| 000001ab_03 | CHEDDAR | 644.930 | — | 0.150 |
| 000001ab_03 | CHEDDAR | 696.730 | 0.450 | 0.300 |

| Window | Model | Curve start s | Request entry Δ s | Request exit Δ s |
|---|---|---:|---:|---:|
| 000001a7_02 | TSFDOM | 1905.697 | — | 0.050 |
| 000001a7_02 | CHEDDAR | 1905.697 | — | -0.200 |
| 000001aa_00 | TSFDOM | 624.304 | 0.100 | -0.200 |
| 000001aa_00 | CHEDDAR | 624.304 | -0.350 | 0.350 |
| 000001aa_02 | TSFDOM | 890.113 | -0.650 | — |
| 000001aa_02 | TSFDOM | 897.813 | — | — |
| 000001aa_02 | CHEDDAR | 890.113 | -0.850 | — |
| 000001aa_02 | CHEDDAR | 897.813 | — | — |
| 000001aa_05 | TSFDOM | 1629.142 | 1.000 | -0.050 |
| 000001aa_05 | CHEDDAR | 1629.142 | 0.850 | 0.150 |
| 000001ab_01 | TSFDOM | 229.463 | -0.400 | — |
| 000001ab_01 | CHEDDAR | 229.463 | -0.100 | — |
| 000001ab_03 | TSFDOM | 644.930 | — | -0.200 |
| 000001ab_03 | TSFDOM | 696.730 | -0.050 | -0.400 |
| 000001ab_03 | CHEDDAR | 644.930 | — | -0.250 |
| 000001ab_03 | CHEDDAR | 696.730 | -0.300 | 0.150 |

### Recommendations and falsifiable road tests

1. **Do not treat phase as delay without the restoring sign.** A measured offset/heading response resembling PD damping is compatible with the existing raw phase. A real logged maneuver should show request opposing both offset and outward heading; a large phase-delay claim needs additional delayed dynamics beyond that restoring law.
2. **Prioritize the lane correction for a reviewed road A/B; keep it disabled pending integration.** The falsifiable new hypothesis is about **1–2cm lower** common-observer 7–20s RMS on similar highway windows, with no consistent curve-entry/release delay and no >10% body-jerk increase. A repeatable absence of that improvement rejects the transported simulator indication. This is separate from the locked S3/S5 forecasts. The lead-filter hypothesis predicts worse wander (order 1cm); smoothing has no reliable improvement indication. A matched road A/B must reduce 7–20s shared-observer lane RMS on independent drives without worsening signed curve width, entry/exit crossing times or body jerk. If the simulator's small changes do not replicate, reject the candidate rather than tuning on the same held-out windows. No forecast in the locked vsim3 file is changed or supplemented as if this were its road outcome.
3. **Use a common observer for model A/B.** If lower model-reported wander comes primarily from perception scaling, independent lane RMS and body motion will not show the same reduction. This is directly falsifiable. Neither full-population shared-reference held-out mean improved (TSFDOM +1.397cm, Cheddar +0.641cm), so the new road hypothesis is no consistent >=2cm improvement from either model on comparable repeated drives. A repeatable >=2cm reduction with one common observer would falsify that hypothesis; it is not a change to the locked controller forecasts.
4. **Do not deploy the smoothing proxy as “residual removal.”** Extend the <1cm training replay result to independent geometry and retained curve timing. Failure to reach constant-offset perception is an experimental failure, not evidence of intrinsic model noise.

## Verification and exact continuation

Code entry points and integration instructions: `tools/odyssey_sim/plan/README.md`. Tests cover disabled/bad/stale/dropout/low-speed/lane-change fallback, correction direction/bounds, filter reentry and phase/DC behavior, restoring-phase interpretation, shared-reference sign/visibility, and both real recurrent ONNX adapters. Runtime benchmark and complete contracts are in `plan/`. Review the final `checks.log`, `lint.log`, `source_manifest.json` and raw campaign logs for mechanical evidence.

Outstanding identification limits are substantive: no independent road/pose truth, no held-out injected-transfer validation, no production compiled-model equivalence check for TSFDOM, unvalidated transport of the one-window injected response, CameraOffset=0-only replay (the now-set +0.12 offset is not validated here), and a failed simulator spectrum gate. The exact next offline step is independent road/heading closure plus compiled/source model equivalence on the same frames. The recommended next road step is a separately reviewed default-versus-lane-correction A/B scored by one fixed observer, holding driving model and CameraOffset constant. Model A/B is secondary: neither tested alternative improved the held-out shared-reference mean. Nothing is enabled on the van.

## Sources

- [Current QCOM catalog](https://raw.githubusercontent.com/sunnypilot/sunnypilot-models/gh-pages/docs/driving_models_v25.json) and [Chestnut catalog](https://raw.githubusercontent.com/sunnypilot/sunnypilot-models/gh-pages/docs/driving_models_chestnut_v28.json): artifact formats, versions and model refs.
- [sunnypilot published model artifacts](https://huggingface.co/datasets/sunnypilot/sunnypilot_models_v1/tree/main/models): adjacent metadata records original ONNX hashes. Every queried metadata URL is preserved in `source_audit.json`.
- [comma public LFS source for Cheddar](https://huggingface.co/buckets/commaai/openpilot-lfs/resolve/sha256/5520d198bbbe2736b7fbbc432c2d7a97e306db74214aa1677fed70521b9b9eb4) and [TSFDOM](https://huggingface.co/buckets/commaai/openpilot-lfs/resolve/sha256/baa30b01cc980a2e616bca5c94f64b64cccce5acfd2456c48bd29da274f4d78b): downloaded bytes verified by SHA-256.
- Local runtime truth: `ODYSSEY.md`, `sunnypilot/models/helpers.py`, `modeld_v2/compile_modeld.py`, `stock_dependencies.py`, `model_adapters.py`, and `modeld.py`. The earlier wander/vsim reports retain their original findings and limitations.


Default unwarped fidelity: 19 windows; desired-curvature r range 0.990074–0.999831. Alternate-model agreement with the logged default is not a fidelity gate for those different networks.

Evidence counts: 19 default decompositions; 95 closed-loop runs; 8 alternate transfer measurements; 19 refined path replays.
