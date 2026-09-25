# ASTRANAV V1 — Reference Integration

V1 is a clean hybrid navigation core assembled from the design patterns identified across the SIH reference implementations.

## Adopted patterns

- IO-VNBD / Priyanshu-style motion learning: causal windowed IMU features, speed as the learned quantity, never direct latitude/longitude regression.
- Harsh / Kishoor-style engineering: timestamp validation, causal processing, replay-friendly deterministic state transitions, innovation gating and sensor-health checks.
- Obito / Tanu-style hybrid fusion: neural velocity is treated as a measurement with confidence/variance rather than replacing the estimator.
- Mahiibhardwaj MARK-V: candidate-road scoring, HMM/Viterbi sequence matching and sticky road locking.
- hmm183 MARK-V architecture: edge ONNX inference, NHC/ZUPT/ZARU constraints, navigation-mode management and offline-first map concepts.
- Vikash / Fadel-style classical foundation: inertial mechanization and frame/calibration separation.

## V1 boundary

The large IMM-UKF/RBPF/FGO hierarchy and other research-heavy components are deliberately not copied into V1. They can be evaluated later against deterministic replay.

## ML contract

Default expected TCN input is: accel x/y/z, gravity x/y/z, gyro yaw/pitch/roll, linear accel x/y/z.

Sampling: 10 Hz. Window: 20 samples (2 seconds).

The Android runtime loads assets/models/astranav_tcn_speed.onnx when supplied. The repository intentionally does not fabricate or silently substitute model weights.

## Integrity

Every learned or external measurement must pass sanity checks and can be rejected by innovation gating. Confidence should control measurement variance.

## Important

Reference-repository benchmark numbers are not ASTRANAV benchmark results. They must be reproduced on the ASTRANAV held-out IO-VNBD evaluation before being quoted.
