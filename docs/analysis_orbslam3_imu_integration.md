# ORB-SLAM3 IMU Integration: A Deep Technical Analysis

## Table of Contents

- [1. Overview](#1-overview)
- [2. IMU Sensor Model and Noise](#2-imu-sensor-model-and-noise)
- [3. Timestamp Alignment and IMU-Camera Synchronization](#3-timestamp-alignment-and-imu-camera-synchronization)
- [4. IMU Preintegration Theory](#4-imu-preintegration-theory)
- [5. Implementation: The Preintegrated Class](#5-implementation-the-preintegrated-class)
- [6. Visual-Inertial Initialization](#6-visual-inertial-initialization)
- [7. How IMU Guides Tracking](#7-how-imu-guides-tracking)
- [8. How IMU Guides Local Mapping](#8-how-imu-guides-local-mapping)
- [9. How IMU Guides Loop Closing and Map Merging](#9-how-imu-guides-loop-closing-and-map-merging)
- [10. g2o Custom Types for IMU](#10-g2o-custom-types-for-imu)
- [11. The Complete IMU Data Flow](#11-the-complete-imu-data-flow)
- [12. Key Equations Summary](#12-key-equations-summary)
- [13. References](#13-references)

---

## 1. Overview

ORB-SLAM3 is the first SLAM system to support visual, visual-inertial, and multi-map SLAM across monocular, stereo, and RGB-D cameras. Its IMU integration is based on the **Maximum-a-Posteriori (MAP) estimation** framework and builds on the **IMU preintegration theory** by Forster et al. (2015/2017).

The key insight is that IMU measurements between two camera frames can be **preintegrated** into a single relative motion constraint, avoiding re-integration when the linearization point (bias estimate) changes. This preintegrated measurement becomes a factor in the optimization graph alongside visual reprojection errors.

### Supported Visual-Inertial Modes

| Mode | Sensor | Key Challenge |
|------|--------|---------------|
| **Mono-Inertial** | Monocular camera + IMU | Scale is unobservable from vision alone; IMU provides scale |
| **Stereo-Inertial** | Stereo camera + IMU | Scale is observable from stereo, but IMU improves robustness |

### State Vector Per Keyframe

In visual-inertial mode, each keyframe `i` carries an extended state:

```
x_i = { R_i, p_i, v_i, b_g_i, b_a_i }
```

Where:
- `R_i ∈ SO(3)` — Orientation (rotation from body to world)
- `p_i ∈ R^3` — Position in world frame
- `v_i ∈ R^3` — Velocity in world frame
- `b_g_i ∈ R^3` — Gyroscope bias
- `b_a_i ∈ R^3` — Accelerometer bias

This is **15 degrees of freedom per keyframe** (compared to 6 DOF in pure visual SLAM).

---

## 2. IMU Sensor Model and Noise

### Raw IMU Measurements

An IMU provides two types of measurements at high frequency (typically 200-1000 Hz):

**Gyroscope** (angular velocity):
```
ω_measured = ω_true + b_g + η_g
```

**Accelerometer** (specific force = linear acceleration - gravity):
```
a_measured = R_wb^T (a_true - g) + b_a + η_a
```

Where:
- `b_g, b_a` — Slowly time-varying biases (modeled as random walks)
- `η_g, η_a` — White noise (measurement noise)
- `g = [0, 0, -9.81]^T` — Gravity vector in world frame
- `R_wb` — Rotation from world to body frame

### Bias Random Walk Model

Biases evolve as random walks:
```
ḃ_g = η_bg    (gyroscope bias drift)
ḃ_a = η_ba    (accelerometer bias drift)
```

### IMU Noise Parameters (from calibration)

ORB-SLAM3 requires these noise parameters in the configuration file:

| Parameter | Symbol | Typical Value | Unit |
|-----------|--------|---------------|------|
| Gyroscope noise density | σ_g | 1.6e-4 | rad/s/√Hz |
| Accelerometer noise density | σ_a | 2.0e-3 | m/s²/√Hz |
| Gyroscope random walk | σ_bg | 1.9e-5 | rad/s²/√Hz |
| Accelerometer random walk | σ_ba | 3.0e-3 | m/s³/√Hz |

These are stored in the `IMU::Calib` class alongside the body-to-camera extrinsic transformation `T_bc`.

---

## 3. Timestamp Alignment and IMU-Camera Synchronization

This is one of the most critical and failure-prone aspects of ORB-SLAM3's IMU integration.

### 3.1 Requirements

ORB-SLAM3 **assumes that IMU and camera timestamps are already synchronized** in the same clock domain. The system does **not** internally estimate or compensate for a time offset between sensors.

Key requirements:
- Camera timestamps and IMU timestamps must use the **same clock reference**
- IMU timestamps must be **monotonically increasing**
- IMU rate should be **≥ 100 Hz** (200 Hz typical), camera at **10-30 Hz**
- If the camera-IMU time offset is known (e.g., from Kalibr), the user must **apply it before feeding data** to ORB-SLAM3

### 3.2 IMU Buffering

IMU measurements arrive asynchronously (typically at 200-1000 Hz) and are pushed into a thread-safe queue:

```cpp
// In Tracking.cc
void Tracking::GrabImuData(const IMU::Point &imuMeasurement) {
    unique_lock<mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

// Each IMU::Point contains:
struct Point {
    Eigen::Vector3f a;   // accelerometer reading (m/s^2)
    Eigen::Vector3f w;   // gyroscope reading (rad/s)
    double t;            // timestamp
};
```

When a new image arrives at timestamp `t_image`:
1. All IMU measurements with timestamps between `t_last_image` and `t_image` are extracted from `mlQueueImuData`
2. Measurements older than the previous frame are discarded
3. The boundary sample (at or after the current frame timestamp) is included

### 3.3 Temporal Integration Boundaries

The preintegration interval is bounded by image timestamps:

```
IMU measurements:  ---|------|------|------|------|------|------|------|---
                      t_imu1 t_imu2 t_imu3 t_imu4 t_imu5 t_imu6 t_imu7

Camera frames:     ------[Frame i]---------------------------[Frame j]---
                         t_i                                  t_j

Preintegration window:   |<-------- Δt_ij = t_j - t_i ------->|
```

All IMU measurements with `t_i < t_imu ≤ t_j` are integrated into the preintegrated measurement `Δ_ij`.

### 3.4 Trapezoidal Interpolation at Frame Boundaries

ORB-SLAM3 **does** perform interpolation at frame boundaries using a trapezoidal scheme. In `PreintegrateIMU()`, four cases are handled:

| Case | Condition | Behavior |
|------|-----------|----------|
| First & not last | `i==0 && i<n-1` | Interpolates acceleration/angular velocity at the previous frame's timestamp boundary |
| Middle | `0<i<n-1` | Standard midpoint (trapezoidal) averaging between consecutive samples |
| Last & not first | `i>0 && i==n-1` | Interpolates at the current frame's timestamp boundary |
| Only one | `i==0 && i==n-1` | Uses single measurement directly |

The boundary interpolation formula for the first sample:
```cpp
tab = t[i+1] - t[i];
tini = t[i] - t_prev_frame;
acc = 0.5f * (a[i] + a[i+1] - (a[i+1]-a[i]) * (tini/tab));  // trapezoidal interp
```

This ensures preintegration windows align exactly with frame timestamps rather than raw IMU sample timestamps.

### 3.5 Dual Preintegration

`PreintegrateIMU()` maintains **two** preintegration objects simultaneously:

```cpp
mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);
pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel, tstep);
```

| Object | From | To | Used For |
|--------|------|----|----------|
| `mpImuPreintegratedFromLastKF` | Last keyframe | Current frame | Optimization (local BA, loop closing) |
| `mpImuPreintegratedFrame` | Last frame | Current frame | Frame-to-frame pose prediction |

This allows the system to predict from the most recent optimized state (keyframe) when available, or from the last frame when the optimizer hasn't run yet.

### 3.6 Common Timestamp Issues

From the ORB-SLAM3 issue tracker, the most common failures are:
- **"Frame with a timestamp older than previous frame detected!"** — Non-monotonic camera timestamps
- **IMU initialization fails** — Time offset between IMU and camera not compensated
- **Tracking resets frequently** — Temporal misalignment causes incorrect preintegration intervals

---

## 4. IMU Preintegration Theory

### 4.1 The Core Problem

Given IMU measurements between keyframes `i` (at time `t_i`) and `j` (at time `t_j`), we want to compute the relative rotation, velocity, and position without depending on the absolute states at `i`.

### 4.2 Continuous-Time IMU Kinematics

The continuous-time motion model in the world frame:

```
Ṙ(t) = R(t) · [ω(t) - b_g]×      (rotation kinematics)
v̇(t) = R(t) · (a(t) - b_a) + g    (velocity kinematics)
ṗ(t) = v(t)                         (position kinematics)
```

Where `[·]×` denotes the skew-symmetric matrix.

### 4.3 Preintegrated Measurements

Instead of integrating in the world frame (which depends on the state at `i`), Forster et al. define **preintegrated measurements** in the body frame of keyframe `i`:

```
ΔR_ij = R_i^T · R_j                                              ... (1)
Δv_ij = R_i^T · (v_j - v_i - g·Δt_ij)                           ... (2)
Δp_ij = R_i^T · (p_j - p_i - v_i·Δt_ij - ½·g·Δt_ij²)          ... (3)
```

Where `Δt_ij = t_j - t_i`.

These can be computed incrementally from IMU measurements **without knowing `R_i`, `v_i`, `p_i`**:

```
ΔR_ij ≈ ∏_{k=i}^{j-1} Exp((ω_k - b_g) · δt)
Δv_ij ≈ Σ_{k=i}^{j-1} ΔR_ik · (a_k - b_a) · δt
Δp_ij ≈ Σ_{k=i}^{j-1} [Δv_ik · δt + ½ · ΔR_ik · (a_k - b_a) · δt²]
```

Where `δt` is the time between consecutive IMU samples.

### 4.4 Bias Correction via First-Order Update

When the bias estimate changes from `b̄` (used during integration) to `b̄ + δb`, the preintegrated measurements are updated using a **first-order Taylor expansion** (avoiding full re-integration):

```
ΔR_ij(b_g + δb_g) ≈ ΔR_ij(b̄_g) · Exp(J_R^g · δb_g)
Δv_ij(b + δb)     ≈ Δv_ij(b̄) + J_v^g · δb_g + J_v^a · δb_a
Δp_ij(b + δb)     ≈ Δp_ij(b̄) + J_p^g · δb_g + J_p^a · δb_a
```

Where `J_R^g`, `J_v^g`, `J_v^a`, `J_p^g`, `J_p^a` are the **bias-correction Jacobians** computed during integration.

### 4.5 Noise Covariance Propagation

The covariance of the preintegrated measurements is propagated using a discrete-time linear system:

```
Σ_k+1 = A_k · Σ_k · A_k^T + B_k · Q · B_k^T
```

Where:
- `Σ_k` is the 9×9 covariance of `[δφ, δv, δp]` (rotation error, velocity error, position error)
- `A_k` is the state transition matrix (9×9)
- `B_k` is the noise input matrix (9×6)
- `Q` is the measurement noise covariance (6×6, from gyro and accel noise densities)

The matrices `A_k` and `B_k` at each IMU step are:

```
        | ΔR_k^T              0       0   |
A_k =   | -ΔR_ik·[a_k-b_a]×  I       0   |
        | -½ΔR_ik·[a_k-b_a]×δt  I·δt  I  |

        | J_r^k·δt   0          |
B_k =   | 0          ΔR_ik·δt   |
        | 0          ½ΔR_ik·δt² |
```

Where `J_r^k` is the right Jacobian of SO(3) for the rotation increment `(ω_k - b_g)·δt`.

---

## 5. Implementation: The `Preintegrated` Class

### 5.1 Class Structure (`ImuTypes.h`)

```
IMU::Preintegrated
├── Member Variables:
│   ├── dR        : cv::Mat (3×3)  — Preintegrated rotation ΔR_ij
│   ├── dV        : cv::Mat (3×1)  — Preintegrated velocity Δv_ij
│   ├── dP        : cv::Mat (3×1)  — Preintegrated position Δp_ij
│   ├── JRg       : cv::Mat (3×3)  — Jacobian ∂ΔR/∂b_g
│   ├── JVg       : cv::Mat (3×3)  — Jacobian ∂Δv/∂b_g
│   ├── JVa       : cv::Mat (3×3)  — Jacobian ∂Δv/∂b_a
│   ├── JPg       : cv::Mat (3×3)  — Jacobian ∂Δp/∂b_g
│   ├── JPa       : cv::Mat (3×3)  — Jacobian ∂Δp/∂b_a
│   ├── C         : cv::Mat (15×15) — Covariance matrix
│   ├── Info      : cv::Mat (15×15) — Information matrix (C^{-1})
│   ├── Nga       : cv::Mat (6×6)  — Measurement noise covariance
│   ├── NgaWalk   : cv::Mat (6×6)  — Bias random walk noise covariance
│   ├── b         : Bias           — Bias at linearization point
│   ├── bu        : Bias           — Updated bias
│   ├── db        : cv::Mat (6×1)  — Bias change (bu - b)
│   ├── dT        : float          — Total integration time Δt_ij
│   └── mvMeasurements : vector<integrable>  — Stored raw IMU measurements
│
├── Key Methods:
│   ├── IntegrateNewMeasurement(acc, angVel, dt)
│   ├── GetDeltaRotation(b_g)      — Returns ΔR with bias correction
│   ├── GetDeltaVelocity(b_g, b_a) — Returns Δv with bias correction
│   ├── GetDeltaPosition(b_g, b_a) — Returns Δp with bias correction
│   ├── GetUpdatedDeltaRotation()   — Uses current bu for correction
│   ├── GetUpdatedDeltaVelocity()   — Uses current bu for correction
│   ├── GetUpdatedDeltaPosition()   — Uses current bu for correction
│   ├── Reintegrate()               — Full re-integration with new bias
│   └── MergePrevious(prev)         — Merge with a previous preintegration
```

### 5.2 `IntegrateNewMeasurement` — Step by Step

This is the core function called for every IMU sample:

```cpp
void Preintegrated::IntegrateNewMeasurement(
    const cv::Point3f &acceleration,
    const cv::Point3f &angVel,
    const float &dt)
{
    // 1. Store raw measurement for potential re-integration
    mvMeasurements.push_back(integrable(acceleration, angVel, dt));

    // 2. Bias-correct the raw measurements
    cv::Mat acc = (cv::Mat_<float>(3,1) << acceleration.x, ...) - b.ba;  // a - b_a
    cv::Mat accW = (cv::Mat_<float>(3,1) << angVel.x, ...) - b.bw;      // ω - b_g

    // 3. Compute rotation increment
    IntegratedRotation dRi(accW, dt);  // Exp((ω - b_g) · dt)
    // Also computes right Jacobian Jr via Rodrigues formula

    // 4. UPDATE ORDER MATTERS: P first, then V, then R
    //    (because P depends on old V and R, V depends on old R)

    // 4a. Update position Jacobians BEFORE modifying dR and dV
    JPa = JPa + JVa * dt - 0.5f * dR * dt * dt;                  // ∂Δp/∂b_a
    JPg = JPg + JVg * dt - 0.5f * dR * skew(acc) * JRg * dt*dt;  // ∂Δp/∂b_g

    // 4b. Update position: dP += dV·dt + ½·dR·(a-b_a)·dt²
    dP = dP + dV * dt + 0.5f * dR * acc * (dt * dt);

    // 4c. Update velocity Jacobians
    JVa = JVa - dR * dt;                        // ∂Δv/∂b_a
    JVg = JVg - dR * skew(acc) * JRg * dt;      // ∂Δv/∂b_g

    // 4d. Update velocity: dV += dR·(a-b_a)·dt
    dV = dV + dR * acc * dt;

    // 4e. Update rotation Jacobian
    JRg = dRi.deltaR.t() * JRg - dRi.rightJ * dt;  // ∂ΔR/∂b_g

    // 4f. Update rotation: dR = dR · Exp((ω-b_g)·dt)
    dR = NormalizeRotation(dR * dRi.deltaR);

    // 5. Propagate covariance (9×9 state: [δφ, δv, δp])
    //    Using: Σ_{k+1} = A·Σ·A^T + B·Nga·B^T
    //    A and B matrices as described in Section 4.5

    // 6. Accumulate total time
    dT += dt;
}
```

### 5.3 Bias-Corrected Retrieval

When the optimizer updates bias estimates, the preintegrated values are corrected without re-integration:

```cpp
cv::Mat Preintegrated::GetDeltaRotation(const Bias &b_)
{
    // First-order correction: ΔR(b̄+δb) ≈ ΔR(b̄) · Exp(JRg · δb_g)
    cv::Mat dbg = (cv::Mat_<float>(3,1) << b_.bwx-b.bwx, b_.bwy-b.bwy, b_.bwz-b.bwz);
    return NormalizeRotation(dR * ExpSO3(JRg * dbg));
}

cv::Mat Preintegrated::GetDeltaVelocity(const Bias &b_)
{
    cv::Mat dbg = ...;  // δb_g
    cv::Mat dba = ...;  // δb_a
    return dV + JVg * dbg + JVa * dba;
}

cv::Mat Preintegrated::GetDeltaPosition(const Bias &b_)
{
    cv::Mat dbg = ...;  // δb_g
    cv::Mat dba = ...;  // δb_a
    return dP + JPg * dbg + JPa * dba;
}
```

### 5.4 The `IntegratedRotation` Helper

```cpp
IntegratedRotation::IntegratedRotation(const cv::Point3f &angVel, const float &dt)
{
    const float angle = cv::norm(angVel) * dt;  // ||ω||·dt

    if (angle < 1e-4) {
        // Small angle approximation
        deltaR = cv::Mat::eye(3,3) + skew(angVel * dt);
        rightJ = cv::Mat::eye(3,3);  // Jr ≈ I for small angles
    } else {
        // Rodrigues formula for Exp(ω·dt) and right Jacobian Jr
        cv::Mat axis = angVel / cv::norm(angVel);
        deltaR = /* Rodrigues(axis, angle) */;
        rightJ = /* Right Jacobian formula */;
    }
}
```

---

## 6. Visual-Inertial Initialization

The initialization is ORB-SLAM3's **most novel contribution** for IMU integration. Unlike VINS-Mono (which uses a multi-step algebraic approach), ORB-SLAM3 uses a **MAP estimation** approach.

### 6.1 Three-Phase Pipeline with Progressive Prior Relaxation

ORB-SLAM3's initialization uses **three progressive phases** with decreasing prior strength on biases, controlled by `priorG` (gyro prior) and `priorA` (accel prior):

```
Phase 1: Initial Estimation (at ~2s)  [priorG=1e2, priorA=1e10]
├── Requires ≥10 keyframes spanning >2s (mono) or >1s (stereo/RGBD)
├── Estimate gravity direction from velocity preintegration:
│   dirG = -Σ R_prev · ΔVelocity   (gravity emerges from velocity residuals)
│   Rwg = rotation aligning z-axis with estimated gravity
├── Run Optimizer::InertialOptimization with strong bias priors
│   ├── VertexGDir (2 DOF): gravity direction on unit sphere
│   ├── VertexScale (1 DOF): exponential parameterization (ensures s>0)
│   ├── EdgeInertialGS: connects poses+vel+bias+gravity+scale
│   └── Biases strongly constrained → essentially assumed zero
├── Scale map: p_i → s · p_i, X_j → s · X_j
├── Rotate world frame: z-axis aligned with gravity
└── Result: coarse metric scale, gravity direction

          ↓

Phase 2: VIBA 1 (at ~5s)  [priorG=1.0, priorA=1e5]
├── Relaxes gyro bias prior (now free to move)
├── Still constrains accelerometer bias (harder to estimate)
├── Runs full visual-inertial BA (FullInertialBA)
└── Result: refined scale, biases beginning to converge

          ↓

Phase 3: VIBA 2 (at ~15s)  [priorG=0.0, priorA=0.0]
├── Removes ALL bias priors → fully observable system
├── Runs full visual-inertial BA
└── Result: fully initialized, biases converged

          ↓

Periodic Scale Refinement (at 25s, 35s, 45s, 55s, 65s, 75s)
└── Runs InertialOptimization for scale+gravity only (ScaleRefinement)
```

### 6.2 Gravity Direction Estimation (from source code)

The initial gravity direction is estimated from the velocity preintegration residuals. The key insight: the preintegrated velocity change between keyframes includes a gravity term `R_k · g · dt`. By summing `R_k^T · ΔV` across many keyframes, the gravity direction emerges:

```cpp
// In LocalMapping::InitializeIMU()
for each keyframe pair (prev, curr):
    dirG -= R_prev * pInt->GetDeltaVelocity();   // Accumulate gravity-induced velocity
    velocity = (p_curr - p_prev) / dt;            // Initial velocity estimate

// Compute rotation from visual frame to gravity-aligned frame
dirG = dirG / |dirG|;                  // normalize
gI = [0, 0, -1];                      // canonical gravity direction
v = gI.cross(dirG);                   // rotation axis
ang = acos(gI.dot(dirG));             // rotation angle
Rwg = Exp(v * ang / |v|);             // gravity-to-world rotation
```

### 6.3 Inertial-Only Optimization — Detail

This uses the special `EdgeInertialGS` edge that includes gravity direction and scale as optimizable vertices:

```
Graph for InertialOptimization:

For each consecutive keyframe pair (KF_i, KF_{i+1}):
    EdgeInertialGS connects 8 vertices:
        [0] VP_i      (pose i — fixed from visual SLAM)
        [1] VV_i      (velocity i — optimized)
        [2] VG         (gyro bias — shared, optimized)
        [3] VA         (accel bias — shared, optimized)
        [4] VP_{i+1}  (pose i+1 — fixed)
        [5] VV_{i+1}  (velocity i+1 — optimized)
        [6] VertexGDir (2 DOF gravity direction — optimized)
        [7] VertexScale (1 DOF metric scale — optimized)
```

The `VertexGDir` stores a rotation `Rwg` and uses only 2 DOF (pitch and roll):
```cpp
class GDirection {
    Eigen::Matrix3d Rwg;
    void Update(const double *pu) {
        Rwg = Rwg * ExpSO3(pu[0], pu[1], 0.0);
        // Only 2 DOF: yaw around gravity is unobservable from accelerometer
    }
};
```

The `VertexScale` uses exponential parameterization to enforce positivity:
```cpp
class VertexScale : public g2o::BaseVertex<1, double> {
    void oplusImpl(const double *update_) {
        setEstimate(estimate() * exp(*update_));  // s_new = s * exp(δ) > 0
    }
};
```

In `EdgeInertialGS`, the gravity vector and position terms become:
```
g = scale * Rwg * gI            (gravity in world frame, scaled)
p = scale * p_visual            (positions scaled by metric scale factor)
```

### 6.3 Convergence Characteristics

| Time After Init Start | Scale Error | Notes |
|----------------------|-------------|-------|
| 2 seconds | ~5% | Usable for tracking |
| 5 seconds | ~2% | Good accuracy |
| 15 seconds | ~1% | Near-converged |
| After first loop closure | <0.5% | Scale corrected globally |

### 6.4 Re-Initialization

If IMU initialization fails (insufficient motion, poor visual tracking), the system can re-attempt initialization when conditions improve. In mono-inertial mode, sufficient **translational and rotational motion** during the first 2 seconds is critical.

---

## 7. How IMU Guides Tracking

### 7.1 Pose Prediction (Replacing Constant Velocity Model)

In pure visual SLAM, a constant velocity model predicts the next frame's pose:
```
T_predicted = T_current · T_velocity    (visual-only)
```

In visual-inertial mode, **IMU preintegration provides a much better prediction**:

```cpp
// Tracking::PredictStateIMU()
void Tracking::PredictStateIMU()
{
    // Get last frame state
    Rwb_last = mLastFrame.GetImuRotation();
    twb_last = mLastFrame.GetImuPosition();
    Vwb_last = mLastFrame.GetVelocity();

    // Get preintegrated measurement since last frame
    Preintegrated* pIMU = mCurrentFrame.mpImuPreintegratedFrame;
    Bias bias = mLastFrame.mImuBias;

    // Predict current state using IMU preintegration + gravity
    //   R_j = R_i · ΔR_ij
    //   v_j = v_i + g·Δt + R_i · Δv_ij
    //   p_j = p_i + v_i·Δt + ½·g·Δt² + R_i · Δp_ij

    Rwb_predicted = Rwb_last * pIMU->GetUpdatedDeltaRotation();
    twb_predicted = twb_last
                    + Vwb_last * pIMU->dT
                    + 0.5f * gravity * pIMU->dT * pIMU->dT
                    + Rwb_last * pIMU->GetUpdatedDeltaPosition();
    Vwb_predicted = Vwb_last
                    + gravity * pIMU->dT
                    + Rwb_last * pIMU->GetUpdatedDeltaVelocity();

    // Set predicted pose on current frame
    mCurrentFrame.SetImuPoseVelocity(Rwb_predicted, twb_predicted, Vwb_predicted);
}
```

`PredictStateIMU` has **two modes** based on whether the local mapper has updated the map:

```cpp
// Mode 1: Predict from last KEYFRAME (when map was updated by local mapper)
Rwb2 = NormalizeRotation(Rwb_KF * pIMU_KF->GetUpdatedDeltaRotation());
twb2 = twb_KF + Vwb_KF*t12 + 0.5f*t12*t12*Gz + Rwb_KF*pIMU_KF->GetUpdatedDeltaPosition();
Vwb2 = Vwb_KF + t12*Gz + Rwb_KF*pIMU_KF->GetUpdatedDeltaVelocity();

// Mode 2: Predict from last FRAME (when map was not updated)
// Same equations but using mLastFrame state and frame-to-frame preintegration
```

This prediction is **significantly better** than the constant velocity model because:
- It correctly accounts for **gravity** (critical for vertical motion)
- It captures **high-frequency dynamics** that occur between camera frames
- It works even during **fast rotations** where feature matching may fail
- It provides a better initial guess for visual matching and optimization

### 7.2 Visual-Inertial Pose Optimization

After feature matching, the tracking thread runs a **combined optimization** minimizing both visual reprojection error and IMU residuals:

```
E_tracking = Σ_k ρ(||u_k - π(T_cw · X_k)||²_Σ_proj)     (visual term)
           + ||r_IMU(x_{last}, x_{cur})||²_Σ_IMU           (inertial term)
           + ||b_cur - b_{last}||²_Σ_bias                   (bias prior term)
```

Where:
- **Visual term**: Reprojection error of observed map points with Huber robust kernel `ρ`
- **Inertial term**: IMU preintegration residual (9 DOF: rotation + velocity + position)
- **Bias prior term**: Regularizes bias to not change too rapidly

### 7.3 g2o Graph for Tracking Optimization

```
[VertexPose (last KF)] ──fixed──
        │
   [EdgeInertial] ← IMU preintegration constraint
        │
[VertexPose (current)] ──to optimize──
[VertexVelocity (current)] ──to optimize──
[VertexGyroBias] ──to optimize──
[VertexAccBias] ──to optimize──
        │
   [EdgeMono/EdgeStereo] ← reprojection errors
        │
[VertexSBAPointXYZ] ──fixed (map points)──
```

The optimization variables are the **current frame's pose, velocity, and biases**. The last keyframe's state is fixed as a reference.

### 7.4 Tracking State Machine with IMU

```
                    ┌──────────────────┐
                    │  NO_IMAGES_YET   │
                    └────────┬─────────┘
                             │ First image + IMU
                    ┌────────v─────────┐
                    │  NOT_INITIALIZED │ ← Visual initialization (2s)
                    └────────┬─────────┘
                             │ Visual map ready
                    ┌────────v─────────┐
                    │ RECENTLY_LOST    │ ← IMU-only prediction (short term)
                    └────────┬─────────┘
                             │ Inertial init done
                    ┌────────v─────────┐
                    │      OK          │ ← Normal VI tracking
                    └────────┬─────────┘
                             │ Tracking failure
                    ┌────────v─────────┐
              ┌─────│      LOST        │
              │     └──────────────────┘
              │
              v
    [Create new map in Atlas]  ← IMU allows short-term
                                 dead-reckoning before declaring lost
```

When tracking is "recently lost," the IMU can provide **dead-reckoning** for a short period (a few seconds), giving the system more time to recover via relocalization before fully declaring tracking as lost.

---

## 8. How IMU Guides Local Mapping

### 8.1 Visual-Inertial Local Bundle Adjustment (`LocalInertialBA`)

This is the most important optimization in the back-end. It jointly optimizes:

**Optimization variables:**
- Recent keyframe poses `{T_i}` (6 DOF each)
- Recent keyframe velocities `{v_i}` (3 DOF each)
- IMU biases `{b_g, b_a}` (6 DOF, shared or per-keyframe)
- Map points observed by these keyframes `{X_j}` (3 DOF each)

**Fixed variables:**
- Older keyframes outside the local window (provide constraints but not optimized)
- Map points observed only by fixed keyframes

### 8.2 Cost Function

```
E_LBA = Σ_{(k,i)} ρ(||r_proj(X_k, T_i)||²_{Σ_proj})     (visual: reprojection errors)
      + Σ_{i} ||r_IMU(x_i, x_{i+1})||²_{Σ_IMU}           (inertial: preintegration)
      + Σ_{i} ||b_g^{i+1} - b_g^{i}||²_{Σ_bg}            (gyro bias random walk)
      + Σ_{i} ||b_a^{i+1} - b_a^{i}||²_{Σ_ba}            (accel bias random walk)
```

### 8.3 g2o Graph for Local Inertial BA

```
    Fixed KF        Local KFs (optimized)           Map Points
    ─────────       ────────────────────           ──────────

    [Pose_0]─fixed  [Pose_1]  [Pose_2]  [Pose_3]   [X_1] [X_2] [X_3] ...
    [Vel_0] ─fixed  [Vel_1]   [Vel_2]   [Vel_3]
    [Bg_0]  ─fixed  [Bg_1]    [Bg_2]    [Bg_3]
    [Ba_0]  ─fixed  [Ba_1]    [Ba_2]    [Ba_3]

    Edges:
    ├── EdgeInertial:   Pose_0→Pose_1, Pose_1→Pose_2, Pose_2→Pose_3
    │   (connects consecutive pose+vel+bias vertices via preintegration)
    ├── EdgeGyroRW:     Bg_0→Bg_1, Bg_1→Bg_2, Bg_2→Bg_3
    │   (penalizes large gyro bias changes between keyframes)
    ├── EdgeAccRW:      Ba_0→Ba_1, Ba_1→Ba_2, Ba_2→Ba_3
    │   (penalizes large accel bias changes between keyframes)
    └── EdgeMono/Stereo: Pose_i → X_j (reprojection errors)
```

### 8.4 Key Differences from Pure Visual BA

| Aspect | Visual-Only BA | Visual-Inertial BA |
|--------|---------------|-------------------|
| Variables per KF | 6 (pose) | 15 (pose + vel + bias) |
| Inter-KF constraints | None (only via shared points) | IMU preintegration factors |
| Observability | 7 DOF gauge freedom | 4 DOF (yaw + 3D position) |
| Scale | Unobservable (mono) | Observable via IMU |
| Gravity | Not modeled | Constrained via accelerometer |

### 8.5 Effect on New Point Triangulation

IMU-guided poses are more accurate, which means:
- Triangulation baselines are better estimated
- Depth estimates for new map points are more reliable
- The system can triangulate points even with short baselines (where pure vision would fail)

---

## 9. How IMU Guides Loop Closing and Map Merging

### 9.1 Loop Closing with IMU

When a loop is detected between the current keyframe and a past keyframe:

1. **Sim(3) Computation**: In mono-inertial mode, the Sim(3) alignment includes scale. In stereo-inertial mode, scale is 1 (already metric).

2. **Pose Graph Optimization (PGO)**: The essential graph is optimized with Sim(3)/SE(3) constraints:
   - Loop closure edge (from place recognition + geometric verification)
   - Covisibility graph edges (strong connections between keyframes)
   - **IMU does not directly participate in PGO** — the essential graph uses only geometric constraints

3. **Post-PGO BA**: After PGO corrects the trajectory, a full GBA is launched that includes IMU factors.

### 9.2 Map Merging with IMU

When the Atlas detects that two sub-maps overlap (via place recognition):

1. **Alignment**: The relative SE(3) / Sim(3) transformation between maps is computed
2. **Merging**: Map points and keyframes from the smaller map are transformed into the larger map's coordinate frame
3. **Welding BA**: A local visual-inertial BA is run around the merge point to smooth the transition:
   - This includes IMU preintegration factors between the now-connected keyframes
   - Velocities and biases are re-estimated for consistency

### 9.3 Scale Correction

In mono-inertial mode, scale drift can accumulate. Loop closure provides an opportunity to correct scale:
- The Sim(3) alignment between loop keyframes estimates a scale correction
- This correction is propagated to all affected keyframes and map points
- After PGO, the GBA with IMU factors further refines the global scale

---

## 10. g2o Custom Types for IMU

### 10.1 Custom Vertex Types

```cpp
// Pose vertex (SE(3) + extrinsic)
class VertexPose : public g2o::BaseVertex<6, ImuCamPose>
{
    // Stores: Rwb, twb (body-to-world) + Rcb, tcb (camera-to-body)
    // Update: uses exponential map on se(3)
};

// Velocity vertex (R^3)
class VertexVelocity : public g2o::BaseVertex<3, Eigen::Vector3d>
{
    // Stores: velocity in world frame
};

// Gyroscope bias vertex (R^3)
class VertexGyroBias : public g2o::BaseVertex<3, Eigen::Vector3d>
{
    // Stores: b_g = [bwx, bwy, bwz]
};

// Accelerometer bias vertex (R^3)
class VertexAccBias : public g2o::BaseVertex<3, Eigen::Vector3d>
{
    // Stores: b_a = [bax, bay, baz]
};
```

### 10.2 IMU Edge Types

```cpp
// Main IMU preintegration edge (9-dimensional residual)
// Connects: VertexPose_i, VertexVelocity_i, VertexGyroBias_i, VertexAccBias_i,
//           VertexPose_j, VertexVelocity_j
class EdgeInertial : public g2o::BaseMultiEdge<9, Vector9d>
{
    // Residual: r = [r_ΔR, r_Δv, r_Δp]  (each 3-dimensional)
    // Information: from Preintegrated::Info (inverse of covariance C)

    void computeError() {
        // Get states from vertices
        R_i, p_i = VertexPose_i->estimate();
        v_i = VertexVelocity_i->estimate();
        R_j, p_j = VertexPose_j->estimate();
        v_j = VertexVelocity_j->estimate();
        bg = VertexGyroBias_i->estimate();
        ba = VertexAccBias_i->estimate();

        // Get bias-corrected preintegrated values
        dR = pInt->GetDeltaRotation(bg, ba);
        dV = pInt->GetDeltaVelocity(bg, ba);
        dP = pInt->GetDeltaPosition(bg, ba);

        // Rotation residual (3 DOF, in Lie algebra)
        eR = LogSO3(dR.t() * R_i.t() * R_j);

        // Velocity residual (3 DOF)
        //   Δv_ij should equal R_i^T · (v_j - v_i - g·Δt)
        eV = R_i.t() * (v_j - v_i - g * dt) - dV;

        // Position residual (3 DOF)
        //   Δp_ij should equal R_i^T · (p_j - p_i - v_i·Δt - ½g·Δt²)
        eP = R_i.t() * (p_j - p_i - v_i * dt - 0.5 * g * dt * dt) - dP;

        _error << eR, eV, eP;  // 9×1 vector
    }

    void linearizeOplus() {
        // Analytical Jacobians of the 9D residual w.r.t. each vertex
        // 6 vertices → _jacobianOplus[0..5]
        // Each is a 9×(vertex_dim) matrix

        // _jacobianOplus[0]: ∂r/∂(Pose_i)    — 9×6
        // _jacobianOplus[1]: ∂r/∂(Vel_i)     — 9×3
        // _jacobianOplus[2]: ∂r/∂(GyroBias)  — 9×3
        // _jacobianOplus[3]: ∂r/∂(AccBias)   — 9×3
        // _jacobianOplus[4]: ∂r/∂(Pose_j)    — 9×6
        // _jacobianOplus[5]: ∂r/∂(Vel_j)     — 9×3

        // Key entries:
        // Position residual w.r.t. position_i: -I (3×3)
        // Position residual w.r.t. position_j: R_i^T · R_j (3×3)
        // Velocity residual w.r.t. velocity_i: -R_i^T (3×3)
        // Velocity residual w.r.t. velocity_j: R_i^T (3×3)
        // ... (full derivation in paper Appendix)
    }
};

// Gyroscope bias random walk edge (3-dimensional residual)
class EdgeGyroRW : public g2o::BaseBinaryEdge<3, Vector3d, VertexGyroBias, VertexGyroBias>
{
    void computeError() {
        // r_bg = b_g^{i+1} - b_g^{i}
        _error = v2->estimate() - v1->estimate();
    }
    // Information: Σ_bg^{-1} (from random walk noise * Δt)
};

// Accelerometer bias random walk edge (3-dimensional residual)
class EdgeAccRW : public g2o::BaseBinaryEdge<3, Vector3d, VertexAccBias, VertexAccBias>
{
    void computeError() {
        // r_ba = b_a^{i+1} - b_a^{i}
        _error = v2->estimate() - v1->estimate();
    }
    // Information: Σ_ba^{-1} (from random walk noise * Δt)
};
```

### 10.3 Information Matrix

The information matrix for `EdgeInertial` comes directly from the preintegration covariance:

```cpp
// When constructing the edge:
EdgeInertial* e = new EdgeInertial(pInt);  // pInt is the Preintegrated object

// Inside the constructor:
// The 9×9 information matrix is extracted from pInt->C (15×15 covariance)
// Only the first 9×9 block (rotation, velocity, position) is used for the edge
// Info = C[0:9, 0:9]^{-1}
setInformation(pInt->C.rowRange(0,9).colRange(0,9).inv());
```

The remaining 6×6 block (for biases) feeds into the `EdgeGyroRW` and `EdgeAccRW` edges.

---

## 11. The Complete IMU Data Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                        DATA ACQUISITION                              │
│                                                                      │
│  IMU sensor (200Hz)          Camera sensor (30Hz)                    │
│  ω_k, a_k, t_imu_k          Image_j, t_cam_j                       │
│       │                           │                                  │
│       └──── timestamps must be aligned ────┘                         │
│                           │                                          │
└───────────────────────────┼──────────────────────────────────────────┘
                            │
                            v
┌─────────────────────────────────────────────────────────────────────┐
│                     SYSTEM.CC (Entry Point)                          │
│                                                                      │
│  GrabImageMonocular(image, timestamp, imuMeasurements)               │
│  ├── Buffer IMU measurements: mvImuMeasurements                      │
│  ├── Create Frame with IMU calibration                               │
│  └── Call Tracking::Track()                                          │
└───────────────────────────┼──────────────────────────────────────────┘
                            │
                            v
┌─────────────────────────────────────────────────────────────────────┐
│                     TRACKING THREAD                                  │
│                                                                      │
│  1. PreintegrateIMU()                                                │
│     ├── Create Preintegrated object (from last frame/KF bias)        │
│     ├── For each IMU measurement between frames:                     │
│     │   └── pIMU->IntegrateNewMeasurement(acc, gyro, dt)             │
│     └── Two preintegrations maintained:                              │
│         ├── mpImuPreintegratedFromLastFrame (frame-to-frame)         │
│         └── mpImuPreintegratedFromLastKF (KF-to-current)             │
│                                                                      │
│  2. PredictStateIMU()                                                │
│     ├── Use preintegrated measurement to predict:                    │
│     │   R_cur = R_last · ΔR                                         │
│     │   v_cur = v_last + g·Δt + R_last · Δv                         │
│     │   p_cur = p_last + v_last·Δt + ½g·Δt² + R_last · Δp          │
│     └── Set predicted pose on current frame                          │
│                                                                      │
│  3. TrackWithMotionModel() / TrackReferenceKeyFrame()                │
│     ├── Use IMU prediction as initial guess                          │
│     ├── Match features against local map                             │
│     └── Run visual-inertial pose optimization:                       │
│         ├── Visual: reprojection errors → EdgeMono/EdgeStereo        │
│         ├── Inertial: preintegration → EdgeInertial                  │
│         └── Bias prior: → EdgeGyroRW, EdgeAccRW                     │
│                                                                      │
│  4. TrackLocalMap()                                                  │
│     ├── Search more map point matches in local map                   │
│     └── Refine pose with all matches + IMU                           │
│                                                                      │
│  5. NeedNewKeyFrame() + CreateNewKeyFrame()                          │
│     ├── Keyframe carries: pose, velocity, bias, preintegration       │
│     └── Pass keyframe + preintegration to Local Mapping              │
└───────────────────────────┼──────────────────────────────────────────┘
                            │
                            v
┌─────────────────────────────────────────────────────────────────────┐
│                    LOCAL MAPPING THREAD                               │
│                                                                      │
│  ProcessNewKeyFrame()                                                │
│  ├── Insert keyframe into map                                        │
│  └── Update covisibility graph                                       │
│                                                                      │
│  CreateNewMapPoints()                                                │
│  ├── Triangulate new points from matched features                    │
│  └── IMU-improved poses → better triangulation baselines             │
│                                                                      │
│  LocalInertialBA()  ← KEY FUNCTION                                   │
│  ├── Build g2o graph:                                                │
│  │   ├── VertexPose for each KF in window                            │
│  │   ├── VertexVelocity for each KF                                  │
│  │   ├── VertexGyroBias, VertexAccBias for each KF                   │
│  │   ├── VertexSBAPointXYZ for map points                            │
│  │   ├── EdgeInertial between consecutive KFs                        │
│  │   ├── EdgeGyroRW, EdgeAccRW between consecutive biases            │
│  │   └── EdgeMono/EdgeStereo for reprojection errors                 │
│  ├── Optimize (Levenberg-Marquardt, ~10 iterations)                  │
│  ├── Update keyframe poses, velocities, biases                       │
│  ├── Update map point positions                                      │
│  └── Propagate updated biases to tracking thread                     │
│                                                                      │
│  KeyFrameCulling()                                                   │
│  └── Remove redundant keyframes (same as visual SLAM)                │
└───────────────────────────┼──────────────────────────────────────────┘
                            │
                            v
┌─────────────────────────────────────────────────────────────────────┐
│               LOOP CLOSING & MAP MERGING THREAD                      │
│                                                                      │
│  DetectLoop() / DetectMapMerge()                                     │
│  ├── DBoW2 place recognition (same as visual SLAM)                   │
│  └── Geometric verification (Sim3/SE3 alignment)                     │
│                                                                      │
│  CorrectLoop()                                                       │
│  ├── Compute Sim(3) correction (includes scale in mono-inertial)     │
│  ├── Pose Graph Optimization on essential graph                      │
│  └── Launch Global BA with IMU factors                               │
│                                                                      │
│  MergeLocal()                                                        │
│  ├── Transform merged map into active map's frame                    │
│  ├── Welding BA around merge point (includes IMU factors)            │
│  └── Update Atlas                                                    │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 12. Key Equations Summary

### IMU Preintegration (between keyframes i and j)

| Quantity | Equation |
|----------|----------|
| Delta rotation | `ΔR_ij = ∏_{k=i}^{j-1} Exp((ω_k - b_g) · δt)` |
| Delta velocity | `Δv_ij = Σ_{k=i}^{j-1} ΔR_ik · (a_k - b_a) · δt` |
| Delta position | `Δp_ij = Σ_{k=i}^{j-1} [Δv_ik · δt + ½ · ΔR_ik · (a_k - b_a) · δt²]` |

### IMU Residual (for optimization)

| Component | Residual |
|-----------|----------|
| Rotation | `r_R = Log(ΔR_ij^T · R_i^T · R_j)` |
| Velocity | `r_v = R_i^T · (v_j - v_i - g·Δt) - Δv_ij` |
| Position | `r_p = R_i^T · (p_j - p_i - v_i·Δt - ½g·Δt²) - Δp_ij` |

### Bias Correction (first-order)

| Quantity | Correction |
|----------|-----------|
| Rotation | `ΔR(b̄+δb) ≈ ΔR(b̄) · Exp(J_R^g · δb_g)` |
| Velocity | `Δv(b̄+δb) ≈ Δv(b̄) + J_v^g · δb_g + J_v^a · δb_a` |
| Position | `Δp(b̄+δb) ≈ Δp(b̄) + J_p^g · δb_g + J_p^a · δb_a` |

### State Prediction from IMU

| Quantity | Prediction |
|----------|-----------|
| Rotation | `R_j = R_i · ΔR_ij` |
| Velocity | `v_j = v_i + g·Δt + R_i · Δv_ij` |
| Position | `p_j = p_i + v_i·Δt + ½g·Δt² + R_i · Δp_ij` |

---

## 13. Design Insights and Architecture Notes

1. **Thread safety**: The `Preintegrated` class uses `std::mutex` on all getters because the tracking thread reads preintegrated values while the local mapping thread may update biases.

2. **Dual preintegration**: Maintaining both keyframe-to-current and frame-to-frame preintegration allows the system to predict from the most recent optimized state (keyframe) when available, or from the last frame when the optimizer hasn't run yet.

3. **Progressive initialization**: The three-phase initialization with decreasing prior strength (`priorG`: 1e2 → 1.0 → 0.0; `priorA`: 1e10 → 1e5 → 0.0) is crucial for robustness. Initially, biases are strongly constrained (essentially zero), then gradually freed as more data accumulates and the system becomes observable.

4. **Body frame convention**: All preintegration is done in the body (IMU) frame. The `Calib::mTbc` and `Calib::mTcb` transforms convert between camera and body frames. The `EdgeInertial` error is computed in the body frame of keyframe `i` to avoid the influence of unobservable states (yaw + absolute position).

5. **SVD normalization**: `NormalizeRotation()` uses SVD to project potentially drifted rotation matrices back onto SO(3), preventing numerical drift during long integration sequences.

6. **Scale observability**: In monocular mode, the `VertexScale` with exponential parameterization (`s * exp(δ)`) is essential because metric scale is unobservable from vision alone but becomes observable with IMU integration. Once determined, the map is rescaled and subsequent optimization works in metric coordinates.

7. **Stored raw measurements**: The `Preintegrated` class stores all raw IMU samples in `mvMeasurements`. This enables `Reintegrate()` when biases change significantly (beyond the first-order correction range), and `MergePrevious()` when keyframes are culled and preintegration windows must be combined.

8. **Update order in integration**: The deliberate order (position → velocity → rotation) in `IntegrateNewMeasurement` is essential because each depends on the previous value of the others. Computing position before updating velocity/rotation ensures consistency with the discrete-time integration scheme.

---

## 14. References

- Campos, C., Elvira, R., Rodriguez, J.J.G., Montiel, J.M.M., Tardos, J.D. "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM." IEEE T-RO, 2021. [arXiv:2007.11898](https://arxiv.org/abs/2007.11898)
- Forster, C., Carlone, L., Dellaert, F., Scaramuzza, D. "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry." IEEE T-RO, 2017. [arXiv:1512.02363](https://arxiv.org/abs/1512.02363)
- ORB-SLAM3 Source Code: [GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- ORB-SLAM3 IMU Preintegration Code Review (Part 1): [Dongwon Shin](https://dongwonshin.oopy.io/245cc7c3-f3fb-8045-98b1-ceec8ea8a82d)
- ORB-SLAM3 IMU Preintegration Code Review (Part 2): [Dongwon Shin](https://dongwonshin.oopy.io/245cc7c3-f3fb-80d2-b59f-df504ab861e2)
- ORB-SLAM3 Detailed Comments Fork: [electech6/ORB_SLAM3_detailed_comments](https://github.com/electech6/ORB_SLAM3_detailed_comments)
- Campos, C., Montiel, J.M.M., Tardos, J.D. "Inertial-Only Optimization for Visual-Inertial Initialization." ICRA 2020.
