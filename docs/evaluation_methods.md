# Evaluation Methods for Visual SLAM

This document describes three complementary approaches for evaluating Visual SLAM trajectory quality: **(1)** standard ground-truth-based metrics, **(2)** ground-truth-free evaluation via sensitivity analysis (GTF-ATE), and **(3)** multi-run statistical analysis. Each operates at a different level of the evaluation hierarchy and captures different aspects of system quality.

---

## 1. Ground-Truth-Based Evaluation (SfM / Benchmark GT)

### What It Is

The standard approach: compare estimated poses against a known reference trajectory. The reference comes from motion capture (TUM, EuRoC), RTK-GPS/INS (KITTI, 4Seasons), or a high-quality SfM reconstruction (COLMAP).

### Metrics

#### 1a. Absolute Trajectory Error (ATE / APE)

The primary metric used by pyslam (via the `evo` library).

**Procedure:**
1. Align the estimated trajectory to the ground truth using Umeyama alignment (SE(3) for stereo/RGB-D, Sim(3) for monocular to recover scale).
2. For each corresponding pose pair, compute the translational distance between the aligned estimate and the ground truth.
3. Report statistics: **RMSE** (primary), mean, median, std, min, max.

**What it captures:** Global accuracy. A single number that answers "how far off is the estimated trajectory from reality, on average?"

**Limitations:**
- Dominated by large errors (a single bad loop closure can inflate RMSE disproportionately).
- The alignment step can mask systematic drift by finding the best rigid-body fit.
- Sensitive to the number of evaluated poses: a system that tracks 50% of frames but tracks them well may score better than one that tracks 100% with moderate drift.

**pyslam implementation:** `pyslam/utilities/evaluation.py:56-127` -- the `evaluate_evo()` function calls `evo.core.metrics.APE` on the translation part after Sim(3)/SE(3) alignment.

#### 1b. Relative Pose Error (RPE)

Measures local consistency over fixed intervals.

**Procedure:**
1. For all pose pairs separated by a fixed delta (time or distance), compute the relative transformation error.
2. Decompose into translational and rotational components.
3. Report RMSE over all pairs.

**What it captures:** Local drift rate. Independent of global alignment -- answers "how much does the system drift per unit of travel?"

**When to prefer RPE over ATE:**
- When comparing systems with different loop closure strategies (RPE measures open-loop drift).
- When the trajectory is very long and global alignment becomes ambiguous.
- When you care about local smoothness (e.g., for real-time control).

**pyslam status:** Not currently computed. Would require adding `evo.core.metrics.RPE` alongside the existing APE call.

#### 1c. KITTI Sub-Segment Drift

Used by the KITTI and 4Seasons benchmarks.

**Procedure:**
1. Extract all sub-segments of lengths 100m, 200m, ..., 800m from the trajectory.
2. For each sub-segment, compute the relative translational error (%) and rotational error (deg/m).
3. Average across all sub-segments of the same length.

**What it captures:** Drift as a function of distance traveled. More nuanced than a single ATE number -- reveals whether drift is constant or accelerating.

**pyslam status:** Reported in the pyslam paper (arXiv:2502.11955) as `trel` (%) and `rrel` (deg/100m), but the implementation is not exposed in the public evaluation pipeline.

### Effect on Evaluation

Ground-truth metrics are the **gold standard** for accuracy assessment. They give definitive answers about geometric quality. However, they have three fundamental constraints:

1. **Availability**: Ground truth requires expensive equipment (motion capture, RTK-GPS) or careful offline processing (COLMAP). Many real-world deployment environments have no ground truth available at all.

2. **Alignment ambiguity**: Monocular systems require scale recovery during alignment. If alignment fails or is poorly conditioned (short trajectories, degenerate motion), the resulting ATE is unreliable.

3. **Post-hoc only**: GT metrics cannot be computed during SLAM operation. They tell you how well you *did*, not how well you *are doing*.

### References

- Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems," IROS 2012 -- defined ATE and RPE.
- Geiger et al., "Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite," CVPR 2012 -- defined sub-segment drift.
- Kummerle et al., "On Measuring the Accuracy of SLAM Algorithms," Autonomous Robots, 2009 -- formal treatment of relative-relation metrics.
- Grupp, "evo: Python package for the evaluation of odometry and SLAM," 2017 -- [github.com/MichaelGrupp/evo](https://github.com/MichaelGrupp/evo).

---

## 2. Ground-Truth-Free Evaluation via Sensitivity Analysis (GTF-ATE)

### What It Is

A method to estimate trajectory quality **without any ground truth** by measuring how sensitive the SLAM output is to small perturbations of the input images. Proposed by Fontan et al. (December 2024).

The core insight: a SLAM system operating in a regime where it has abundant features, good parallax, and stable tracking will produce nearly identical trajectories whether or not the input images have a small amount of added noise. Conversely, a system near failure (few features, poor illumination, fast motion) will produce wildly different trajectories under the same perturbation -- because it was already operating at the edge of its capabilities.

### The Method

#### Step 1: Run SLAM on Original Images

Run the SLAM pipeline on the unmodified image sequence. Produce trajectory `T_orig`.

Because most SLAM systems are non-deterministic (due to threading, RANSAC randomness, etc.), run `k` times on the original images to obtain `{T_orig_1, ..., T_orig_k}`.

#### Step 2: Run SLAM on Noise-Augmented Images

For a noise level `Δσ`, add per-pixel additive Gaussian noise to every frame:

```
I_noisy(x, y) = I_original(x, y) + N(0, Δσ²)
```

where `N(0, Δσ²)` is a sample from a zero-mean Gaussian with standard deviation `Δσ`, drawn independently for each pixel and each frame. The noisy pixel values are clamped to the valid range [0, 255].

Run the SLAM pipeline `k_Δ` times on the noisy images to obtain `{T_noisy_1, ..., T_noisy_k_Δ}`.

#### Step 3: Compute GTF-ATE

Align all trajectories (original and noisy) to a common reference (e.g., `T_orig_1`) using Sim(3) alignment (same Umeyama method used for standard ATE). Then compute the spread:

```
GTF-ATE(Δσ) = ATE(T_orig, T_noisy)
```

In practice, this is the average pairwise ATE between the original-run trajectories and the noise-augmented trajectories. The paper shows that this quantity correlates strongly with the true ATE (measured against ground truth), with the correlation strength depending on the choice of `Δσ`.

#### Step 4: Sweep Noise Levels (Optional)

Compute GTF-ATE for multiple `Δσ` values to build a **sensitivity curve**. The shape of this curve is informative:
- **Flat curve** (small GTF-ATE even at high noise): the system is robust and likely accurate.
- **Steep curve** (GTF-ATE rises rapidly with `Δσ`): the system is fragile and likely inaccurate.

### Parameters

| Parameter | Meaning | Recommended |
|-----------|---------|-------------|
| `Δσ` | Standard deviation of per-pixel Gaussian noise (in intensity units, 0-255 scale) | Sweep multiple values; the paper tests a range and selects the `Δσ` with highest R² correlation to true ATE |
| `k` | Number of runs on original images | Captures baseline non-determinism; `k ≥ 3` |
| `k_Δ` | Number of runs on noisy images | Higher = more stable GTF-ATE estimate. Paper tests `k_Δ = 6` and `k_Δ = 60`; `k_Δ = 6` is a practical minimum |

### Effect on Evaluation

GTF-ATE addresses the biggest limitation of standard metrics: **it requires no ground truth**. This enables:

1. **Evaluation in the wild**: Assess SLAM quality on arbitrary real-world sequences where no reference trajectory exists. This is the typical deployment scenario.

2. **Hyperparameter tuning without GT**: The paper demonstrates using GTF-ATE to select optimal SLAM hyperparameters (e.g., number of features, RANSAC thresholds) purely from input data. The selected parameters match those that minimize true ATE.

3. **System comparison without GT**: Compare two SLAM systems (e.g., ORB-SLAM3 vs. a learning-based pipeline) on the same sequence without needing to know the true trajectory.

**Tradeoffs:**

- **Computational cost**: Requires `k + k_Δ` runs per sequence per noise level. At `k=3, k_Δ=6`, that is 9 runs per sequence -- roughly 9x the cost of a single evaluation. At `k_Δ=60`, it is 63x.
- **Noise level sensitivity**: The correlation between GTF-ATE and true ATE depends on choosing an appropriate `Δσ`. Too small and the perturbation has no effect; too large and it destroys all features indiscriminately. The optimal `Δσ` may vary across datasets and systems.
- **Proxy, not ground truth**: GTF-ATE is a **correlation-based proxy**. It tells you which configuration is *relatively* better, but does not give you absolute metric accuracy in meters. Two systems can have the same GTF-ATE but different true ATEs if they fail in different ways.
- **Assumes feature-based pipeline**: The noise augmentation strategy is designed around the observation that pixel noise degrades feature detection/matching. For direct methods (LSD-SLAM, DSO) or learning-based dense methods (DUSt3R, VGGT), the sensitivity profile may differ, and the correlation with true ATE may be weaker.

### Practical Implementation Sketch

```python
import numpy as np

def add_gaussian_noise(image, delta_sigma):
    """Add per-pixel Gaussian noise to an image."""
    noise = np.random.normal(0, delta_sigma, image.shape).astype(np.float32)
    noisy = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return noisy

def compute_gtf_ate(slam_runner, image_sequence, k=3, k_delta=6, delta_sigmas=[5, 10, 15, 20]):
    """
    Compute GTF-ATE for a SLAM system on an image sequence.

    Args:
        slam_runner: callable that takes an image sequence and returns a trajectory (Nx4x4 poses)
        image_sequence: list of images (HxWx3 uint8 arrays)
        k: number of runs on original images
        k_delta: number of runs per noise level
        delta_sigmas: list of noise standard deviations to test

    Returns:
        dict mapping delta_sigma -> GTF-ATE value
    """
    # Step 1: Run on original images k times
    original_trajectories = []
    for _ in range(k):
        traj = slam_runner(image_sequence)
        original_trajectories.append(traj)

    # Use first original trajectory as reference for alignment
    T_ref = original_trajectories[0]

    results = {}
    for delta_sigma in delta_sigmas:
        # Step 2: Create noisy image sequence
        noisy_sequence = [add_gaussian_noise(img, delta_sigma) for img in image_sequence]

        # Step 3: Run on noisy images k_delta times
        noisy_trajectories = []
        for _ in range(k_delta):
            traj = slam_runner(noisy_sequence)
            noisy_trajectories.append(traj)

        # Step 4: Compute average ATE between original and noisy trajectories
        ate_values = []
        for t_orig in original_trajectories:
            for t_noisy in noisy_trajectories:
                ate = compute_ate_aligned(t_orig, t_noisy)  # Sim(3)-aligned ATE
                ate_values.append(ate)

        results[delta_sigma] = np.mean(ate_values)

    return results
```

### References

- Fontan, Civera, Fischer, Milford, "Look Ma, No Ground Truth! Ground-Truth-Free Tuning of Structure from Motion and Visual SLAM," arXiv:2412.01116, December 2024.
- Fontan et al., VSLAM-LAB framework -- [github.com/VSLAM-LAB/VSLAM-LAB](https://github.com/VSLAM-LAB/VSLAM-LAB) (same research group, implements the evaluation infrastructure).

---

## 3. Multi-Run Statistical Analysis

### What It Is

Run the same SLAM system on the same sequence multiple times and analyze the **distribution** of results rather than a single point estimate. This captures the non-determinism inherent in most SLAM systems (threading races, RANSAC sampling, hash-map ordering, etc.) and provides a measure of **reliability** alongside accuracy.

### Why SLAM is Non-Deterministic

Most visual SLAM systems produce different trajectories on different runs of the same input, due to:

1. **RANSAC randomness**: Feature matching and pose estimation use random sampling. Different seeds produce different inlier sets and thus different poses.
2. **Thread scheduling**: Local mapping and loop closing run in parallel threads. The exact interleaving of operations (which keyframes get processed, when BA runs) varies between runs.
3. **Floating-point non-determinism**: Multi-threaded reductions and SIMD operations can produce slightly different numerical results due to operation reordering.
4. **Keyframe timing**: Whether a frame becomes a keyframe can depend on the current state of local mapping (is it busy?), which varies with thread scheduling.

### The Method

#### Step 1: Run N Times

Execute the SLAM pipeline N times on the identical input sequence with the same configuration. Collect:
- N estimated trajectories
- N sets of metrics (ATE, % frames lost, runtime, map size, etc.)

#### Step 2: Compute Distribution Statistics

For each metric, compute across the N runs:

| Statistic | What It Tells You |
|-----------|-------------------|
| **Mean** | Expected performance on a typical run |
| **Median** | Robust central tendency (less sensitive to outlier runs) |
| **Std Dev** | Spread / reliability -- how much does performance vary? |
| **Min / Max** | Best-case and worst-case bounds |
| **IQR** | Interquartile range -- robust spread measure |

#### Step 3: Visualize

**Boxplots** (as used by VSLAM-LAB) are the standard visualization:
- One box per system/configuration
- Box shows IQR (25th-75th percentile)
- Whiskers show full range or 1.5×IQR
- Individual points for outlier runs

**Cumulative ATE curves** show the distribution of per-pose errors:
- X-axis: ATE threshold
- Y-axis: fraction of poses with ATE below threshold
- Steeper curves = more consistently accurate

### How Many Runs?

| N | Use Case | Statistical Power |
|---|----------|-------------------|
| 3 | Quick sanity check | Can detect gross instability; too few for confidence intervals |
| 5 | Reasonable minimum | Median is meaningful; std dev is rough |
| 10 | Standard practice | Good estimates of mean, std, IQR; boxplots are informative |
| 30+ | Rigorous comparison | Enables hypothesis testing (Wilcoxon, Mann-Whitney U) with statistical significance |

The pyslam evaluation manager already supports `number_of_runs_per_dataset` and averages metrics across iterations (see `slam_evaluation_manager.py:433-530`). The infrastructure is there; only the visualization (boxplots) and formal statistical tests are missing.

### Effect on Evaluation

Multi-run analysis reveals a dimension of system quality that single-run metrics completely miss: **reliability**.

Consider two systems evaluated on the same sequence:

| System | Run 1 ATE | Run 2 ATE | Run 3 ATE | Run 4 ATE | Run 5 ATE | Mean | Std |
|--------|-----------|-----------|-----------|-----------|-----------|------|-----|
| A | 0.05 | 0.04 | 0.06 | 0.05 | 0.05 | **0.050** | **0.007** |
| B | 0.02 | 0.03 | 0.02 | 0.45 | 0.03 | **0.110** | **0.187** |

System B has a *much* better best-case (0.02 vs. 0.04), but its Run 4 is catastrophic (likely a false loop closure or tracking failure). Single-run reporting could show B as either the clear winner (if you happened to get Run 1) or a disaster (if you got Run 4).

**What multi-run analysis captures that single-run does not:**

1. **Robustness**: A system with low mean ATE but high std is unreliable. For deployment, you often prefer a slightly worse but consistent system over a brilliant-but-brittle one.

2. **Failure rate**: The fraction of runs where the system fails completely (tracking lost, divergence) is arguably more important than average ATE for practical deployment. pyslam already tracks this as `percent_lost`.

3. **Confidence in comparisons**: Without multi-run data, you cannot distinguish "System A is better than System B" from "System A got lucky on this particular run." With N=10+, you can compute confidence intervals and statistical tests.

4. **Non-determinism as a quality signal**: High run-to-run variance itself indicates architectural fragility -- the system's behavior depends too much on RANSAC draws or thread timing. This is independent of accuracy and is a valid quality metric on its own.

**Tradeoffs:**

- **Computational cost**: Linear in N. At N=10, you spend 10x the compute of a single run. For fast systems (ORB-SLAM3 at ~30ms/frame) this is manageable; for slow systems (dense neural SLAM at ~500ms/frame) it becomes expensive.
- **Storage**: N sets of trajectories, metrics, and plots per sequence. With multiple datasets and configurations, this grows quickly.
- **Diminishing returns**: Going from N=1 to N=5 is transformative. Going from N=10 to N=30 is incremental unless you need formal statistical significance.

### Combining Multi-Run with GTF-ATE

The two methods are naturally complementary:

- **Multi-run on original images** (Section 3) measures **intrinsic non-determinism** -- how much does the system vary due to its own randomness?
- **GTF-ATE** (Section 2) measures **sensitivity to input perturbation** -- how much does the system vary when the input is slightly degraded?

A system can be deterministic (low multi-run variance) but fragile (high GTF-ATE) if it consistently converges to the same wrong answer on difficult sequences. Conversely, a system can be non-deterministic (high multi-run variance) but robust (low GTF-ATE) if it explores different solutions but they are all roughly correct.

The most informative evaluation uses both:

```
              Low GTF-ATE          High GTF-ATE
           ┌─────────────────┬─────────────────────┐
Low Var    │ IDEAL: accurate  │ Consistently wrong   │
(multi-run)│ and reliable     │ (deterministic but   │
           │                  │ fragile to noise)    │
           ├─────────────────┼─────────────────────┤
High Var   │ Accurate on avg  │ WORST: unreliable    │
(multi-run)│ but unreliable   │ and inaccurate       │
           │ (needs more runs)│                      │
           └─────────────────┴─────────────────────┘
```

### References

- Fontan et al., "VSLAM-LAB: A Comprehensive Framework for Visual SLAM Methods and Datasets," arXiv:2504.04457, April 2025 -- uses boxplots over multiple runs; categorizes sequences by difficulty.
- Bodin et al., "SLAMBench2: Multi-Objective Head-to-Head Benchmarking for Visual SLAM," ICRA 2018 -- multi-run + multi-objective (accuracy, FPS, memory, energy).
- Sansoni & Tosetti, "The SLAM Confidence Trap," arXiv:2602.15884, February 2026 -- argues that benchmark optimization without uncertainty estimation leads to brittle systems; multi-run variance is one measure of this.

---

## Summary: Which Method to Use When

| Scenario | Method | Why |
|----------|--------|-----|
| Benchmark comparison with GT available | **ATE/RPE** (Section 1) | Gold standard; definitive accuracy numbers |
| Deploying to new environment, no GT | **GTF-ATE** (Section 2) | Only option for accuracy estimation without a reference |
| Hyperparameter tuning | **GTF-ATE** (Section 2) | Avoids overfitting to benchmark GT |
| Assessing deployment reliability | **Multi-run** (Section 3) | Reveals failure rates and worst-case behavior |
| Comparing two systems fairly | **All three** | GT metrics for accuracy, GTF-ATE for generalization, multi-run for reliability |

### Current pyslam Support

| Method | Status | Location |
|--------|--------|----------|
| ATE RMSE (via evo APE) | Implemented | `pyslam/utilities/evaluation.py:56-127` |
| ATE stats (mean, median, std, min, max) | Implemented | `pyslam/utilities/evaluation.py:78` |
| % Frames Lost | Implemented | Saved in `other_metrics_info.txt` |
| Multi-run averaging | Implemented | `slam_evaluation_manager.py:433-530` |
| Comparative tables (CSV/HTML/PDF) | Implemented | `slam_evaluation_manager.py:532-598` |
| RPE | Not implemented | Would add `evo.core.metrics.RPE` |
| KITTI sub-segment drift | Not implemented | Needs sub-segment extraction logic |
| GTF-ATE | Not implemented | Needs image noise injection + multi-run infrastructure |
| Multi-run boxplots | Not implemented | Needs matplotlib boxplot generation |
