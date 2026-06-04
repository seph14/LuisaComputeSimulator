# Contact Tuning Reference (P4.4)

Systematically documented stable solver parameters for each standing demo.

## Solver Parameters

| Parameter | Default | H1 | G1 | ANYmal (proxy) | Notes |
|-----------|---------|-----|-----|----------------|-------|
| `implicit_dt` | 1/300 | 1/300 | 1/300 | 1/300 | Stable at 300Hz |
| `num_substep` | 3 | 3 | 3 | 3 | Joint stability |
| `nonlinear_iter_count` | 5 | 100 | 100 | 5 | H1/G1 use Newton reference value |
| `pcg_iter_count` | 200 | 200 | 200 | 200 | |
| `use_floor` | True | True | True | True | Z-up ground plane |
| `use_self_collision` | False | False | False | False | Disabled (stability) |

## Joint Drive Parameters

| Parameter | H1 (Newton: 150/5) | G1 (Newton: 500/10) | ANYmal (Newton: 150/5) |
|-----------|---------------------|----------------------|------------------------|
| `kp` (target_ke) | 150 | 500 | 150 |
| `kd` (target_kd) | 5 | 10 | 5 |
| Max drift (30f) | 0.0000 rad | 0.0000 rad | 0.0013 rad |

## Contact Parameters (Staged — vs Newton)

| Parameter | H1 Newton | H1 Current | G1 Newton | G1 Current |
|-----------|-----------|------------|-----------|------------|
| `shape ke` | 2.0e3 | default (1e8) | 1.0e3 | default |
| `shape kd` | 1.0e2 | default (0) | 2.0e2 | default |
| `shape kf` | 1.0e3 | default | 1.0e3 | default |
| `shape mu` | 0.75 | 0.5 (default) | 0.75 | 0.5 (default) |

## Failure Modes Observed

| Mode | Symptoms | Mitigation |
|------|----------|------------|
| **Drift** | Joint positions slowly diverge from target | Increase kp/kd, decrease dt |
| **Foot penetration** | Feet sink below ground plane | Increase contact stiffness (future C++ work) |
| **Explosion** | Bodies fly apart in first few frames | Reduce initial overlap (increase floor_clearance), reduce dt |
| **Slow convergence** | Bodies oscillate for many frames before settling | Increase nonlinear_iter_count, reduce kd |

## Current Tuning Status

- **H1**: Stable at kp=150, kd=5, 100 nonlinear iters. Bodies hold position perfectly (drift=0.0000).
  - Gap: Contact stiffness not aligned with Newton (requires C++ material param override).
- **G1 (23dof)**: Stable at kp=500, kd=10, 100 nonlinear iters.
  - Gap: Same contact tuning gap as H1.
- **ANYmal (proxy)**: Stable at kp=150, kd=5. Base height 0.65m (vs Newton 0.68m).
  - Gap: Procedural geometry, not real ANYmal asset.

## Next Steps

1. Add contact stiffness/kd/kf/mu as configurable per-body parameters in C++ WorldData
2. Expose collision group/mask to Python for inter-world isolation
3. Implement mesh bounding_box approximation for H1/G1 (replaces full trimesh mesh)
4. Per-joint friction parameter (currently uses per-vertex material friction_mu)
