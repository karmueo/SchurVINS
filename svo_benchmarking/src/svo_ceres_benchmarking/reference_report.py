"""SE(3), no-scale TUM trajectory comparison against a non-independent VIO prior."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def associate(estimate, reference, tolerance=0.01):
    candidates = []
    for i, stamp in enumerate(estimate[:, 0]):
        j = np.searchsorted(reference[:, 0], stamp)
        for k in (j - 1, j):
            if 0 <= k < len(reference):
                error = abs(stamp - reference[k, 0])
                if error <= tolerance:
                    candidates.append((error, i, k))
    used_est, used_ref, pairs = set(), set(), []
    for _, i, j in sorted(candidates):
        if i not in used_est and j not in used_ref:
            used_est.add(i)
            used_ref.add(j)
            pairs.append((i, j))
    return np.asarray(sorted(pairs), dtype=int).reshape(-1, 2)


def report(estimate_path, reference_path, output, tolerance=0.01):
    if not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError('Time tolerance must be finite and positive')
    estimate = np.atleast_2d(np.loadtxt(estimate_path))
    reference = np.atleast_2d(np.loadtxt(reference_path))
    for name, values in [('estimate', estimate), ('reference', reference)]:
        if values.shape[1] != 8 or not np.isfinite(values).all():
            raise ValueError(f'{name} must contain finite TUM timestamp xyz xyzw rows')
        if np.any(np.linalg.norm(values[:, 4:8], axis=1) < 1e-12):
            raise ValueError(f'{name} contains a zero quaternion')
        if np.any(np.diff(values[:, 0]) <= 0):
            raise ValueError(f'{name} timestamps must be strictly increasing')
    pairs = associate(estimate, reference, tolerance)
    if len(pairs) < 3:
        raise ValueError('Fewer than three associated poses')
    est, ref = estimate[pairs[:, 0]], reference[pairs[:, 1]]
    ex, rx = est[:, 1:4], ref[:, 1:4]
    ec, rc = ex.mean(0), rx.mean(0)
    u, _, vt = np.linalg.svd((ex - ec).T @ (rx - rc))
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    translation = rc - rotation @ ec
    aligned = ex @ rotation.T + translation
    est_rot = rotation @ Rotation.from_quat(est[:, 4:8]).as_matrix()
    ref_rot = Rotation.from_quat(ref[:, 4:8]).as_matrix()
    rpe_translation, rpe_rotation = [], []
    for i, stamp in enumerate(est[:, 0]):
        j = np.searchsorted(est[:, 0], stamp + 1.0)
        options = [k for k in (j - 1, j) if i < k < len(est)]
        if not options:
            continue
        k = min(options, key=lambda x: abs(est[x, 0] - stamp - 1.0))
        if abs(est[k, 0] - stamp - 1.0) > tolerance:
            continue
        dr = ref_rot[i].T @ ref_rot[k]
        de = est_rot[i].T @ est_rot[k]
        tr = ref_rot[i].T @ (rx[k] - rx[i])
        te = est_rot[i].T @ (aligned[k] - aligned[i])
        rpe_translation.append(np.linalg.norm(te - tr))
        rpe_rotation.append(Rotation.from_matrix(dr.T @ de).magnitude())
    rmse = lambda v: float(np.sqrt(np.mean(np.square(v)))) if len(v) else None
    summary = {
        'reference_warning': 'Reference includes other VIO priors; it is not independent ground truth.',
        'alignment': 'SE(3), scale fixed to 1', 'time_tolerance_seconds': tolerance,
        'associated_poses': len(pairs), 'estimate_poses': len(estimate),
        'reference_poses': len(reference), 'association_coverage': len(pairs) / len(estimate),
        'overlap_estimate_poses': int(np.count_nonzero((estimate[:, 0] >= reference[0, 0]) &
                                                       (estimate[:, 0] <= reference[-1, 0]))),
        'overlap_start': float(est[0, 0]), 'overlap_end': float(est[-1, 0]),
        'ate_position_rmse_m': rmse(np.linalg.norm(aligned - rx, axis=1)),
        'rpe_interval_seconds': 1.0, 'rpe_pairs': len(rpe_translation),
        'rpe_translation_rmse_m': rmse(rpe_translation),
        'rpe_rotation_rmse_rad': rmse(rpe_rotation),
        'alignment_rotation': rotation.tolist(), 'alignment_translation': translation.tolist(),
    }
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'reference_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    np.savetxt(output / 'associated_aligned.txt', np.column_stack((est[:, 0], aligned, Rotation.from_matrix(est_rot).as_quat())))
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    figure, axes = plt.subplots(1, 2, figsize=(11, 4))
    axes[0].plot(rx[:, 0], rx[:, 1], label='Reference (VIO prior)')
    axes[0].plot(aligned[:, 0], aligned[:, 1], label='SchurVINS, SE(3) aligned')
    axes[0].set(xlabel='x [m]', ylabel='y [m]', aspect='equal')
    axes[0].legend()
    axes[1].plot(est[:, 0] - est[0, 0], np.linalg.norm(aligned - rx, axis=1))
    axes[1].set(xlabel='Time [s]', ylabel='Position difference [m]')
    figure.suptitle('Reference is not independent ground truth; scale fixed to 1')
    figure.tight_layout()
    figure.savefig(output / 'trajectory_reference.png', dpi=160)
    plt.close(figure)
    return summary
