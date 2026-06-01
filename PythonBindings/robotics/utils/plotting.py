"""
Plotting utilities for robotics simulations (ROADMAP B.5).

Supports joint trajectory, energy, contact count, and body state plotting.
Uses matplotlib (optional — graceful fallback if not installed).
"""

from typing import List, Dict, Optional
import numpy as np

try:
    import matplotlib.pyplot as plt
    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False


class SimulationRecorder:
    """Record simulation state over time for later plotting."""

    def __init__(self):
        self.frames: List[int] = []
        self.joint_q: Dict[int, List[float]] = {}  # joint_idx -> [values]
        self.joint_qd: Dict[int, List[float]] = {}
        self.energies: Dict[str, List[float]] = {}
        self.contact_counts: List[int] = []
        self.body_positions: Dict[str, List[np.ndarray]] = {}

    def record(self, frame: int, solver):
        """Record current simulation state."""
        self.frames.append(frame)

        # Joint positions and velocities
        q_vals = solver.get_all_joint_values()
        qd_vals = solver.get_all_joint_velocities() if hasattr(solver, 'get_all_joint_velocities') else []
        for i, v in enumerate(q_vals):
            if i not in self.joint_q:
                self.joint_q[i] = []
            self.joint_q[i].append(v)
        for i, v in enumerate(qd_vals):
            if i not in self.joint_qd:
                self.joint_qd[i] = []
            self.joint_qd[i].append(v)

    def record_body(self, name: str, center: np.ndarray):
        """Record body center position."""
        if name not in self.body_positions:
            self.body_positions[name] = []
        self.body_positions[name].append(center.copy())


def plot_joint_trajectory(recorder: SimulationRecorder,
                          joint_labels: Optional[Dict[int, str]] = None,
                          title: str = "Joint Trajectory",
                          save_path: Optional[str] = None):
    """Plot joint position and velocity trajectories."""
    if not _HAS_MPL:
        print("[plotting] matplotlib not available, skipping plot")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    for jidx, values in recorder.joint_q.items():
        label = joint_labels.get(jidx, f"Joint {jidx}") if joint_labels else f"Joint {jidx}"
        ax1.plot(recorder.frames, values, label=label)
    ax1.set_ylabel("Position (rad/m)")
    ax1.set_title(f"{title} — Positions")
    ax1.legend()
    ax1.grid(True)

    for jidx, values in recorder.joint_qd.items():
        label = joint_labels.get(jidx, f"Joint {jidx}") if joint_labels else f"Joint {jidx}"
        ax2.plot(recorder.frames, values, label=label)
    ax2.set_xlabel("Frame")
    ax2.set_ylabel("Velocity (rad/s or m/s)")
    ax2.set_title(f"{title} — Velocities")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
    plt.show()


def plot_body_trajectory(recorder: SimulationRecorder,
                         body_names: Optional[List[str]] = None,
                         axis: str = "z",
                         title: str = "Body Trajectory",
                         save_path: Optional[str] = None):
    """Plot body center trajectories along a specified axis."""
    if not _HAS_MPL:
        print("[plotting] matplotlib not available, skipping plot")
        return

    axis_idx = {"x": 0, "y": 1, "z": 2}[axis.lower()]
    fig, ax = plt.subplots(figsize=(10, 4))

    names = body_names or list(recorder.body_positions.keys())
    for name in names:
        if name in recorder.body_positions:
            vals = [p[axis_idx] for p in recorder.body_positions[name]]
            ax.plot(recorder.frames[:len(vals)], vals, label=name)

    ax.set_xlabel("Frame")
    ax.set_ylabel(f"{axis.upper()} Position (m)")
    ax.set_title(title)
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
    plt.show()


def print_joint_summary_table(recorder: SimulationRecorder,
                              joint_labels: Optional[Dict[int, str]] = None):
    """Print a summary table of joint statistics."""
    print(f"\n{'='*60}")
    print(f"Joint Summary ({len(recorder.frames)} frames)")
    print(f"{'='*60}")
    print(f"{'Joint':<12} {'Min':>10} {'Max':>10} {'Mean':>10} {'Std':>10}")
    print(f"{'-'*60}")
    for jidx, values in recorder.joint_q.items():
        label = joint_labels.get(jidx, f"J{jidx}") if joint_labels else f"J{jidx}"
        arr = np.array(values)
        print(f"{label:<12} {arr.min():10.4f} {arr.max():10.4f} "
              f"{arr.mean():10.4f} {arr.std():10.4f}")
    print(f"{'='*60}")
