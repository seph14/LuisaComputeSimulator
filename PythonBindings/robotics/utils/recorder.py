"""
Simulation frame recording and replay (ROADMAP B.4).

Saves per-frame mesh snapshots as OBJ files for offline viewing/animation.
"""

import os
import json
import numpy as np
from typing import List, Dict, Optional


class FrameRecorder:
    """Record simulation frames to disk for offline replay."""

    def __init__(self, output_dir: str, prefix: str = "frame"):
        self.output_dir = output_dir
        self.prefix = prefix
        self.frame_idx = 0
        self.metadata: List[Dict] = []  # per-frame joint values, timestamps
        os.makedirs(output_dir, exist_ok=True)

    def record(self, solver):
        """Save current frame as OBJ + metadata."""
        # Save mesh snapshot
        obj_path = os.path.join(self.output_dir,
                                f"{self.prefix}_{self.frame_idx:06d}.obj")
        solver.save_result(obj_path)

        # Save joint state metadata
        joint_values = solver.get_all_joint_values()
        joint_types = solver.get_all_joint_types()
        self.metadata.append({
            "frame": self.frame_idx,
            "joint_values": [float(v) for v in joint_values],
            "joint_types": [int(t) for t in joint_types],
        })
        self.frame_idx += 1

    def save_metadata(self, path: Optional[str] = None):
        """Write accumulated metadata as JSON."""
        if path is None:
            path = os.path.join(self.output_dir, "metadata.json")
        with open(path, "w") as f:
            json.dump(self.metadata, f, indent=2)

    def get_joint_trajectory(self, joint_idx: int) -> List[float]:
        """Extract trajectory for a specific joint."""
        return [m["joint_values"][joint_idx]
                for m in self.metadata
                if joint_idx < len(m["joint_values"])]


class FrameReplay:
    """Load recorded frames for offline analysis/visualization."""

    def __init__(self, record_dir: str, prefix: str = "frame"):
        self.record_dir = record_dir
        self.prefix = prefix
        self.metadata = self._load_metadata()

    def _load_metadata(self) -> List[Dict]:
        path = os.path.join(self.record_dir, "metadata.json")
        if os.path.exists(path):
            with open(path) as f:
                return json.load(f)
        return []

    @property
    def num_frames(self) -> int:
        return len(self.metadata)

    def get_frame_path(self, frame_idx: int) -> str:
        return os.path.join(self.record_dir,
                            f"{self.prefix}_{frame_idx:06d}.obj")

    def get_joint_angles(self, joint_idx: int) -> np.ndarray:
        return np.array([m["joint_values"][joint_idx] for m in self.metadata])

    def get_all_joint_angles(self) -> np.ndarray:
        """Return (num_frames, num_joints) array of joint values."""
        if not self.metadata:
            return np.array([])
        n_joints = len(self.metadata[0]["joint_values"])
        data = np.zeros((len(self.metadata), n_joints))
        for i, m in enumerate(self.metadata):
            data[i] = m["joint_values"][:n_joints]
        return data
