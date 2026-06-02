#!/usr/bin/env python3
"""
Download robot assets for LuisaComputeSimulator robotics pipeline.

Sources:
  - newton-physics/newton-assets (H1, G1, Franka Panda)
  - google-deepmind/mujoco_menagerie (UR10e, ANYmal B/C, Allegro, Shadow Hand)

Usage:
  python scripts/download_robot_assets.py                        # download all
  python scripts/download_robot_assets.py --target unitree_h1    # single robot
  python scripts/download_robot_assets.py --list                 # list available

Requirements:
  git (for sparse checkouts)
  pip install pyyaml (optional, for manifest validation)

Output:
  PythonBindings/robotics/assets/
"""

import argparse
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ASSETS_DIR = ROOT / "PythonBindings" / "robotics" / "assets"

NEWTON_REPO = "https://github.com/newton-physics/newton-assets.git"
MUJOCO_REPO = "https://github.com/google-deepmind/mujoco_menagerie.git"

MANIFEST = {
    # (repo, sparse_path, dest_subdir, license_note)
    "unitree_h1": (NEWTON_REPO, "unitree_h1", "unitree_h1", "NVIDIA proprietary (Newton Bundle)"),
    "unitree_g1": (NEWTON_REPO, "unitree_g1", "unitree_g1", "NVIDIA proprietary (Newton Bundle)"),
    "franka_emika_panda": (NEWTON_REPO, "franka_emika_panda", "franka_emika_panda", "Apache 2.0 / BSD"),
    "universal_robots_ur10e": (MUJOCO_REPO, "universal_robots_ur10e", "universal_robots_ur10e", "BSD-3-Clause"),
    "universal_robots_ur5e": (MUJOCO_REPO, "universal_robots_ur5e", "universal_robots_ur5e", "BSD-3-Clause"),
    "anybotics_anymal_b": (MUJOCO_REPO, "anybotics_anymal_b", "anybotics_anymal_b", "BSD-3-Clause"),
    "anybotics_anymal_c": (MUJOCO_REPO, "anybotics_anymal_c", "anybotics_anymal_c", "BSD-3-Clause"),
    "wonik_allegro": (MUJOCO_REPO, "wonik_allegro", "wonik_allegro", "BSD-3-Clause"),
    "shadow_hand": (MUJOCO_REPO, "shadow_hand", "shadow_hand", "BSD-2-Clause"),
    "franka_panda_mjcf": (MUJOCO_REPO, "franka_emika_panda", "franka_panda_mjcf", "Apache 2.0"),
}


def sparse_clone(repo_url, sparse_path, dest_dir):
    dest_dir = Path(dest_dir)
    if dest_dir.exists() and list(dest_dir.iterdir()):
        print(f"  [{sparse_path}] already exists at {dest_dir}, skipping.")
        return True

    dest_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp) / "repo"
        print(f"  [{sparse_path}] cloning {repo_url} ...")
        try:
            subprocess.run(
                ["git", "clone", "--depth", "1", "--filter=blob:none",
                 "--sparse", repo_url, str(tmp_path)],
                check=True, capture_output=True, text=True,
            )
            subprocess.run(
                ["git", "-C", str(tmp_path), "sparse-checkout", "set", sparse_path],
                check=True, capture_output=True, text=True,
            )
            src = tmp_path / sparse_path
            if not src.exists():
                print(f"  WARNING: path {sparse_path} not found in repo {repo_url}")
                return False
            # Copy files
            for item in src.iterdir():
                item_dest = dest_dir / item.name
                if item.is_dir():
                    if item_dest.exists():
                        shutil.rmtree(item_dest)
                    shutil.copytree(item, item_dest)
                else:
                    shutil.copy2(item, item_dest)
        except subprocess.CalledProcessError as e:
            print(f"  ERROR: git clone failed: {e.stderr}")
            return False
    print(f"  [{sparse_path}] done.")
    return True


def main():
    parser = argparse.ArgumentParser(description="Download robot assets for LuisaComputeSimulator")
    parser.add_argument("--target", type=str, help="Download a single robot by key")
    parser.add_argument("--list", action="store_true", help="List available robots")
    args = parser.parse_args()

    if args.list:
        print("Available robots:")
        for key, (repo, path, dest, lic) in MANIFEST.items():
            print(f"  {key:30s}  ->  assets/{dest}  ({lic})")
        return

    ASSETS_DIR.mkdir(parents=True, exist_ok=True)

    targets = [args.target] if args.target else list(MANIFEST.keys())
    failed = []

    for key in targets:
        if key not in MANIFEST:
            print(f"Unknown target: {key}")
            failed.append(key)
            continue
        repo, sparse_path, dest, lic = MANIFEST[key]
        print(f"\n=== {key} ({lic}) ===")
        dest_path = ASSETS_DIR / dest
        if not sparse_clone(repo, sparse_path, dest_path):
            failed.append(key)

    if failed:
        print(f"\nFAILED: {', '.join(failed)}")
        sys.exit(1)
    print(f"\nAll assets downloaded to {ASSETS_DIR}")


if __name__ == "__main__":
    main()
