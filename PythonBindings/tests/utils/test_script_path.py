"""Project root path for test scripts.

Usage:
    from utils.test_script_path import PROJECT_ROOT
    cube_mesh_path = os.path.join(PROJECT_ROOT, "Resources", "InputMesh", "cube.obj")
    output_dir = os.path.join(PROJECT_ROOT, "Resources", "OutputMesh")
"""

import os
import sys

PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..")
)

BUILD_BIN = os.path.join(PROJECT_ROOT, "build", "bin")
if os.path.isdir(BUILD_BIN) and BUILD_BIN not in sys.path:
    sys.path.insert(0, BUILD_BIN)
