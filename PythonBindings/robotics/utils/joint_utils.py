"""
Joint state query and control helpers for robotics simulations.
"""

import numpy as np
import lcs_py as lcs

JTYPE_NAMES = {0: "FIX", 1: "PRI", 2: "REV"}


def get_joint_type_name(jtype_id):
    return JTYPE_NAMES.get(jtype_id, "UNK")


def log_joint_states(solver: lcs.NewtonSolver):
    joint_values = solver.get_all_joint_values()
    joint_types = solver.get_all_joint_types()
    parts = []
    for i in range(len(joint_values)):
        jt = JTYPE_NAMES.get(joint_types[i], "?")
        parts.append(f"{jt}={joint_values[i]:.4f}")
    return ", ".join(parts)


def print_joint_summary(solver):
    print(f"Joint count: {solver.get_joint_count()}")
    for i in range(solver.get_joint_count()):
        jtype = solver.get_joint_type(i)
        print(f"  Joint {i}: type={get_joint_type_name(jtype)} ({jtype})")


def get_revolute_angles(solver):
    vals = solver.get_all_joint_values()
    types = solver.get_all_joint_types()
    return [vals[i] for i in range(len(vals)) if types[i] == 2]


def get_prismatic_slides(solver):
    vals = solver.get_all_joint_values()
    types = solver.get_all_joint_types()
    return [vals[i] for i in range(len(vals)) if types[i] == 1]


def print_validation(solver):
    vals = solver.get_all_joint_values()
    types = solver.get_all_joint_types()
    for i in range(len(vals)):
        if types[i] == 2:
            print(f"Revolute joint {i} angle: {vals[i]:.4f} rad")
        elif types[i] == 1:
            print(f"Prismatic joint {i} slide: {vals[i]:.4f} m")
