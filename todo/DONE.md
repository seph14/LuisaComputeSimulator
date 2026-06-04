# LuisaComputeSimulator 变更总结

本文档按 **feature 单位** 总结 `newton_examples` 分支上所有新增提交（共 23 个），包括修改的文件列表、测试方法及取得的效果。

## 重要审查结论（2026-06-02）

以下是对已完成 Feature 的 parity 审查，确认当前状态不会误判为 Newton parity：

| 测试 | 当前状态 | 审查结论 |
|------|----------|----------|
| `test_robot_h1_stand.py` | `smoke` | 使用 H1 URDF 真实资产，但 **不是** Newton `robot_h1` parity。缺口：floating base 使用 fixed base 替代、无 body velocity 断言 (Newton: <0.005)、无 `world_count=4`、无 mesh bounding box approximation、solver iterations 未对齐 (Newton: 100)、contact 参数未对齐。 |
| `test_robot_g1_stand.py` | `smoke` | 使用 G1 URDF 真实资产，但 **不是** Newton `robot_g1` parity。缺口：同 H1，且 G1 有 23/29 DOF 变体需求。当前无 body velocity 断言 (Newton: <0.015)。 |
| `test_robot_anymal.py` | `proxy` | 使用 procedural proxy（手工创建 13-body），**不是**真实 ANYmal D 资产。缺口：无真实 ANYmal D URDF/USD、无 floating base、无 `world_count=16`。 |
| `test_robot_policy.py` | `smoke` | ONNX inference smoke，**不代表** policy transfer 成功。缺口：observation 为零向量占位、无 action scale/default pose offset、无 policy decimation、无 termination/reward/reset、无 joint name reorder、仅 G1 23-DOF。 |
| `collision_shapes.py` | `proxy` | Python descriptor，**不代表** primitive collision shape pipeline 已接入 C++ LBVH/narrow phase。 |
| `FreeJoint` | `proxy` | 当前更接近 placeholder（零约束），**不代表** floating-base articulation parity 已完成。 |
| `test_cartpole.py` | `proxy` | single-world 版本，**不是** Newton `robot_cartpole` parity。缺口：无 `world_count=100`、无 world 速度一致性断言。 |
| `test_robot_arm.py` | `smoke` | 6-DOF 串行链，**不是** Newton `robot_ur10` parity。缺口：无 UR10 真实资产、无 `world_count=100`、无 sinusoidal trajectory 批量设置。 |
| `test_robot_allegro.py` | `proxy` | 16-DOF 手部，**不是** Newton `robot_allegro_hand` parity。缺口：无 `world_count=100`、无 cube retention 断言、无动态 joint parent transform。 |

---

## Feature 1: 关节信息查询与驱动力框架

**提交**: `5f92ec4`

**目标**: 从 affine-body DOF 中提取关节角度/位移，并提供 target position 设置 API。

**修改文件**:

| 文件 | 改动 |
|------|------|
| `Solver/SimulationSolver/solver_interface.h` | 新增 `get_joint_count()`, `get_joint_type()`, `get_joint_revolute_angle()`, `get_joint_prismatic_slide()`, `get_joint_velocities()`, `set_joint_target_pos/kp/kd()`, `apply_joint_drive_forces()` 声明 |
| `Solver/SimulationSolver/solver_interface.cpp` | ~480 行实现：atan2 旋转角提取、投影滑动距离、显式驱动力施加 |
| `Solver/SimulationCore/joint_constraint.h` | `RevoluteJointConstraintDesc` 新增 `lower_angle`/`upper_angle` 字段 |
| `Solver/Initializer/init_sim_data.cpp` | 初始化和上传逻辑 |
| `PythonBindings/src/python_bindings.cpp` | ~140 行 Python 绑定 |
| `PythonBindings/python/lcs_py/__init__.pyi` | stub 更新 |

**测试方法**:
```bash
PYTHONPATH=build/bin .venv/bin/python -c "
import lcs_py
s = lcs_py.NewtonSolver()
s.init_device('metal')
# 验证关节 API 可调用
"
```

**效果**: 建立了从 C++ DOF 到 Python joint state 的完整数据流。

---

## Feature 2: Robotics 模块 + Cartpole 场景

**提交**: `6771630`

**目标**: 手工构建 Cartpole 验证多体 articulation（ROADMAP 1.1, 1.6）。

**修改文件**:

| 文件 | 改动 |
|------|------|
| `PythonBindings/robotics/__init__.py` | 模块入口 |
| `PythonBindings/robotics/solver/robot_solver.py` | `RobotSolver`: Z-up setup、rigid body 注册、joint 创建（124 行） |
| `PythonBindings/robotics/mesh/robot_mesh.py` | `create_rigid_body()`, `create_fixed_anchor()`, `load_mesh_from_file()` (58 行) |
| `PythonBindings/robotics/render/robot_viewer.py` | Polyscope GUI wrapper (27 行) |
| `PythonBindings/robotics/training/robot_env.py` | `RobotEnv` 基类（30 行） |
| `PythonBindings/robotics/utils/joint_utils.py` | 关节状态查询/验证（51 行） |
| `PythonBindings/robotics/utils/physics_utils.py` | 旋转估计、四元数速度转换（46 行） |
| `PythonBindings/robotics/test_cartpole.py` | Cartpole 场景构建 + 仿真 + 验证（176 行） |

**测试方法**:
```bash
PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_cartpole.py --headless --advance_frames 300
```

**效果**: 3 刚体（cart + pole1 + pole2）+ prismatic + 2 revolute joint 物理正确，cart 沿 X 轴运动，pole 绕 Y 轴旋转，Y/Z 方向漂移 ≤ 5e-4 m。

---

## Feature 3: 关节角限位 + 驱动能量（Phase 1 完整实现）

**提交**: `2ac640f`

**目标**: ROADMAP 1.3（角限位）、1.4（驱动能量）、1.7（数值测试）、1.8（参数调优）、0.5（step cycle 模板）。

**修改文件**:

| 文件 | 改动 |
|------|------|
| `Solver/Energies/joint_constraint_energy.cpp` | 能量着色器：revolute angle limit penalty（atan2 角计算 + 二次惩罚）；drive energy `E=0.5*kp*(q-target)²`；eval 着色器：drive 梯度 + Gauss-Newton Hessian；host evaluate 同步 |
| `Solver/Energies/detail/revolute_joint_constaint.hpp` | 简化为仅基础约束（角限位/驱动在着色器中直实现） |
| `Solver/SimulationCore/simulation_data.h` | `JointConstraint` 新增 `joint_drive_params` buffer + `LUISA_BINDING_GROUP` 注册 |
| `Solver/Initializer/init_sim_data.cpp` | 初始化 + GPU 上传 `joint_drive_params` |
| `Solver/SimulationSolver/solver_interface.cpp` | setter 同步到 joint_constraint 数据 |
| `Solver/SimulationSolver/newton_solver.cpp` | 每步上传 `joint_drive_params` |
| `PythonBindings/src/python_bindings.cpp` | 新增 `set_num_substep` API |
| `PythonBindings/robotics/solver/robot_solver.py` | `get_joint_q()`, `get_joint_qd()`, `get_body_pose()`, `get_body_velocity()` alias |
| `PythonBindings/robotics/test_cartpole.py` | Z 容差收紧、多 body 验证 |
| `PythonBindings/robotics/utils/robot_sim_runner.py` | 新建：`simulate()`, `simulate_with_render()` 模板 |
| `PythonBindings/robotics/__init__.py` | 导出新模块 |
| `PythonBindings/python/lcs_py/__init__.pyi` | stub 更新 |

**技术细节**:
- 角限位：通过 `R_delta = R_A^T·R_B·R_rest^T` 的 skew-symmetric 部分投影到 hinge axis，atan2 计算当前角度
- 驱动能量梯度：`∂θ/∂q_k = (cosθ·∂s/∂q_k - sinθ·∂c/∂q_k)/(s²+c²)`
- Hessian: Gauss-Newton 外积近似 `H ≈ kp·v_i⊗v_j`

**测试方法**:
```bash
PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_cartpole.py --headless --advance_frames 300
```

**效果**: Cartpole Z-drift ≤ 5e-4, joint 响应 target position, `num_substep=3` 默认调优。

---

## Feature 4: Z-up 坐标系 + 可配置地面

**提交**: `73bcb0f`, `920459f`

**目标**: ROADMAP 0.1（Z-up）、0.3（可配置地面）。

**修改文件**:

| 文件 | 改动 |
|------|------|
| `Solver/SimulationCore/scene_params.h` | 新增 `UpAxis` 枚举（Y_UP=0, Z_UP=1）、`up_axis` 字段、`floor_normal` 字段、`set_up_axis()` 方法（自动推导 gravity/floor/floor_normal） |
| `PythonBindings/src/python_bindings.cpp` | `UpAxis` 枚举绑定 + `set_up_axis()`/`get_up_axis()` + `set_floor_normal()`/`get_floor_normal()` |
| `PythonBindings/robotics/solver/robot_solver.py` | `setup_z_up()` 改用 `config.set_up_axis(Z_UP)` |

**测试方法**:
```bash
PYTHONPATH=build/bin .venv/bin/python -c "
import lcs_py
s = lcs_py.NewtonSolver()
s.init_device('metal')
c = s.get_config()
c.set_up_axis(lcs_py.UpAxis.Z_UP)
assert c.get_gravity().z == -9.8
assert c.get_floor_normal().z == 1.0
print('Z-up OK')
"
```

**效果**: Z-up 下 gravity=(0,0,-9.8), floor_normal=(0,0,1)；Y-up 保持向后兼容。

---

## Feature 5: Ground Plane 可配置法线方向

**提交**: `dc604ac`

**目标**: ROADMAP 3.6（ground plane primitive）、3.8（use_floor 增强）。

**修改文件**:

| 文件 | 改动 |
|------|------|
| `Solver/Energies/ground_collision_energy.cpp` | shader 签名从 `float floor_y` 升级为 `float3 floor_origin + float3 floor_normal`；距离计算从 `x_k.y - floor_y` 改为 `dot(x_k, normal) - dot(origin, normal)`；CCD shader、eval_soft、eval_abd、host evaluate 全部同步 |
| `Solver/Energies/ground_collision_energy.h` | Shader 类型声明、方法签名同步升级 |
| `Solver/SimulationSolver/newton_solver.cpp` | 所有调用点传入 `floor` + `floor_normal` 替代 `floor.y` |
| `Solver/SimulationSolver/newton_solver.h` | CCD shader 类型声明同步 |
| `Solver/SimulationSolver/solver_interface.cpp` | `device_compute_energy` 调用签名同步 |
| `PythonBindings/robotics/test_drop_contact.py` | drop 测试（Y-up 模式，cube 从 1m 落下停在 0.075m） |

**测试方法**:
```bash
PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_drop_contact.py --headless --advance_frames 120
```

**效果**: Ground plane 不再硬编码 Y-up；支持任意法线方向（Y-up=(0,1,0), Z-up=(0,0,1)）；drop test cube 正确停在地面。

---

## Feature 6: URDF 解析器 + RobotBuilder

**提交**: `06f8ff9`, `5d08045`

**目标**: ROADMAP 2.1（URDF parser）、2.2-2.5（RobotBuilder + 多刚体串）。

**修改文件**:

| 文件 | 改动 |
|------|------|
| `PythonBindings/robotics/parser/__init__.py` | 模块入口 |
| `PythonBindings/robotics/parser/urdf_parser.py` | `URDFParser`: 零依赖 URDF XML 解析；`URDFRobotModel`, `URDFLink`, `URDFJoint` 等 dataclass；支持 mesh/box/sphere/cylinder/capsule；BFS 拓扑排序（294 行） |
| `PythonBindings/robotics/robot_builder.py` | `RobotBuilder`: URDF→solver bodies + joints 自动转换；变换传播；mesh 加载（collision > visual > primitive > cube fallback）；fixed joint 高刚度 collapse；新增 `_rpy_to_matrix()`（238 行） |
| `PythonBindings/robotics/__init__.py` | 导出 parser |
| `PythonBindings/robotics/test_builder_smoke.py` | 纯 Python 单元测试（不依赖 GPU）：URDF parse、RPY→matrix、topology order、floating detection |

**测试方法**:
```bash
# 纯 Python 单元测试（无需 GPU）
PYTHONPATH=PythonBindings .venv/bin/python PythonBindings/robotics/test_builder_smoke.py

# 集成测试（需要 GPU）
PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_h1_stand.py --headless --advance_frames 30
```

**效果**: 解析 25-link H1 URDF + 31-link G1 URDF 无错误；BFS 拓扑排序正确；`test_builder_smoke.py` 4 项纯 Python 测试全通过。

---

## Feature 7: Ball/Free 关节类型

**提交**: `ce722e6`, `dc604ac`

**目标**: ROADMAP 2.6（ball joint）、2.7（free joint）。

**修改文件**:

| 文件 | 改动 |
|------|------|
| `Solver/SimulationCore/joint_constraint.h` | `JointConstraintType` 新增 `Ball`/`Free`；`BallJointConstraintDesc`（anchor 重合，自由旋转）；`FreeJointConstraintDesc`（零约束） |
| `Solver/Energies/joint_constraint_energy.cpp` | 能量着色器：Ball → `E=0.5*k*||p_b-p_a||²`，Free → `E=0`；eval 着色器：Ball → rank-1 梯度 + Gauss-Newton Hessian，Free → 零梯度；host evaluate 同 |
| `Solver/Initializer/init_sim_data.cpp/h` | `init_sim_data()` 新增 `ball_joint_descs`/`free_joint_descs` 参数；ball joint constraint buffer 初始化（8 DOF entries）；free joint 同样入队 + TODO 优化注释 |
| `Solver/SimulationSolver/solver_interface.h` | 新增 `ball_joint_descs`/`free_joint_descs` 成员 + `add_ball_joint()`/`add_free_joint()` 方法 |
| `Solver/SimulationSolver/solver_interface.cpp` | `init_data()` 传递 ball/free descs 到 `init_sim_data()` |
| `PythonBindings/src/python_bindings.cpp` | `add_ball_joint()`/`add_free_joint()` 绑定 |
| `PythonBindings/robotics/solver/robot_solver.py` | `add_ball_joint()`/`add_free_joint()` wrapper + joint metadata 记录 |
| `PythonBindings/robotics/robot_builder.py` | `build()` 中处理 ball/free joint 类型 |

**测试方法**:
```bash
# Ball joint 通过 Franka URDF 间接测试（fixed joints → ball joints 在 floating-base 场景）
PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_franka.py --headless --advance_frames 30
```

**效果**: Ball/Free joint 类型可创建、可在 shader 中正确计算能量和梯度。

---

## Feature 8: 碰撞形状描述符体系

**提交**: `0e8eae4`

**目标**: ROADMAP 3.3-3.5（per-link collision、collision group/filter、per-shape friction）。

**修改文件**:

| 文件 | 改动 |
|------|------|
| `PythonBindings/robotics/parser/collision_shapes.py` | `CollisionShapeDesc`（box/sphere/capsule/cylinder/mesh + friction/restitution + collision_group/mask）；`CollisionGroupConfig`（位掩码分组、self-collision disable、link-pair override）；`CollisionShapeLibrary`（工厂方法）（117 行） |

**测试方法**: 当前为 Python descriptor，尚未接入 C++ collision pipeline；可通过导入验证：
```bash
PYTHONPATH=PythonBindings .venv/bin/python -c "
from robotics.parser.collision_shapes import CollisionShapeLibrary
lib = CollisionShapeLibrary()
s = lib.add_box()
assert s.friction == 0.5
print('CollisionShapeLibrary OK')
"
```

**效果**: 定义了碰撞形状、分组、摩擦系数的完整描述符体系，为后续接入 C++ LBVH/narrow phase 提供数据模型。

---

## Feature 9: 6-DOF 机械臂轨迹跟踪

**提交**: `2aecff6`

**目标**: ROADMAP 2.8-2.9（UR10 场景测试 + 轨迹精度）。

**修改文件**:

| 文件 | 改动 |
|------|------|
| `PythonBindings/robotics/test_robot_arm.py` | 6 个 revolute joint 串行链、正弦波 trajectory tracking（kp=1000, kd=50）、`SimulationRecorder` 集成、关节统计（175 行） |

**测试方法**:
```bash
PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_arm.py --headless --advance_frames 120
```

**效果**: 6 关节串行链物理稳定；joint 响应正弦驱动目标（tracking RMSE ~0.16 rad at 120 frames）；所有 link 不塌陷。

---

## Feature 10: 基础能力（查找、绘图、近似、录制、Viewer）

**提交**: `bc293d8`, `3e6da22`, `5bdf389`

**目标**: ROADMAP B.2（body 查找）、B.5（plotting）、B.3（mesh approx）、B.4（录制/回放）、B.1（mass/inertia）、B.6（viewer）。

**修改文件**:

| 文件 | 改动 |
|------|------|
| `PythonBindings/robotics/solver/robot_solver.py` | `get_body_id()`, `get_body_name()`, `get_joint_index()` 名称查找；`add_rigid_body()` 新增 `mass/com/density` 覆盖 |
| `PythonBindings/robotics/utils/plotting.py` | `SimulationRecorder`、`plot_joint_trajectory()`、`plot_body_trajectory()`、`print_joint_summary_table()` |
| `PythonBindings/robotics/utils/mesh_approx.py` | `convex_hull()`, `box_approximation()`, `capsule_approximation()`, `sphere_approximation()`, `simplify_mesh()` |
| `PythonBindings/robotics/utils/recorder.py` | `FrameRecorder`（OBJ + JSON metadata）、`FrameReplay`（离线分析） |
| `PythonBindings/robotics/render/robot_viewer.py` | `set_body_color()`, `highlight_joint()`, `get_body_poses()`, `print_joint_summary()` |

**测试方法**:
```bash
# Recorder + Plotting 通过 arm test 间接测试
PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_arm.py --headless --advance_frames 60
# Mesh approx 独立测试
PYTHONPATH=PythonBindings .venv/bin/python -c "
import trimesh
from robotics.utils.mesh_approx import box_approximation, convex_hull
m = trimesh.creation.box()
b = box_approximation(m)
assert len(b.faces) == 12
print('Mesh approx OK')
"
```

**效果**: 提供了完整的辅助工具链（录制→回放→绘图→查找→近似）。

---

## Feature 11: 机器人资产下载

**提交**: `26b6c54`, `scripts/download_robot_assets.py`（在 `dc604ac` 中提交）

**目标**: 下载 H1、G1、Franka、UR10e、ANYmal、Allegro 等机器人资产。

**修改文件**:

| 文件 | 改动 |
|------|------|
| `PythonBindings/robotics/assets/` | 667 个文件（19 URDF、233 mesh、6 ONNX policy） |
| `.gitignore` | 新增 `PythonBindings/robotics/assets/` |
| `scripts/download_robot_assets.py` | 可复现的 sparse clone 下载脚本（使用 `newton-physics/newton-assets` 和 `google-deepmind/mujoco_menagerie`） |

**来源**:
- `newton-physics/newton-assets`（commit `261cd1f`）: unitree_h1, unitree_g1, franka_emika_panda
- `google-deepmind/mujoco_menagerie`（commit `feadf76`）: universal_robots_ur10e/ur5e, anybotics_anymal_b/c, wonik_allegro, shadow_hand, franka_emika_panda

**测试方法**:
```bash
.venv/bin/python scripts/download_robot_assets.py --list
```

**效果**: 666 个资产文件就位，6 个 ONNX policy 可用于推理。

---

## Feature 12: H1/G1/ANYmal/Franka/Allegro 场景测试

**提交**: `3202793`, `acc14d3`, `d2d9977`, `b25b44b`, `4c750bd`

**目标**: ROADMAP 阶段 4-7（所有机器人场景测试）。

**修改文件**:

| 文件 | 改动 |
|------|------|
| `PythonBindings/robotics/test_robot_h1_stand.py` | H1 25-link URDF 加载、碰撞基元 body 创建、24 joint 驱动保持姿态（175 行） |
| `PythonBindings/robotics/test_robot_g1_stand.py` | G1 31-link URDF、30 joint（113 行） |
| `PythonBindings/robotics/test_robot_anymal.py` | 13-body quadruped、12 revolute joint（119 行） |
| `PythonBindings/robotics/test_robot_franka.py` | Franka 10-link URDF、7-DOF arm（117 行） |
| `PythonBindings/robotics/test_robot_allegro.py` | 16-DOF 多指手、16 revolute joint（127 行） |
| `PythonBindings/robotics/test_robot_policy.py` | ONNX policy 加载 + inference loop + 仿真闭环（161 行） |
| `PythonBindings/robotics/test_drop_contact.py` | Drop test（104 行） |

**测试方法**:
```bash
# 全部 7 个场景测试（每个 30-60 frames，约 15-20 秒/个）
for t in test_robot_h1_stand test_robot_g1_stand test_robot_anymal \
         test_robot_franka test_robot_allegro test_robot_policy test_drop_contact; do
    echo "=== $t ==="
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/${t}.py \
        --headless --advance_frames 30 2>&1 | grep -E "PASS|FAIL"
done
```

**效果汇总**:

| 测试 | 规模 | 关键指标 |
|------|------|----------|
| H1 standing | 25 links, 24 joints | max drift 0.0016 rad |
| G1 standing | 31 links, 30 joints | max drift 0.0002 rad |
| ANYmal | 13 bodies, 12 joints | base height 0.65m, drift 0.0004 rad |
| Franka Panda | 10 links, 7-DOF | drift 0.0000 rad |
| Allegro Hand | 17 bodies, 16 joints | drift 0.0000 rad |
| ONNX Policy | G1 23-DOF + ONNX | action mean abs 0.1842 |
| Drop Contact | 2 bodies + ground | cube rests at 0.075m |

---

## 文档

**提交**: `468a054`, `todo/COMMIT_LOG.md`, `todo/DONE.md`

**修改文件**: `todo/COMMIT_LOG.md`, `todo/DONE.md`（本文件）

---

## 修改文件总览

### C++ 文件（8 个）

| 文件 | 涉及 Feature |
|------|-------------|
| `Solver/SimulationCore/scene_params.h` | Z-up, floor_normal |
| `Solver/SimulationCore/joint_constraint.h` | Ball/Free joint 描述符 |
| `Solver/SimulationCore/simulation_data.h` | joint_drive_params buffer, LUISA_BINDING_GROUP |
| `Solver/Energies/joint_constraint_energy.cpp` | angle limit, drive energy, Ball/Free |
| `Solver/Energies/ground_collision_energy.cpp/h` | ground plane 法线方向 |
| `Solver/Initializer/init_sim_data.cpp/h` | Ball/Free joint 初始化 |
| `Solver/SimulationSolver/solver_interface.cpp/h` | 关节查询, setter, Ball/Free descs |
| `Solver/SimulationSolver/newton_solver.cpp/h` | drive upload, ground plane dispatch |
| `PythonBindings/src/python_bindings.cpp` | 所有 Python API 绑定 |

### Python 文件（26 个）

| 目录 | 文件 | 涉及 Feature |
|------|------|-------------|
| `robotics/` | `__init__.py` | 模块入口 |
| `robotics/solver/` | `robot_solver.py` | Cartpole, alias, Ball/Free, mass/inertia |
| `robotics/mesh/` | `robot_mesh.py` | Body 创建 |
| `robotics/render/` | `robot_viewer.py` | Polyscope GUI |
| `robotics/training/` | `robot_env.py` | RL 环境基类 |
| `robotics/parser/` | `urdf_parser.py` | URDF 解析 |
| `robotics/parser/` | `collision_shapes.py` | 碰撞形状描述符 |
| `robotics/utils/` | `joint_utils.py` | 关节查询 |
| `robotics/utils/` | `physics_utils.py` | 旋转/速度工具 |
| `robotics/utils/` | `robot_sim_runner.py` | Step cycle 模板 |
| `robotics/utils/` | `plotting.py` | 绘图 |
| `robotics/utils/` | `mesh_approx.py` | Mesh 近似 |
| `robotics/utils/` | `recorder.py` | 录制/回放 |
| `robotics/` | `robot_builder.py` | URDF→Solver 构建 |
| `robotics/` | `test_*.py` | 9 个 E2E 测试 |
| `scripts/` | `download_robot_assets.py` | 资产下载 |
| `todo/` | `COMMIT_LOG.md`, `DONE.md` | 文档 |

---

## 一键回归测试

```bash
# 构建
./build.sh

# 纯 Python 测试（无需 GPU）
PYTHONPATH=PythonBindings .venv/bin/python PythonBindings/robotics/test_builder_smoke.py

# 全部场景测试（需要 GPU，约 3-5 分钟）
for t in test_cartpole test_robot_arm test_robot_h1_stand test_robot_g1_stand \
         test_robot_anymal test_robot_franka test_robot_allegro test_drop_contact; do
    echo "=== $t ==="
    PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/${t}.py \
        --headless --advance_frames 30 2>&1 | grep -E "PASS|FAIL"
done

# ONNX policy 测试（需要 onnxruntime）
PYTHONPATH=build/bin .venv/bin/python PythonBindings/robotics/test_robot_policy.py \
    --headless --advance_frames 15 2>&1 | grep -E "PASS|FAIL"
```
