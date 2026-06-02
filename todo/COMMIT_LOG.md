# LuisaComputeSimulator 提交详细记录

本文档记录 `newton_examples` 分支相对 `origin/newton_examples` 领先的提交、对应 ROADMAP 任务、改动文件、技术细节，以及对当前实现质量和后续计划的审查结论。

**口径说明**:
- 当前分支相对 `origin/newton_examples` 领先 21 个提交：20 个代码/资产提交（`4835e42` 到 `4c750bd`）+ 1 个文档提交（`468a054`，新增本文件）。
- `24af0c5` 和 `1dfd734` 位于当前分支历史中，但不是相对 `origin/newton_examples` 的新增提交；下文保留它们作为基线背景。
- “ROADMAP 已覆盖”不等同于“Newton example 已 1:1 复现”。本分支很多测试是 smoke / scaffold / proxy 场景，不能替代 Newton README 的 exact scene parity。

---

## 前期基础设施（会话开始前已存在的提交）

### 1. `24af0c5` — 初始穿透检测
- **ROADMAP**: 无（前置工作）
- **改动**: 为后续 collision pipeline 添加初始穿透检测能力
- **文件**: Solver/CollisionDetector/narrow_phase.*

### 2. `1dfd734` — 关节体间碰撞剔除
- **ROADMAP**: 无（前置工作）
- **改动**: 在 joint-connected bodies 之间禁用碰撞检测，避免关节约束与碰撞力冲突
- **文件**: Solver/CollisionDetector/*, Solver/SimulationSolver/*

### 3. `4835e42` — 小准备
- **ROADMAP**: 无（前置工作）
- **改动**: 代码整理，为后续 robotics 模块做准备
- **文件**: 多个文件小幅修改

### 4. `5f92ec4` — 关节信息查询 + 关节驱动力
- **ROADMAP**: 1.2（joint q/qd 查询）, 1.5（set_joint_target_pos）
- **改动**:
  - C++ 端新增 `get_joint_revolute_angle()`, `get_joint_prismatic_slide()`, `get_joint_velocities()`
  - C++ 端新增 `set_joint_target_pos/kp/kd()` 和 `apply_joint_drive_forces()`
  - Python bindings 暴露所有关节查询和控制 API
  - `JointConstraintDesc` 新增 `lower_angle`/`upper_angle` 字段
- **文件**: `solver_interface.cpp/h`, `python_bindings.cpp`, `joint_constraint.h`, `init_sim_data.cpp`
- **技术细节**: 从 affine-body DOF 数据通过旋转矩阵分解计算 revolute angle（atan2 方法），通过投影计算 prismatic slide distance

### 5. `6771630` — Robotics 模块初始化
- **ROADMAP**: 1.1（Cartpole 场景）, 1.6（body pose/velocity）
- **改动**:
  - 新建 `PythonBindings/robotics/` 模块（solver, mesh, render, training, utils 子模块）
  - `RobotSolver` 类封装 NewtonSolver，提供 Z-up setup、rigid body 注册、joint 创建
  - `test_cartpole.py`: 手工构建 3 刚体 cartpole（cart + pole1 + pole2），prismatic + 2 revolute joints
  - `robot_mesh.py`: 刚体 mesh 创建、固定 anchor 创建
  - `joint_utils.py`: 关节状态查询/验证工具
  - `physics_utils.py`: 旋转矩阵估计、四元数速度转换
- **文件**: 15 个新文件，530 行新增

---

## Phase 0：基础设施准备（本会话完成）

### 6. `2ac640f` — Phase 1 完整实现
- **ROADMAP**: 1.3（revolute angle limit）, 1.4（joint drive energy）, 1.7（数值测试）, 1.8（参数调优）, 0.5（step cycle 模板）
- **改动**:
  - **1.3 角限位**: 能量着色器中新增 angle limit penalty，通过 `R_delta` 矩阵提取旋转角度，超出范围时施加 `0.5*k*(θ-limit)²` 二次惩罚
  - **1.4 驱动能量**: 在能量着色器、eval 着色器、host evaluate 中新增隐式 drive energy `E=0.5*kp*(q-target)²`，带完整梯度（Gauss-Newton Hessian 近似）
  - **数据流**: `JointConstraint` 结构体新增 `joint_drive_params` GPU buffer（(target_pos, kp, kd) per joint），在 `LUISA_BINDING_GROUP` 中注册
  - **per-step upload**: `newton_solver.cpp` 中新增每步上传 `joint_drive_params`，保证 setter 调用后 GPU 数据同步
  - **1.7 测试增强**: Z drift 容差收紧到 3e-4，新增多 body Z 轴约束验证
  - **1.8 参数调优**: `setup_z_up()` 默认 `num_substep=3`，新增 `set_num_substep` Python API
  - **0.5 模板**: 新建 `robot_sim_runner.py`（`simulate()`, `simulate_with_render()`）
  - **便捷 API**: `RobotSolver` 新增 `get_joint_q()`, `get_joint_qd()`, `get_body_pose()`, `get_body_velocity()`
- **文件**: 12 个文件，470 行新增
- **技术细节**: 
  - Revolute angle 通过 `R_delta = R_A^T·R_B·R_rest^T` 的 skew-symmetric 部分投影到 hinge axis 计算
  - 驱动能量梯度通过链式法则 `∂θ/∂q_k = (cosθ·∂s/∂q_k - sinθ·∂c/∂q_k)/(s²+c²)` 计算
  - Hessian 使用 Gauss-Newton 外积近似 `H ≈ kp·v_i⊗v_j`（忽略二阶项）

### 7. `73bcb0f` — Z-up 坐标系（ROADMAP 0.1）
- **改动**:
  - `SceneParams` 新增 `UpAxis` 枚举（`Y_UP=0`, `Z_UP=1`）
  - `set_up_axis()` 方法自动推导 gravity/floor 默认值：Z_UP → `gravity=(0,0,-9.8)`, Y_UP → `gravity=(0,-9.8,0)`
  - Python bindings 暴露 `UpAxis` 枚举 + `set_up_axis()`/`get_up_axis()` 方法
- **文件**: `scene_params.h`, `python_bindings.cpp`
- **技术细节**: 保持向后兼容（默认 Y_UP），`floor_normal` 在后续提交中添加

### 8. `920459f` — 可配置地面（ROADMAP 0.3）
- **改动**:
  - `SceneParams` 新增 `floor_normal` 字段（Y-up 默认 `(0,1,0)`, Z-up 默认 `(0,0,1)`）
  - `set_up_axis()` 自动设置对应的 `floor_normal`
  - Python bindings 暴露 `set/get_floor_normal`
- **文件**: `scene_params.h`, `python_bindings.cpp`
- **完成度**: 部分完成。`floor_normal` 目前只是配置接口；ground collision energy、ground CCD 和 `newton_solver.cpp` 仍使用 `floor.y` / Y-up 平面逻辑。
- **技术细节**: `floor_normal` 为后续 ground plane shader 升级预留接口，尚不能宣称任意平面地面可用。

---

## Phase 2：URDF Importer + UR10 Position Control

### 9. `06f8ff9` — URDF 解析器（ROADMAP 2.1）
- **改动**:
  - 纯 Python 实现，使用 `xml.etree.ElementTree` 解析 URDF XML
  - 数据类：`URDFRobotModel`, `URDFLink`, `URDFJoint`, `URDFInertial`, `URDFVisual`, `URDFCollision`
  - 支持所有标准 geometry 类型：mesh, box, sphere, cylinder, capsule
  - 提取 joint limits, axis, dynamics (damping/friction)
  - 构建父子拓扑图 + 拓扑排序（BFS）
- **文件**: `robotics/parser/__init__.py`, `urdf_parser.py`（294 行）
- **技术细节**: 零外部依赖，通过 `<joint>` 的 `<parent>/<child>` 关系自动推导 root link

### 10. `5d08045` — RobotBuilder（ROADMAP 2.2-2.5）
- **改动**:
  - `RobotBuilder` 类：URDFRobotModel → solver bodies + joints 的自动转换
  - 父子链式变换传播（BFS 遍历 link tree，使用 scipy 旋转矩阵）
  - Mesh 加载优先级：collision mesh > visual mesh > primitive > cube fallback
  - 支持 box/sphere/cylinder/capsule 原始几何体生成
  - Fixed joint 使用高刚度 penalty 近似 collapse
- **文件**: `robot_builder.py`（238 行）
- **技术细节**: 所有变换在 Python 端预计算（世界坐标），通过 `add_rigid_body` 的 tx/ty/tz 参数传入

### 11. `ce722e6` — Ball/Free 关节（ROADMAP 2.6-2.7）
- **改动**:
  - `JointConstraintType` 枚举新增 `Ball`（球铰）和 `Free`（浮动基座）
  - `BallJointConstraintDesc`: anchor 重合约束，允许自由旋转
  - `FreeJointConstraintDesc`: 零约束（占位符，用于 floating base robot）
  - 能量着色器：Ball → `E=0.5*k*||p_b-p_a||²`, Free → `E=0`
  - Eval 着色器：Ball → 解析梯度（rank-1 外积 Hessian）, Free → 零梯度
  - Host evaluate 同逻辑
- **文件**: `joint_constraint.h`, `joint_constraint_energy.cpp`
- **完成度**: 部分完成。类型和 energy 分支已经存在，但 `init_sim_data.cpp` 仍只初始化 Fixed/Prismatic/Revolute，`solver_interface`/Python bindings/`RobotBuilder` 尚没有可实际创建 Ball/Free joint 的完整入口。
- **技术细节**: Ball joint 是最简单的约束类型，仅需位置惩罚，无需轴/角度计算；后续必须补齐 descriptor 入队、buffer 初始化、API 暴露和测试。

### 12. `2aecff6` — 6-DOF 机械臂测试（ROADMAP 2.8-2.9）
- **改动**:
  - 6 个 revolute joint 串行链（UR10-like 比例）
  - 正弦波 joint trajectory tracking，kp=1000, kd=50
  - SimulationRecorder 集成，打印关节统计
  - 验证：关节响应驱动目标（RMSE < 1.0 rad），所有 link 不塌陷
- **文件**: `test_robot_arm.py`（175 行）
- **技术细节**: 使用过程化几何体（cube mesh），无需外部 URDF 资产

---

## Phase 0 收尾 + 基础能力

### 13. `bc293d8` — Body 查找 + 绘图（ROADMAP B.2, B.5）
- **改动**:
  - `RobotSolver`: `get_body_id()`, `get_body_name()`, `get_joint_index()` 名称查找 API
  - `plotting.py`: `SimulationRecorder`（记录 joint q/qd, body positions）
  - `plot_joint_trajectory()`: 关节位置+速度子图
  - `plot_body_trajectory()`: 任意轴身体轨迹
  - `print_joint_summary_table()`: 统计摘要
- **文件**: `robot_solver.py`, `plotting.py`（158 行）

### 14. `0e8eae4` — 碰撞形状体系（ROADMAP 3.3-3.5）
- **改动**:
  - `CollisionShapeDesc`: box/sphere/capsule/cylinder/mesh + friction/restitution
  - `CollisionGroupConfig`: 位掩码分组, self-collision disable, link-pair 覆盖
  - `CollisionShapeLibrary`: 便捷工厂方法
  - 预定义分组：TERRAIN, ROBOT_LINK, GRIPPER, OBJECT
- **文件**: `collision_shapes.py`（117 行）
- **完成度**: scaffold。当前仅是 Python descriptor/library，尚未接入 C++ collision buffer、LBVH/narrow phase、per-shape friction 或 collision mask。`should_collide()` 目前只检查 `group_a & group_b`，还没有使用每个 shape 的 `collision_mask`。

---

## Phase 3-7：资产下载 + 全管线测试

### 15. `26b6c54` — 下载机器人资产
- **改动**:
  - 从 `newton-physics/newton-assets` sparse clone: unitree_h1, unitree_g1, franka_emika_panda
  - 从 `google-deepmind/mujoco_menagerie` sparse clone: universal_robots_ur10e/ur5e, anybotics_anymal_b/c, wonik_allegro, shadow_hand, franka_emika_panda
  - 总计 666 文件, 233 mesh (STL/OBJ), 6 ONNX policy, 19 URDF
  - 存储于 `PythonBindings/robotics/assets/`（已在 `.gitignore` 中）
- **文件**: 667 个新文件
- **审查意见**: 不建议把第三方机器人资产直接提交进主代码历史。该提交实际包含大量 OBJ/STL/USD/PNG/ONNX 二进制或大文本 mesh，`git show --stat` 显示 667 files / 4,421,118 insertions。应改为下载脚本、manifest、license 记录和 `.gitignore` 下的缓存目录，避免仓库膨胀和许可证风险。

### 16. `5bdf389` — Mass/inertia + Viewer（ROADMAP B.1, B.6）
- **改动**:
  - `RobotSolver.add_rigid_body()`: 新增 `mass`, `com`, `density` 覆盖参数
  - `RobotViewer`: `set_body_color()`, `highlight_joint()`, `get_body_poses()`, `print_joint_summary()`
- **文件**: `robot_solver.py`, `robot_viewer.py`

### 17. `3e6da22` — Mesh 近似 + 录制（ROADMAP B.3, B.4）
- **改动**:
  - `mesh_approx.py`: convex_hull, box/sphere/capsule 近似, quadric decimation
  - `recorder.py`: `FrameRecorder`（OBJ + JSON metadata）, `FrameReplay`（加载分析）
- **文件**: `mesh_approx.py`, `recorder.py`（168 行）

---

## Phase 4-7：机器人场景测试

### 18. `3202793` — H1 人形站立（ROADMAP 4.1-4.8）
- **改动**: 加载 25-link, 24-joint H1 URDF，用碰撞基元代替 mesh，joint drive 保持初始姿态
- **结果**: max drift 0.0016 rad, 所有 body 在地面以上
- **文件**: `test_robot_h1_stand.py`
- **完成度**: smoke test。该测试主要验证 joint drive 不发散，不等价于 Newton `robot_h1`：没有真实 floating base、mesh approximation 策略、严格速度阈值、接触调参或 batched `world_count=4`。

### 19. `acc14d3` — G1 人形站立（ROADMAP 4.9）
- **改动**: 加载 31-link, 30-joint G1 URDF（23-DOF 变体）
- **结果**: max drift 0.0002 rad, perfect stability
- **文件**: `test_robot_g1_stand.py`
- **完成度**: smoke test。当前断言较宽松，不能支撑“perfect stability”；应补充 root/base pose、body velocity、ground contact 和 world_count=4 验证。

### 20. `d2d9977` — ONNX Policy 推理（ROADMAP 5.7-5.9）
- **改动**:
  - 加载 G1 的 `mjw_g1_23DOF.onnx`（123-dim observation → 23-dim action）
  - 每 15 个 physics step 执行一次 policy inference
  - 将 inference 输出作为 joint target 施加到 solver
  - 回退模式：onnxruntime 不可用时纯 physics simulation
- **结果**: action mean abs 0.1842, max drift 0.0745 rad
- **文件**: `test_robot_policy.py`
- **完成度**: inference smoke test。observation 目前主要填 joint positions，没有实现 Newton policy 所需的 body-frame base velocity、projected gravity、command、previous action、joint error、action reorder 等字段；所以只能说明 ONNX session 和 target setter 可运行，不能说明 policy transfer 已完成。

### 21. `b25b44b` — ANYmal + Franka + Allegro（ROADMAP 4.10, 6, 7）
- **改动**:
  - ANYmal: 13-body quadruped（base + 4 legs × 3 links），12 revolute joints，base height 0.65m
  - Franka Panda: 从 URDF 加载 10-link 7-DOF 臂，drift 0.0000
  - Allegro Hand: 16-DOF 多指手（4 fingers × 4 links），16 revolute joints，drift 0.0000
- **结果**: 三个测试全部通过，joint drift 均 < 0.001 rad
- **文件**: `test_robot_anymal.py`, `test_robot_franka.py`, `test_robot_allegro.py`
- **完成度**: proxy scenes。ANYmal/Franka/Allegro 测试覆盖了多关节链不崩溃，但没有复现 Newton 的真实资产、floating base、standing/walking observation、gripper/contact、hand cube retention 或 Panda pick-and-place。

### 22. `4c750bd` — Drop 接触测试（ROADMAP 3.6-3.8）
- **改动**: Y-up 模式下 cube 从 1m 自由落体，落在地面 plane（`use_floor=True`）
- **结果**: 最终高度 0.075m（cube 半高），stable rest，无穿透
- **文件**: `test_drop_contact.py`

---

## 文件改动统计

| 类别 | 文件数 | 说明 |
|------|--------|------|
| C++ 核心 | 8 | solver_interface, newton_solver, joint_constraint_energy, simulation_data, scene_params, joint_constraint, init_sim_data, python_bindings |
| Python robotics | 16 | solver, mesh, render, training, parser, utils, robot_builder |
| Python 测试 | 9 | cartpole, arm, h1, g1, anymal, franka, allegro, policy, drop |
| 资产 | 667 | URDF, MJCF, STL, OBJ, ONNX |
| **总计** | **~700** | |

---

## 审查结论：当前 21 个提交是否合理

总体结论：方向合理，但完成度描述过于乐观；当前分支更像“robotics layer 原型 + 多个 smoke tests”，还不是 Newton README basic/robotic examples 的可对齐复现。建议先修正能力边界，再继续扩展。

### 合理的部分

| 方向 | 结论 | 原因 |
|------|------|------|
| 先做 Python robotics wrapper | 合理 | 不侵入现有 cloth/soft/tet/rigid core，符合 ROADMAP “先手工场景验证，再 importer 自动化”的策略。 |
| Z-up/gravity API | 合理 | Newton/MuJoCo/IsaacSim 都以 Z-up 为主，越早统一越好。 |
| joint q/qd/body pose API | 合理 | 这是 control、observation、debug 和 RL 环境的最低接口。 |
| joint drive energy | 合理但需验证 | 能快速做 position-control demos，但 penalty drive 对高 DOF chain 会有 drift 和刚度调参风险。 |
| URDF parser + RobotBuilder | 合理 | 可支撑 UR10/G1/H1/ANYmal 的初步加载；但 importer 仍需要 frame transform 和 inertial/collision 的严肃验证。 |
| 多个 headless smoke tests | 合理 | 可以防止接口级回归，适合作为后续 examples 的前置检查。 |

### 必须修改的部分

| 严重度 | 问题 | 修改建议 |
|--------|------|----------|
| 高 | `floor_normal` 只暴露未接入 core，Z-up `use_floor=True` 仍可能走 Y-up floor 逻辑 | 将 ROADMAP 0.3/A.4 标记为未完成；实现前不要用 Z-up floor 断言 Newton ground parity。下一步应把 ground energy、ground CCD、host evaluate 都改为 `dot(x - floor_origin, floor_normal)`。 |
| 高 | Ball/Free joint 只有 enum/energy 分支，没有完整初始化/API 创建路径 | 补齐 `SolverInterface::add_ball_joint`、`init_sim_data` buffer 初始化、bindings、RobotSolver/RobotBuilder 支持和 `basic_joints` 测试。 |
| 高 | 第三方资产直接入库 | 从主分支移除大资产提交，改为 `scripts/download_robot_assets.py` + manifest + license audit；只提交小型自制测试资产。 |
| 高 | “H1/G1/ANYmal/Policy 已完成”的表述不准确 | 改成 smoke/proxy tests；真正 Newton parity 需要 world_count、floating base、contact、velocity threshold 和 observation 对齐。 |
| 中 | `RobotBuilder` 声称 stdlib parser 零依赖，但 builder 运行时依赖 `scipy.spatial.transform.Rotation` | 用项目内 `physics_utils`/NumPy 实现 RPY→matrix，或把 scipy 写入依赖和 build docs。建议前者。 |
| 中 | `RobotSolver.get_joint_index()` 永远返回 `-1` | 建立 joint metadata（name、parent、child、type、index）并在 add_joint 时维护。 |
| 中 | `CollisionGroupConfig.should_collide()` 没用 shape-level mask | 改成 `shape_a.collision_group & shape_b.collision_mask` 和反向 mask 同时检查，并接入 C++。 |
| 中 | `RobotSolver.add_rigid_body(mass/com)` 没有真正设置 COM/inertia tensor | 在 C++ `WorldData`/initializer 中补显式 mass、COM、inertia 覆盖；否则文档中 B.1 只能算部分完成。 |
| 中 | ONNX policy observation 是占位向量 | 实现 `ArticulationView.compute_observation()`，按 Newton policy 字段逐项对齐后再评估 policy。 |
| 低 | 多个测试文件位于 `PythonBindings/robotics/` 而非 `PythonBindings/tests/` | 若要纳入 CI，建议迁移到 `PythonBindings/tests/robotics/` 或配置 pytest discovery。 |

### 建议的提交重组

| 当前提交 | 建议 |
|----------|------|
| `26b6c54` 资产下载 | 不应保留为普通代码提交；替换为下载脚本和 manifest。 |
| `18-21` 高级机器人 demo | 拆为 “proxy smoke test” 与 “Newton parity todo”，避免误导评审者以为阶段 4-7 已完成。 |
| `14` collision shape descriptors | 保留，但标题改为 `Add Python collision shape descriptors`，不要写成 collision shape pipeline 已完成。 |
| `11` Ball/Free | 保留，但追加 follow-up commit 补齐 API 和 init，否则不要把 `basic_joints` 标为 unblocked。 |

---

## 之后的任务：接入强化学习流程

当前平台已具备基本的机器人仿真能力（URDF 加载、关节约束/驱动、ONNX 推理闭环），但要成为跨平台高性能 RL 训练平台，还需以下工作：

**未来计划评估**:
- 当前计划的方向完整，但优先级需要前移“能力真实性验证”。如果 ground、Ball/Free、rigid-rigid contact、observation 仍是部分实现，直接做 Gym/API/性能优化会把错误接口固化。
- Newton README 对齐应拆成两条线：先跑通 `basic_pendulum`、`robot_cartpole`、`basic_joints` 这类小场景；再做 UR10、quadruped、H1/G1/ANYmal。
- RL 训练前最小闭环不是 Gym API，而是 batched articulation state/control + reset + observation parity + deterministic smoke tests。
- 所有 “standing/policy” 结果必须区分 proxy scene 与真实资产 parity，否则无法判断和 Newton 的差距来自 importer、joint、contact 还是 policy observation。

### 建议立即插入的修复阶段（Phase R）

| 编号 | 任务 | 验收标准 |
|------|------|----------|
| R.1 | ground plane 真正支持 Z-up/任意 normal | `test_drop_contact.py` 在 Z-up 下启用 `use_floor=True`，cube resting height 由 z 坐标验证；ground CCD 与 energy 都不再硬编码 `floor.y`。 |
| R.2 | Ball/Free joint 创建链路补齐 | `basic_joints` 可创建 revolute/prismatic/ball 三类 articulation；RobotBuilder 能处理 `floating/free` root。 |
| R.3 | 移除大资产提交，改为下载脚本 | 仓库不包含第三方大 mesh/ONNX；脚本支持 sparse download、checksum/manifest、license 记录。 |
| R.4 | RobotBuilder 去除 scipy 依赖或声明依赖 | 已实施最小修复：`robot_builder.py` 改为导入 `URDFParser`，并用 NumPy 实现 RPY→matrix，避免 clean venv 直接缺 scipy 失败。仍需补最小 URDF build smoke test。 |
| R.5 | joint/body metadata | `get_joint_index(name)`、joint name list、body name list 可用；后续 action reorder 不再靠数组猜测。 |
| R.6 | proxy/parity 测试分层 | 文件命名和断言区分 `test_robot_*_proxy.py` 与 `test_newton_*_parity.py`。 |

### 一、仿真性能与规模

| 编号 | 任务 | 说明 |
|------|------|------|
| P.1 | **批量环境复制** | `scene.replicate(robot, world_count=N)` — C++ 端将同一 robot model 复制 N 份，通过 world offset 空间隔离。这是 RL 训练的前提（需要数千并行环境） |
| P.2 | **World isolation** | 确保不同 world 之间不碰撞（collision mask 或空间分离），避免跨环境物理交互 |
| P.3 | **GPU 性能优化** | 分析 step time，优化 shader dispatch、减少 CPU-GPU 同步点、合并 kernel launch |
| P.4 | **VRAM 监控** | batched environments（100+ worlds）对显存需求极高，需要监控和优化 buffer 分配 |
| P.5 | **跨后端验证** | 当前主要在 Metal (macOS) 测试，需要验证 CUDA/Vulkan/DirectX 后端的一致性和性能 |
| P.6 | **Headless 渲染优化** | RL 训练不需要渲染，headless 模式应跳过所有可视化相关 buffer 分配和 shader 编译 |

### 二、观测与控制接口

| 编号 | 任务 | 说明 |
|------|------|------|
| O.1 | **ArticulationView** | 封装 robot model + world_count，提供批量接口 `get_joint_q()` → `[W, dof]`, `set_joint_target_pos()` → `[W, dof]` |
| O.2 | **标准观测 buffer** | `compute_observation()`：base lin/ang vel（body frame）、projected gravity、commanded vel、joint pos error、joint vel、previous action |
| O.3 | **Body-frame 速度** | `quat_rotate_inv(q_base, lin_vel_world)` — 从 affine DOF 提取 rotation quaternion 并转换速度到 body frame |
| O.4 | **Projected gravity** | `quat_rotate_inv(q_base, gravity_direction)` — 重力方向在 body frame 中的投影，用于 policy 感知姿态 |
| O.5 | **Action scaling & reordering** | 支持 `action_scale`（将 policy 输出映射到 joint target）、`joint name mapping`（lab ↔ mujoco 重排） |
| O.6 | **World reset** | `reset_worlds(indices)` — 将指定 world 的 `joint_q`/`joint_qd` 恢复到初始值，支持 domain randomization |
| O.7 | **Termination detection** | 检测 base height < threshold、joint limit violation、body contact with ground 等终止条件 |

### 三、物理精度与稳定性

| 编号 | 任务 | 说明 |
|------|------|------|
| A.1 | **Hard constraint joints** | 当前 joint 是 penalty constraint，多体链可能 drift。需要实现 Lagrange multiplier 或 augmented Lagrangian 的 hard constraint |
| A.2 | **Joint armature** | 给 joint DOF 加假惯性以提高数值稳定性，特别是在高刚度驱动下 |
| A.3 | **Joint effort limit** | 从 URDF 读取 motor effort limits，clamp joint forces |
| A.4 | **Ground plane shader 升级** | 将 ground collision 从 hardcoded Y-up 改为可配置 `floor_normal + floor_height` 的任意平面 |
| A.5 | **Rigid-rigid contact** | 当前碰撞管道仅支持 soft-body（cloth/tet）接触。需要扩展 LBVH/narrow phase 支持 rigid-rigid 碰撞 |
| A.6 | **Contact friction 调优** | 系统化调参：`stiffness_collision`、`damping_rate`、friction model，特别是足式机器人的足端接触 |
| A.7 | **Policy transfer 验证** | 对比 Newton/MuJoCo 输出的 observation vector，逐字段统计误差，评估 sim-to-real gap |

### 四、平台与生态

| 编号 | 任务 | 说明 |
|------|------|------|
| E.1 | **Gymnasium API** | 实现标准 `gym.Env` 接口（`reset()`, `step()`, `render()`），可直接接入 Stable-Baselines3、RLlib 等框架 |
| E.2 | **并行环境向量化** | `AsyncVectorEnv` 或 `SubprocVecEnv` 包装，支持多进程数据收集 |
| E.3 | **CUDA 后端验证** | 在 Linux + NVIDIA GPU 上验证完整管线（shader 编译、计算精度、性能） |
| E.4 | **Vulkan 后端验证** | Linux/Windows 上的 Vulkan 后端验证，特别是 shader 兼容性 |
| E.5 | **CI/CD 流水线** | 自动化测试：每个 PR 运行所有 robot test（headless mode）+ 性能回归检测 |
| E.6 | **Docker 镜像** | 提供预配置的 Docker 镜像（CUDA + onnxruntime + Python deps），便于云端训练 |
| E.7 | **文档与教程** | 完善 `Document/PythonAPI.md`、`Document/RobotAPI.md`，添加 Jupyter notebook 教程 |

### 五、高级特性

| 编号 | 任务 | 说明 |
|------|------|------|
| X.1 | **USD stage 导入** | 支持 USD 格式场景导入（IsaacSim 标准格式），便于与 NVIDIA Omniverse 生态互通 |
| X.2 | **Domain randomization** | 物理参数随机化（mass, friction, joint stiffness, gravity），提高 policy 鲁棒性 |
| X.3 | **Teacher-student 训练** | 用高精度 solver（更多 substep/iteration）作为 teacher，低精度作为 student，加速训练 |
| X.4 | **GPU graph capture** | 参考 Newton 的 `wp.ScopedCapture()`，用 CUDA graph 消除 kernel launch overhead |
| X.5 | **分布式训练** | 多 GPU 或多节点训练支持，每个 GPU 独立运行一组环境 |
| X.6 | **实时 rendering + training** | 训练过程中偶尔渲染，用于调试 policy 行为（类似 IsaacSim 的 play mode） |

### 优先级建议

1. **R.1-R.6（修复阶段）** — 先修正 ground、Ball/Free、资产、metadata 和测试分层，否则后续 parity 判断不可靠。
2. **O.1 + P.1-P.2（ArticulationView + 批量环境 + world isolation）** — RL 训练的结构前提；建议先支持 4/16/100 worlds，而不是直接追求数千环境。
3. **O.2-O.7（观测接口）** — 与 policy 对接的必要组件；必须逐字段和 Newton/MuJoCo 打印对齐。
4. **A.2-A.4（armature + effort limit + ground plane shader）** — 比 hard constraint 更短路径，能提升 UR10/standing/policy smoke 的可信度。
5. **A.5-A.6（rigid-rigid contact + friction）** — 足式机器人 standing/walking 的硬依赖，应在 H1/G1/ANYmal parity 前完成。
6. **E.1-E.2（Gym API）** — 等 reset/observation/action/termination 稳定后再包装标准 RL 框架接口。
7. **A.1（hard constraint joints）** — 作为稳定性升级单独推进；不要阻塞早期 proxy/parity examples，但要为高 DOF hand/legged robots 保留路线。
8. **P.3-P.6（性能优化）** — 扩展到 100+ 环境时必需；在 correctness 未稳定前不要过早合并 kernel 或隐藏同步点。
9. **E.3-E.7（跨平台 + CI + 文档）** — 生产化。
10. **X.1-X.6（高级特性）** — 竞争力提升，放在 Newton README examples 基本对齐之后。

### Newton README 对齐顺序建议

| 顺序 | Example | 依赖 | 当前可做程度 |
|------|---------|------|--------------|
| 1 | `basic_pendulum` | revolute + Z-up + ground 可关闭 | 可立即做 proxy，若要 ground parity 需 R.1。 |
| 2 | `robot_cartpole` | prismatic/revolute + joint q/qd + batch | 单 world proxy 已接近；100 worlds 需 O.1/P.1。 |
| 3 | `basic_joints` | Ball joint + prismatic limit | 被 R.2 阻塞。 |
| 4 | `robot_ur10` | importer + joint drive + batched target | 需要 RobotBuilder transform/inertia 修正和 ArticulationView。 |
| 5 | `basic_shapes` | primitive collision shapes + rigid-rigid contact + Z-up floor | 被 A.5/R.1 阻塞。 |
| 6 | `basic_urdf` / quadruped standing | floating base + contact + batch | 被 R.2/A.5/O.1 阻塞。 |
| 7 | H1/G1/ANYmal standing | importer + floating base + contact tuning + velocity thresholds | 当前只有 proxy smoke，不能算 parity。 |
| 8 | `robot_policy` / `anymal_c_walk` | observation parity + action reorder + reset + stable contact | 当前只有 ONNX smoke。 |
| 9 | Allegro/Panda | high DOF drive + grasp contact + IK/gripper | 后期目标。 |
