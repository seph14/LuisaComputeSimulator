# LuisaComputeSimulator Robot Simulation Roadmap

本文档基于 LuisaComputeSimulator 当前 pipeline 能力与 Newton (https://github.com/newton-physics/newton) 的差异分析，规划分阶段实现机器人训练平台的建设目标。

---

## 总体策略

- **复用现有 NewtonSolver 作为底层物理求解器**：不改动 cloth/soft/tet/rigid 的求解核心，在现有 rigid affine-body dynamics、joint constraint、collision pipeline 之上叠加 robotics layer。
- **坐标系统一为 Z-up**：当前 `SceneParams.gravity = {0, -9.8, 0}` 是 Y-up，机器人社区（Newton、MuJoCo、IsaacSim）均为 Z-up。必须在阶段 1 就统一为 Z-up，否则所有 imported robot 姿态都会错。
- **所有关节统一为 Z-up**：在现有 fixed/prismatic/revolute 上扩展 ball joint、free joint。
- **先手工构造场景验证核心能力，再做 importer 自动化**：Cartpole 手工建场景 → 验证 joint stability → 做 URDF importer → 做批量 env → 做 RL policy。

---

## 阶段 0：基础设施准备（1-2 周）

### 目标
确保现有 pipeline 在 Z-up 坐标系下能正确运转，为后续所有 robot demo 做好准备。

### 构建任务

| 编号 | 任务 | 说明 |
|------|------|------|
| 0.1 | 支持 Z-up 坐标系 | `SceneParams` 新增 `up_axis` 枚举（Y_UP / Z_UP）；gravity/floor 根据 up_axis 自动推导默认值。Z-up 时默认 `gravity = (0, 0, -9.8)`。|
| 0.2 | Python API 支持设置 gravity | `config.gravity = np.array([0.0, 0.0, -9.8], dtype=np.float32)` 或 `config.set_up_axis("Z")`。 |
| 0.3 | 地面高度可配置 | 当前 floor 逻辑仅以 `floor{0,0,0}` 隐式区分，需支持地面方程 (`plane_normal`, `plane_origin`) 或高度值。 |
| 0.4 | 验证现有 rigid joint demos 在 Z-up 下通过 | 修改 `test_rigid_joint_animation.py`，添加 Z-up 模式，确保 fixed/prismatic/revolute 测试全部通过。 |
| 0.5 | 确立 Newton-like step cycle 模板 | 在 Python 端实现 `simulate()` / `step()` / `render()` 模板函数，后续所有 demo 复用。 |

### 预期产出
- Z-up 下的 `test_rigid_joint_animation.py` 全部 pass。
- 一个 `utils/robot_sim_runner.py` 提供标准 step cycle 模板。

---

## 阶段 1：Cartpole Demo + Joint State API（2-3 周）

### 目标
手工构建 Cartpole 场景，验证当前 rigid affine-body + joint penalty constraint 是否足以支撑多体 articulation，并补全 joint state query/control API。

### 构建任务

| 编号 | 任务 | 说明 |
|------|------|------|
| 1.1 | 手工创建 Cartpole 场景 | 3 个刚体（cart + pole1 + pole2），cart-world prismatic joint，pole revolute joints，不依赖 URDF。Python 脚本 `test_robot_cartpole.py`。|
| 1.2 | 实现 `get_joint_q` / `get_joint_qd` | 从 affine-body DOF 计算 prismatic slide distance 和 revolute angle，返回 `np.ndarray`。 |
| 1.3 | 实现 revolute joint angle limit | `JointConstraintDesc` 增加 `lower_angle` / `upper_angle`，energy 中添加 limit penalty。 |
| 1.4 | 实现 joint drive energy | `RevoluteJointDriveEnergy` 和 `PrismaticJointDriveEnergy`：`E_drive = 0.5 * kp * (q - q_target)^2`。 |
| 1.5 | 实现 `set_joint_target_pos` | Python API 可设置每个 joint 的目标角度/位移。 |
| 1.6 | `get_body_pose` / `get_body_velocity` | 从 affine DOF 提取 world-frame body translation/rotation/velocity。 |
| 1.7 | Cartpole 数值测试 | cart 沿指定轴运动，pole 主要绕 revolute axis 旋转，z 方向远动 ≤ 1e-4。 |
| 1.8 | 调优 C++  solver 参数 | 调整 `implicit_dt`、`num_substep`、`nonlinear_iter_count`、joint stiffness 以保证稳定性。 |

### 预期产出
- `PythonBindings/tests/test_robot_cartpole.py` headless 测试通过。
- `world_count=1` 的 cartpole 物理行为正确。
- 新增 Python API 可用清单在 `Document/PythonAPI.md` 中更新。

---

## 阶段 2：URDF Importer + UR10 Position Control（3-5 周）

### 目标
实现 URDF 导入，ROBOT 自动化场景构建。复现 Newton `example_robot_ur10.py` 的关节目标跟踪行为。

### 构建任务

| 编号 | 任务 | 说明 |
|------|------|------|
| 2.1 | URDF parser（Python 端） | 使用 `yourdfpy` 或手写解析，提取 links、joints、visual/collision mesh paths、origin transforms、axis、limits、inertial 参数。 |
| 2.2 | `RobotBuilder` C++ 数据结构 | `Solver/SimulationCore/robot_model.h`：link descriptors（mass, com, inertia, collision shapes, visual shapes, parent-child tree）、joint descriptors（type, axis, anchor, limits, drive params）。 |
| 2.3 | URDF → RobotBuilder 转换 | Python 端将 URDF parse 结果映射到 `RobotBuilder` API，注册 rigid-body `WorldData` 和 joint constraints。 |
| 2.4 | 支持 `collapse_fixed_joints` | 固定关节直接合并 link geometry，减少关节数量。先做简单版：不 collapse 但 fixed joints 使用高强度 penalty。 |
| 2.5 | 多刚体 joint 串创建 | `build_articulation(joint_chain)`，自动按 parent-child tree 创建 rigid bodies + joints，处理 joint parent/child transform。 |
| 2.6 | ball/spherical joint | 新增 `BallJointConstraintDesc` 和 `BallJointConstraintEnergy`，约束两点重合但允许任意相对旋转。 |
| 2.7 | free joint / floating base | 新增 `FreeJointConstraintDesc`（本质上是无约束），用于 floating base 机器人。 |
| 2.8 | UR10 场景测试 | 加载 Newton 的 `universal_robots_ur10` asset，设置正弦 joint trajectory，验证 arm 不发散、关节角跟踪目标。 |
| 2.9 | joint target 轨迹精度测试 | 比较 target trajectory 和实际 joint q 的 MSE < 0.1 rad。 |

### 预期产出
- `PythonBindings/tests/test_robot_ur10.py` 可运行。
- `RobotBuilder` C++ 层和 Python wrapper。
- `Document/RobotAPI.md` 初稿。

---

## 阶段 3：Collision Shape 体系 + Ground Contact Stability（2-3 周）

### 目标
完善碰撞几何体系，支持地面接触稳定，使机器人能稳定站立。

### 构建任务

| 编号 | 任务 | 说明 |
|------|------|------|
| 3.1 | primitive collision shapes | 新增 box/sphere/capsule/cylinder 碰撞几何的初始化流程，生成 proxy mesh 用于现有 triangle-mesh LBVH/narrow phase。 |
| 3.2 | convex hull approximation | 对 imported visual mesh 做 convex hull decomposition 或直接用单 hull 作为碰撞体。 |
| 3.3 | per-link collision shape 支持 | `RobotLinkDesc` 支持一个 link 绑定多个 collision shapes。 |
| 3.4 | collision group/filter | 支持 shape-level collision group mask，可配置 disable self-collision within robot。 |
| 3.5 | per-shape friction/restitution | `CollisionShapeDesc` 中可设 `mu` / `restitution`。 |
| 3.6 | ground plane primitive | 显式 ground plane（无限平面）碰撞，不使用 heightfield 但平面碰撞对稳定性更好。 |
| 3.7 | contact stability tuning | 系统化调参：`stiffness_collision`、`damping_rate`、`implicit_dt`、`num_substep`、`nonlinear_iter_count`。 |
| 3.8 | `use_floor` 增强 | 支持地面法线方向和高度，不再硬编码 `floor{0,0,0}` + Y 为下。 |

### 预期产出
- box/sphere/capsule/cylinder 均可用于碰撞。
- `use_floor` 支持 Z-up 地面平面。
- 一个简单的 rigid body stacking 或 drop test 通过。

---

## 阶段 4：H1/G1/ANYmal Standing Demos + Batched Environments（4-6 周）

### 目标
实现 Newton `example_robot_h1.py`、`example_robot_g1.py`、`example_robot_anymal_d.py` 的简化版（只做 standing，不做 walking）。

### 构建任务

| 编号 | 任务 | 说明 |
|------|------|------|
| 4.1 | floating base robot import | URDF importer 支持 `floating=True`，创建 free joint 作为 root。 |
| 4.2 | joint position drive 全链路 | 设置所有 joint target 为初始值，让 robot 停留在初始姿态。 |
| 4.3 | `joint_target_ke` / `joint_target_kd` | 从 URDF/YAML 读取 joint drive 刚度/阻尼。 |
| 4.4 | `joint_effort_limit` | 从 URDF 读取 motor effort limits，力上限。 |
| 4.5 | `joint_armature` | 给 joint DOF 加假惯性以提高数值稳定性。 |
| 4.6 | batched environment replication | `scene.replicate(robot, world_count=N, spacing=...)`：C++ 端将同一 robot model 复制 N 份，world offset 通过 translation。 |
| 4.7 | world isolation | 确保不同 world 之间不碰撞（通过 collision mask 或 spatial separation）。 |
| 4.8 | H1 standing test | `world_count=4`，所有 bodies z > 0，所有 body velocities < 0.015。 |
| 4.9 | G1 standing test | 同 H1 的测试标准。 |
| 4.10 | ANYmal D standing test | 四足 holding initial posture，check base height ~0.68m，all bodies above ground。 |

### 预期产出
- `PythonBindings/tests/test_robot_h1_standing.py`
- `PythonBindings/tests/test_robot_g1_standing.py`
- `PythonBindings/tests/test_robot_anymal_d.py`
- batched env API 初步可用。

---

## 阶段 5：Observation & Control + ONNX Policy Loop（4-8 周）

### 目标
实现 Newton `example_robot_policy.py` 和 `example_robot_anymal_c_walk.py` 的推理闭环。

### 构建任务

| 编号 | 任务 | 说明 |
|------|------|------|
| 5.1 | `ArticulationView` 类 | 封装 robot model + world_count，提供 `get_joint_q()`、`get_joint_qd()`、`set_joint_target_pos()` 等批量接口，返回 shape `[world_count, dof]` 的 numpy 数组。 |
| 5.2 | observation buffer | `compute_observation()`：base linear vel (body frame)、base angular vel (body frame)、projected gravity、commanded vel、joint pos error、joint vel、previous action。 |
| 5.3 | base velocity in body frame | `quat_rotate_inv(q_base, lin_vel_world)`，需从 affine body DOF 提取 rotation quaternion。 |
| 5.4 | projected gravity | `quat_rotate_inv(q_base, gravity_direction)`。 |
| 5.5 | action scaling & reordering | 支持 `action_scale`、`joint name mapping`（如 lab_to_mujoco reorder）。 |
| 5.6 | `reset_worlds(indices)` | 将指定 world 的 `joint_q` / `joint_qd` 恢复到初始值。 |
| 5.7 | ONNX Runtime 集成 | Python 端用 `onnxruntime` 加载 policy，CPU 推理。后期可接 GPU。 |
| 5.8 | policy step pipeline | 每 N 个 physics substep 执行一次 policy inference。 |
| 5.9 | ANYmal C walk test | 验证 walking policy inference 不崩溃，base 沿 forward 方向运动。 |
| 5.10 | policy transfer 验证工具 | 逐字段打印 observation 与 Newton 对比，统计误差。 |

### 预期产出
- `PythonBindings/tests/test_robot_policy.py`，支持 `--robot` 参数选择 robot type。
- observation / action 接口稳定。

---

## 阶段 6：Allegro Hand + Deformable Fingertip（4-6 周）

### 目标
实现 Newton `example_robot_allegro_hand.py`，展示多关节灵巧手。

### 构建任务

| 编号 | 任务 | 说明 |
|------|------|------|
| 6.1 | 大量 revolute joint 稳定性 | 20+ revolute joints 在 penalty constraint 下不振荡。可能需要增加 joint damping。 |
| 6.2 | joint parent transform 动态更新 | `SolverNotifyFlags.JOINT_PROPERTIES` 等价物，允许运行时修改 joint anchor frame。 |
| 6.3 | 立方体抓取 | hand holding cube，验证物体不掉落。 |
| 6.4 | hand demo | `world_count=16`，验证 50%+ 立方体仍被抓在手中。 |

### 预期产出
- `PythonBindings/tests/test_robot_allegro_hand.py`

---

## 阶段 7：Panda Manipulation（后期，6-10 周）

### 目标
实现 Newton `example_robot_panda_hydro.py` 的简化版（不做 hydroelastic contact，用普通接触替代）。

### 构建任务

| 编号 | 任务 | 说明 |
|------|------|------|
| 7.1 | IK solver | 解析逆运动学求解 Franka arm 的目标关节角。 |
| 7.2 | gripper drive | 手指/夹爪的 joint target position 控制。 |
| 7.3 | pick-and-place 闭环 | grasp object → lift → move to target → release → return。 |
| 7.4 | multi-body contact stability | arm + gripper + object + table 的多体接触稳定。 |

### 预期产出
- `PythonBindings/tests/test_robot_panda.py`

---

## 其他基础能力（穿插在各阶段）

| 编号 | 能力 | 说明 |
|------|------|------|
| B.1 | joint mass/inertia 接口 | `WorldData` 支持显式设置 mass、COM、inertia tensor（覆盖 mesh-computed 值）。 |
| B.2 | body label/name 查询 | 按名称查找 body/joint index。 |
| B.3 | mesh approximate | box approximation、convex hull decomposition。 |
| B.4 | 录制/回放 | 仿照 Newton recording/replay viewer，保存和回放仿真帧。 |
| B.5 | plotting | 物理量绘制（joint trajectory, energy, contact count 等）。 |
| B.6 | viewer integration | 与 Polyscope 集成显示 robot state，支持 joint highlight、contact visualization。 |
| B.7 | USD importer（可选） | 后期支持 USD stage 导入，但 URDF 优先。 |
| B.8 | GPU graph capture（可选） | 参考 Newton 的 `wp.ScopedCapture()`，用于 GPU 端仿真加速。 |

---

## 关键风险

1. **Penalty joint stability**：当前 joint 是 penalty constraint 而非 hard constraint，多体链可能产生 drift。需要在 joint drive stiffness/damping 和 Newton iterations 之间仔细平衡。
2. **Foot contact fidelity**：足式机器人的足端接触对 step size、contact stiffness、friction 极度敏感。可能需要在 robot 场景中关闭 `use_self_collision` 并只依赖地面碰撞。
3. **Policy transfer**：即使 API 对齐，训练在 Newton/MuJoCo 动力学下的 policy 在你的 solver 上也不保证稳定。需要先将 observation vector 逐字段对齐，再逐步训练新 policy。
4. **坐标系混淆**：大量内部代码可能隐式依赖 Y-up。所有 mesh transform、gravity、animation target 都要审计。
5. **VRAM 和性能**：batched environments（100+ worlds）对内存和计算需求很高，需要监控 GPU 显存和 step time。

---

## 里程碑检查点

| 里程碑 | 标志 |
|--------|------|
| M1 | Z-up 下 cartpole 物理正确 |
| M2 | URDF quadruped 可以从文件加载并站立在地面上 |
| M3 | 100-world cartpole batch 正确运行 |
| M4 | H1/G1 standing，所有 bodies above ground |
| M5 | ANYmal ONNX policy inference 不崩溃 |
| M6 | Allegro hand 50%+ cubes held |
| M7 | Panda pick-and-place 完成一次完整抓放循环 |
