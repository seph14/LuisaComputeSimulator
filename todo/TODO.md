# TODO: Newton Robot/RL Parity Execution Plan

本文档用于把当前 LCS robotics 原型推进到可对齐 Newton robot examples 与强化学习场景的实现路线。后续 agent 可以按阶段或任务块并行执行。

## 当前结论

- 当前 `PythonBindings/robotics/` 已具备 `RobotSolver` wrapper、URDF parser、RobotBuilder、joint query、joint target drive、Ball/Free joint、H1/G1/ANYmal/policy smoke tests。
- 当前实现更准确地说是 robotics smoke/proxy layer，不是 Newton `robot_h1`、`robot_policy` 或 `robot_anymal_c_walk` 的 1:1 parity。
- 真正运行 RL 场景前，必须优先补齐 batched articulation、world replication、reset、observation/action mapping、floating-base/contact parity。

## 状态分层约定

后续更新 `todo/SCENE_EXAMPLES.md` 和测试结果时，统一使用以下状态：

| 状态 | 含义 |
|------|------|
| `not_started` | 尚无实现 |
| `proxy` | 用手工或简化资产近似场景，验证接口和数值不崩 |
| `smoke` | 使用部分真实资产或真实 policy，但断言宽松，只验证流程跑通 |
| `parity_in_progress` | 资产、参数、assertion 正在对齐 Newton |
| `parity_passed` | 与 Newton example 的核心配置和测试断言对齐通过 |

## P0: 文档与测试分层修正

目标：避免其他 agent 把 smoke/proxy 当作 parity，先建立清晰任务边界。

### P0.1 更新 `todo/SCENE_EXAMPLES.md`

- 给每个 example 添加状态字段：`not_started/proxy/smoke/parity_in_progress/parity_passed`。
- 给每个 example 添加缺口字段：`asset/importer/batch/floating_base/contact/observation/assertion`。
- 将 H1/G1/ANYmal/policy 当前状态标为 `smoke` 或 `proxy`，不要标为已完成 parity。
- 同步 `todo/DONE.md` 和 `todo/COMMIT_LOG.md` 中关于 H1/G1/ANYmal/policy 的审查结论。

验收标准：

- `SCENE_EXAMPLES.md` 能直接回答每个 example 是 proxy、smoke 还是 parity。
- 每个未 parity 的 example 都列出阻塞组件。

### P0.2 增加 parity 验收列

- 为每个 example 增加 Newton 原始断言摘要。
- 为每个 LCS 脚本增加当前断言摘要。
- 标出断言差距，例如 H1 缺 `all body velocities < 5e-3` 和 `world_count=4`。

验收标准：

- 任何 agent 可以从文档判断一个 test 是否能代表 Newton parity。

## P1: 小场景 Parity 基线

目标：先用小场景证明 joint、Z-up、ground、query API 是可靠的，再推进 H1/RL。

### P1.1 实现 `basic_pendulum` parity

- 新增脚本建议路径：`PythonBindings/robotics/test_basic_pendulum.py`。
- 构建 2 个 rigid box link 和 1 个 fixed anchor。
- 使用 2 个 revolute joints。
- Z-up：gravity `(0, 0, -9.8)`，地面可先关闭；如开启 ground，必须验证 Z-up floor。
- 对齐 Newton 断言：link 在摆平面内运动，速度不发散。

验收标准：

- headless 模式可运行。
- 最终 link 位置和速度满足 `todo/SCENE_EXAMPLES.md` 中 B01 断言。

### P1.2 完成 `robot_cartpole` parity 第一阶段

- 当前 `PythonBindings/robotics/test_cartpole.py` 是 single-world proxy。
- 对齐 Newton 的 cart/pole 运动方向、初始 joint q、velocity assertions。
- 保留手工构造路线，不阻塞于 USD importer。

验收标准：

- single-world 版本通过 Newton 风格断言。
- 明确记录与 Newton `world_count=100` 的剩余差距。

### P1.3 实现 `basic_joints` parity

- 新增脚本建议路径：`PythonBindings/robotics/test_basic_joints.py`。
- 构建 revolute、prismatic、ball 三组独立 articulation。
- 使用 Ball joint 验证 anchor distance。
- 使用 prismatic limit 验证 slider 范围。

验收标准：

- Ball anchor distance 维持在目标容差内。
- Prismatic slide 不越界。
- Revolute 行保持平面内运动。

## P2: Batched Articulation 与 World Replication

目标：这是 Newton `world_count` 和 RL 训练的结构前提。

### P2.1 实现 Robot/World metadata

- 为 `RobotSolver` 或新 `RobotModel` 维护 body names、joint names、parent/child、joint type、joint index、world id。
- 不再依赖注册顺序猜测 action/joint mapping。

验收标准：

- `get_body_id(name)`、`get_joint_index(name)`、`get_joint_name(idx)` 对真实 URDF 和手工场景都稳定。
- 支持同一 robot 被复制后通过 `(world_id, body_name)` 查找 body。

### P2.2 实现 `replicate(robot, world_count, spacing)`

- 将同一 robot model 复制到多个 world。
- 每个 world 有空间 offset。
- 后续应接入 collision group/mask 避免跨 world 碰撞。

验收标准：

- `robot_cartpole --world-count 4` 初步可跑。
- 所有 world 的同名 joint/body 查询返回正确 shape。

### P2.3 实现 `ArticulationView`

- 建议路径：`PythonBindings/robotics/solver/articulation_view.py`。
- 提供 `get_joint_q()`、`get_joint_qd()`、`set_joint_target_pos()`、`get_body_pose()`、`get_body_velocity()`。
- 返回 shape 应为 `[world_count, dof]` 或 `[world_count, body_count, ...]`。

验收标准：

- `robot_cartpole --world-count 100` 可验证所有 worlds 运动一致。
- 可以批量设置 joint target。

### P2.4 实现 `reset_worlds(indices)`

- 保存初始 body/joint state。
- 允许按 world id 恢复 q、qd、body pose、velocity。

验收标准：

- 连续运行多个 episode，不需要重建 solver。
- reset 后 observation 与初始状态一致。

## P3: Importer 与物理参数补齐

目标：让 URDF/robot scene 能接近 Newton/MuJoCo 的物理配置。

### P3.1 修正 URDF transform pipeline

- 正确处理 link transform、joint origin、collision origin、visual origin、RPY、mesh scale。
- 使用 `PythonBindings/robotics/tools/compare_urdf_parser.py` 与 reference parser 对比。

验收标准：

- H1/G1/Franka/ANYmal 的 link/joint tree 与 reference parser 一致。
- 抽样 link world transform 与 reference 误差在可解释范围内。

### P3.2 接入 mass/COM/inertia override

- 当前 `RobotSolver.add_rigid_body(mass/com/density)` 仍偏 wrapper 层，不代表完整 inertia tensor 对齐。
- 需要在 C++ `WorldData`/initializer 中支持显式 mass、COM、inertia tensor。

验收标准：

- 可从 URDF inertial 加载 mass、COM、inertia。
- 可查询 body mass/inertia，结果与 URDF 一致。

### P3.3 接入 joint physical params

- 支持 joint limits、friction、armature、effort limit、target mode。
- Newton H1/G1/ANYmal 依赖这些参数保持稳定。

验收标准：

- H1 可设置 `target_ke=150`、`target_kd=5`、`limit_ke=1e3`、`limit_kd=1e1`、`friction=1e-5`。
- UR10 可设置 position target mode 和 effort/drive 参数。

### P3.4 接入 primitive collision shape pipeline

- 当前 `collision_shapes.py` 是 Python descriptor，尚未完整接入 C++ contact pipeline。
- 需要支持 box/sphere/capsule/cylinder/cone/mesh 的 shape metadata、friction、collision group/mask。

验收标准：

- `basic_shapes` 可以用 primitive shape 构建并验证 resting height。
- collision group 可用于 self-collision disable 和 world isolation。

## P4: H1/G1/ANYmal Standing Parity

目标：把当前 humanoid/quadruped smoke tests 升级为 Newton-style parity tests。

### P4.1 H1 parity test

- 当前脚本：`PythonBindings/robotics/test_robot_h1_stand.py`。
- 当前状态：smoke/proxy。
- 缺口：USD importer、真实 mesh bounding box approximation、floating base、`world_count=4`、contact tuning、body velocity assertion。

验收标准：

- 所有 bodies z > 0。
- 所有 body velocities < 5e-3，若初期达不到，需要记录 staged threshold。
- 支持 `world_count=4`。

### P4.2 G1 parity test

- 当前脚本：`PythonBindings/robotics/test_robot_g1_stand.py`。
- 缺口类似 H1，但 G1 有 23/29 DOF 变体和 policy 需求。

验收标准：

- 所有 bodies z > 0。
- 所有 body velocities < 0.015 或记录 staged threshold。
- 支持 `world_count=4`。

### P4.3 ANYmal standing parity

- 当前脚本：`PythonBindings/robotics/test_robot_anymal.py` 是 procedural proxy。
- 需要真实 ANYmal asset、floating base、12 revolute joints、contact 稳定性。

验收标准：

- base height 接近 Newton 配置。
- 足端接触稳定。
- 支持 `world_count=16` 的 staged 版本。

### P4.4 Contact tuning sweep

- 系统扫描 substep、nonlinear iterations、PCG iterations、collision stiffness、damping、friction。
- 对 H1/G1/ANYmal 分别记录稳定参数。

验收标准：

- 每个 standing demo 有一组可重复的稳定参数。
- 文档记录失败模式：drift、foot penetration、explosion、slow convergence。

## P5: RL Policy Pipeline

目标：从 ONNX smoke 走向可运行 Newton policy 场景。

### P5.1 Observation builder

- 实现 base linear/angular velocity in body frame。
- 实现 projected gravity。
- 实现 command velocity。
- 实现 joint position error、joint velocity、previous action。
- 支持 observation shape `12 + 3 * num_dofs` 或 example-specific shape。

验收标准：

- 与 Newton/MuJoCo 逐字段打印对齐。
- `test_robot_policy.py` 不再使用零向量或只填 joint q 的占位 observation。

### P5.2 Action mapping

- 支持 `action_scale`。
- 支持 default pose offset。
- 支持 joint name reorder，例如 lab/physx/mujoco order mapping。

验收标准：

- 同一个 policy 输出能映射到正确 joint names。
- 可打印 action-to-joint table 供检查。

### P5.3 Policy decimation loop

- 支持每 N 个 physics step 执行一次 policy。
- 保留 previous action。
- 支持 command velocity 更新。

验收标准：

- `robot_policy` 和 `anymal_c_walk` 能配置 decimation。
- policy step 数与 frame/substep 关系正确。

### P5.4 Termination/reward/reset

- 实现 base height termination。
- 实现 fall detection。
- 实现 timeout。
- RL reward 可先做最小占位，但 reset 必须真实恢复状态。

验收标准：

- 能连续跑多个 episode。
- reset 不重建 solver。

### P5.5 Batch ONNX inference

- 支持 `[world_count, obs_dim] -> [world_count, action_dim]`。
- `onnxruntime` 作为 optional dependency，缺失时应明确 skip，而不是伪通过。

验收标准：

- world_count > 1 时每个 world 都有独立 observation/action。

## P6: 高级 Newton Examples

这些任务排在 P1-P5 之后，不建议提前做完整 parity。

### P6.1 `robot_ur10`

- 依赖 importer、ArticulationView、position drive、batch target。
- 目标：`world_count=100`，disable contacts，sinusoidal joint trajectory。

### P6.2 `robot_anymal_c_walk`

- 依赖完整 RL observation/action/reset/contact。
- 目标：base forward displacement 和 velocity assertions。

### P6.3 `robot_policy`

- 依赖多 robot config、YAML、joint reorder、batch inference。
- 目标：支持 anymal/go2/g1 policy variants。

### P6.4 `robot_allegro_hand`

- 依赖高 DOF drive、动态 joint parent transform、grasp contact。
- 目标：cube retention ratio。

### P6.5 `robot_panda_hydro`

- 依赖 IK、gripper effort limit、SDF/hydroelastic 或普通 contact fallback。
- 初期可先做 non-hydroelastic downgraded parity。

## 可并行分工建议

| Agent | 范围 | 文件入口 |
|-------|------|----------|
| Agent A | 文档状态矩阵与验收标准 | `todo/SCENE_EXAMPLES.md`, `todo/DONE.md`, `todo/COMMIT_LOG.md` |
| Agent B | P1 小场景 parity | `PythonBindings/robotics/test_basic_pendulum.py`, `test_basic_joints.py`, `test_cartpole.py` |
| Agent C | P2 batch/articulation | `PythonBindings/robotics/solver/robot_solver.py`, new `articulation_view.py` |
| Agent D | P3 importer/physics params | `PythonBindings/robotics/parser/urdf_parser.py`, `robot_builder.py`, C++ `WorldData`/initializer |
| Agent E | P4 standing contact parity | H1/G1/ANYmal tests, solver config/contact tuning |
| Agent F | P5 RL policy pipeline | `PythonBindings/robotics/test_robot_policy.py`, `training/robot_env.py`, new observation/action utilities |

## 推荐执行顺序

1. P0 文档与状态修正。
2. P1.1/P1.2/P1.3 小场景 parity。
3. P2.1/P2.2/P2.3 batch + ArticulationView。
4. P3.1/P3.2/P3.3 importer 和物理参数。
5. P4.1/P4.2/P4.3 standing parity。
6. P5.1-P5.5 RL policy pipeline。
7. P6 高级 examples。

## 当前不要误判为完成的事项

- `test_robot_h1_stand.py` 不是 Newton `robot_h1` parity，只是 H1 URDF smoke/proxy。
- `test_robot_g1_stand.py` 不是 Newton `robot_g1` parity。
- `test_robot_anymal.py` 是 procedural proxy，不是真实 ANYmal D parity。
- `test_robot_policy.py` 是 ONNX inference smoke，不代表 policy transfer 成功。
- `collision_shapes.py` 是 descriptor，不代表 primitive collision shape pipeline 已接入 C++。
- `FreeJoint` 当前更接近 placeholder，不代表 floating-base articulation parity 已完成。
