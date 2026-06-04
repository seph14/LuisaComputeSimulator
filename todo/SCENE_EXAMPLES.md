# Scene Examples: 1:1 Newton 场景复刻

本文档列出 Newton README 中覆盖的 **Basic Examples** 与 **Robot Examples**（跳过 Rod/Cable 和 MPM 相关内容），逐一记录 Newton 原始场景的物理配置参数、测试断言和 LCS 侧实现要点，用于 1:1 复刻。

## 状态分层约定

| 状态 | 含义 |
|------|------|
| `not_started` | 尚无实现 |
| `proxy` | 用手工或简化资产近似场景，验证接口和数值不崩 |
| `smoke` | 使用部分真实资产或真实 policy，但断言宽松，只验证流程跑通 |
| `parity_in_progress` | 资产、参数、assertion 正在对齐 Newton |
| `parity_passed` | 与 Newton example 的核心配置和测试断言对齐通过 |

## 缺口字段说明

每个未 parity 的 example 标注阻塞组件：

| 缺口字段 | 含义 |
|----------|------|
| `asset` | 缺少真实 USD/URDF 资产或 mesh 文件 |
| `importer` | URDF/USD importer 未完成或 transform pipeline 有误差 |
| `batch` | 缺少 world_count 批量复制能力 |
| `floating_base` | 缺少 free joint / floating base 支持 |
| `contact` | 接触稳定性不足或 collision shape pipeline 未接入 C++ |
| `observation` | 缺少 observation builder（body-frame velocity, projected gravity 等）|
| `assertion` | 当前断言与 Newton 原始断言有差距 |
| `joint_drive` | joint physical params（limit_ke/kd, friction, armature）未对齐 |
| `collision_group` | 缺少 collision group/mask 实现 |

## 当前状态总览

| Example | 状态 | 阻塞缺口 |
|---------|------|----------|
| B01 basic_pendulum | `not_started` | — |
| B02 basic_urdf | `not_started` | importer, floating_base, batch |
| B03 basic_shapes | `not_started` | contact |
| B04 basic_joints | `not_started` | — |
| B05 basic_conveyor | `not_started` | contact, collision_group |
| B06 basic_heightfield | `not_started` | contact |
| B07 recording | `proxy` | — |
| B08 replay_viewer | `not_started` | — |
| B09 basic_viewer | `proxy` | — |
| B10 basic_plotting | `not_started` | — |
| R01 robot_cartpole | `proxy` | batch, assertion |
| R02 robot_g1 | `smoke` | importer, floating_base, contact, assertion, joint_drive |
| R03 robot_h1 | `smoke` | importer, floating_base, contact, assertion, joint_drive |
| R04 robot_anymal_d | `proxy` | asset, importer, floating_base, contact, assertion |
| R05 robot_anymal_c_walk | `not_started` | asset, importer, floating_base, observation, batch, assertion |
| R06 robot_policy | `smoke` | observation, assertion, floating_base, batch |
| R07 robot_ur10 | `smoke` | batch, assertion |
| R08 robot_panda_hydro | `not_started` | asset, importer, contact |
| R09 robot_allegro_hand | `proxy` | batch, assertion, contact |

---

## Basic Examples

---

### B01 — basic_pendulum (双摆)

- **状态**: `not_started`
- **缺口**: 无阻塞组件，待实现
- **Newton 源文件**: `newton/examples/basic/example_basic_pendulum.py`
- **运行命令**: `python -m newton.examples basic_pendulum`
- **求解器**: `SolverXPBD`
- **LCS 脚本**: `PythonBindings/robotics/test_basic_pendulum.py`（待创建）

**场景配置**:
| 参数 | 值 |
|------|-----|
| fps | 100 |
| substeps | 10 |
| sim_dt | 0.001 s |
| link_0 | box hx=1.0, hy=0.1, hz=0.1 |
| link_1 | box hx=1.0, hy=0.1, hz=0.1 |
| j0 (world→link_0) | revolute, axis=(0,1,0), parent anchor=(0,0,5) |
| j1 (link_0→link_1) | revolute, axis=(0,1,0), child anchor offset=hx |
| ground_plane | 有 |

**测试断言** (`test_final`):
- 两个 link 的 x 坐标 ≈ 0, y ∈ (-1, 1), z ∈ (0, 5)
- 超出摆平面的速度分量 ≈ 0
- 平面内速度在合理范围内 (< 10 m/s linear, < 10 rad/s angular)

**LCS 实现要点**:
- 手工创建 2 个 rigid body（box mesh），1 个 box 作为 world anchor（fixed body 模拟）
- 2 个 revolute joints：`add_revolute_joint(fixed_anchor, link_0, ...)` + `add_revolute_joint(link_0, link_1, ...)`
- Z-up 坐标系：重力 (0,0,-9.8)，地面 z=0
- 不需要 joint drive，纯重力动力学
- 新增脚本 `PythonBindings/robotics/test_basic_pendulum.py`

**Parity 验收**:
| 项目 | Newton 断言 | LCS 当前断言 |
|------|-------------|-------------|
| link x 坐标 | x ≈ 0 | 待实现 |
| link y 范围 | y ∈ (-1, 1) | 待实现 |
| link z 范围 | z ∈ (0, 5) | 待实现 |
| 平面外速度 | ≈ 0 | 待实现 |
| 平面内速度 | < 10 m/s linear, < 10 rad/s angular | 待实现 |

---

### B02 — basic_urdf (四足 URDF)

- **状态**: `not_started`
- **缺口**: `importer`, `floating_base`, `batch`
- **Newton 源文件**: `newton/examples/basic/example_basic_urdf.py`
- **运行命令**: `python -m newton.examples basic_urdf --world-count 100`
- **求解器**: `SolverXPBD`（可选 VBD）
- **资产**: `newton/examples/assets/quadruped.urdf`

**场景配置**:
| 参数 | 值 |
|------|-----| 
| joint armature | 0.01 |
| joint target_ke | 2000.0 |
| joint target_kd | 1.0 |
| shape mu | 1.0 |
| body armature | 0.01（叠加到每个 body 的 inertia 上） |
| floating | True |
| enable_self_collisions | False |
| initial joint q | 最后一组 12 个 DOF 设特定值 |
| world_count | 100 |
| ground_plane | 有 |

**测试断言** (`test_final`):
- 所有 link 速度 < 0.15
- 每个 world 的 root body z ≈ 0.46（误差 < 0.01）

**LCS 实现要点**:
- 需要 URDF importer（阶段 2）
- floating base 需要 free joint
- 12 个 revolute joints
- joint position drive 保持初始姿态
- batched replication：100 worlds
- 新增脚本 `test_robot_quadruped.py`

**Parity 验收**:
| 项目 | Newton 断言 | LCS 当前断言 |
|------|-------------|-------------|
| link 速度 | 所有 link 速度 < 0.15 | 待实现 |
| root body z | z ≈ 0.46 (误差 < 0.01) | 待实现 |
| world_count | 100 | 待实现 |

---

### B03 — basic_shapes (形状掉落)

- **状态**: `not_started`
- **缺口**: `contact`
- **Newton 源文件**: `newton/examples/basic/example_basic_shapes.py`
- **运行命令**: `python -m newton.examples basic_shapes`
- **求解器**: `SolverXPBD`（可选 VBD）
- **资产**: `newton/examples/assets/bunny.usd`（bunny mesh）

**场景配置**:
| 参数 | 值 |
|------|-----| 
| drop_z | 2.0 |
| shape mu | 0.5 |
| sphere | radius=0.5 |
| ellipsoid | rx=0.5, ry=0.5, rz=0.25 |
| capsule | radius=0.3, half_height=0.7 |
| cylinder | radius=0.4, half_height=0.6 |
| box | hx=0.5, hy=0.35, hz=0.25 |
| cone | radius=0.45, half_height=0.6 |
| mesh (bunny) | bunny.usd mesh |

**测试断言** (`test_final`):
- 每个形状 resting 在 ground 上：z 位置 = half_extent（误差 < 0.1）
- bunny z > -0.05, 且 x ≈ 0, y ≈ 4.0

**LCS 实现要点**:
- 需要 primitive collision shapes（阶段 3）：box/sphere/capsule/cylinder/cone
- 需要 mesh collision（bunny），已有能力
- 不同形状分别验证 resting height
- 新增脚本 `test_basic_shapes.py`

**Parity 验收**:
| 项目 | Newton 断言 | LCS 当前断言 |
|------|-------------|-------------|
| 各形状 resting z | z = half_extent (误差 < 0.1) | 待实现 |
| bunny z | z > -0.05 | 待实现 |
| bunny xy | x ≈ 0, y ≈ 4.0 | 待实现 |

---

### B04 — basic_joints (关节类型演示)

- **状态**: `not_started`
- **缺口**: 无阻塞组件（已有 ball joint）
- **Newton 源文件**: `newton/examples/basic/example_basic_joints.py`
- **运行命令**: `python -m newton.examples basic_joints`
- **求解器**: `SolverXPBD`（可选 VBD）

**场景配置**:
| 参数 | 值 |
|------|-----|
| fps | 100 |
| substeps | 10 |
| sim_dt | 0.001 s |
| cuboid | hx=0.1, hy=0.1, hz=0.75 |
| upper cuboid | hz = 0.25 × cuboid_hz |
| revolute 行 | axis=(1,0,0), parent (static) → child, initial angle=π/2 |
| prismatic 行 | axis=(0,0,1), parent (static) → child, limits=(-0.3, 0.3) |
| ball 行 | parent (static sphere) → child (cuboid), initial quat=rpy(0.5,0.6,0.7) |
| static shapes | density=0 |

**测试断言** (`test_final`):
- revolute 运动在平面内（垂直于 axis 的角速度 ≈ 0）
- ball body 保持到 joint sphere 距离 ≈ 0.75 ± 0.005
- slider 约束运动停止
- 所有运动中 link 的速度 < 3.0

**LCS 实现要点**:
- 需要 ball/spherical joint（阶段 2.6）
- ball joint 已实现，无需 blocked
- 3 个独立 articulation：revolute、prismatic、ball
- static body 密度=0（fixed）
- 新增脚本 `test_basic_joints.py`

**Parity 验收**:
| 项目 | Newton 断言 | LCS 当前断言 |
|------|-------------|-------------|
| revolute 平面内运动 | 垂直 axis 角速度 ≈ 0 | 待实现 |
| ball anchor distance | 0.75 ± 0.005 | 待实现 |
| slider 约束 | 运动停在 limit 内 | 待实现 |
| 所有 link 速度 | < 3.0 | 待实现 |

---

### B05 — basic_conveyor (传送带)

- **状态**: `not_started`
- **缺口**: `contact, collision_group`
- **Newton 源文件**: `newton/examples/basic/example_basic_conveyor.py`
- **运行命令**: `python -m newton.examples basic_conveyor`
- **求解器**: `SolverXPBD`（可选 VBD）

**场景配置**:
| 参数 | 值 |
|------|-----| 
| belt | 环形 prism mesh, inner_radius=1.56, outer_radius=2.04, hz=0.04, kinematic |
| belt speed | 0.75 m/s tangential → revolute joint 恒定角速度 |
| rails | 环形 prism mesh, static |
| bags (18) | box/capsule/sphere, 随机分布在 belt 上, 每种类型 6 个 |
| collision groups | belt=7, rails=3, 每个 bag 独立负 group |
| friction | belt mu=1.2, rail mu=0.8, bag mu=1.0 |

**测试断言** (`test_final`):
- belt body z 位置稳定
- 所有 bag z > -0.5（没穿透地面）
- 所有 bag 在场景边界内 (|x| < 4.0, |y| < 4.0)

**LCS 实现要点**:
- **较高复杂度**：需要 procedural annular mesh 生成
- kinematic body 需要 `body.set_is_kinematic()` 或 animation target
- 需要 collision group/filter（阶段 3.4）
- 大量物体接触：18 bags + belt + rails
- **建议后置到阶段 4 之后**
- 新增脚本 `test_basic_conveyor.py`

---

### B06 — basic_heightfield (高度场地形)

- **状态**: `not_started`
- **缺口**: `contact`（heightfield 碰撞未支持）
- **Newton 源文件**: `newton/examples/basic/example_basic_heightfield.py`
- **运行命令**: `python -m newton.examples basic_heightfield`
- **求解器**: `SolverXPBD`（可选 Mujoco）

**场景配置**:
| 参数 | 值 |
|------|-----| 
| heightfield | 50×50 grid, hx=hy=5.0, elevation=sin(x)*cos(y)*0.5 |
| spheres (5) | radius=0.3, 分散在高度场上方 |

**测试断言** (`test_final`):
- 所有 sphere z > -1.0（没穿透 heightfield）

**LCS 实现要点**:
- 需要 heightfield 碰撞几何（当前没有）
- **当前 blocked by: 无 heightfield support**
- 可降级为手工 mesh terrain（用现有 triangle mesh collision）
- 新增脚本 `test_basic_heightfield.py`

---

### B07 — recording (录制回放)

- **状态**: `proxy`
- **缺口**: 无阻塞组件
- **Newton 源文件**: `newton/examples/basic/example_recording.py`
- **运行命令**: `python -m newton.examples recording`

**场景简述**: 录制仿真状态到文件，用于离线回放分析。

**LCS 实现要点**:
- 已有 `save_current_frame_state_to_host()` 和 `save_sim_result()`
- 需要封装为 per-frame 录制 API
- 新增脚本 `test_recording.py`

---

### B08 — replay_viewer (回放查看器)

- **状态**: `not_started`
- **缺口**: 待封装
- **Newton 源文件**: `newton/examples/basic/example_replay_viewer.py`
- **运行命令**: `python -m newton.examples replay_viewer`

**场景简述**: 加载录制数据并回放渲染。

**LCS 实现要点**:
- 基于 recording 录制格式实现 replay
- 新增脚本 `test_replay_viewer.py`

---

### B09 — basic_viewer (基础查看器)

- **状态**: `proxy`
- **缺口**: 无阻塞组件
- **Newton 源文件**: `newton/examples/basic/example_basic_viewer.py`
- **运行命令**: `python -m newton.examples basic_viewer`

**场景简述**: 最小化 viewer demo，测试 UI 交互。

**LCS 实现要点**:
- Polyscope GUI 已有 `utils.polyscope_gui.SimulationGUI`
- 需要确保 robot bodies 显示正确
- 新增脚本 `test_basic_viewer.py`

---

### B10 — basic_plotting (数据绘制)

- **状态**: `not_started`
- **缺口**: 待封装
- **Newton 源文件**: `newton/examples/basic/example_basic_plotting.py`
- **运行命令**: `python -m newton.examples basic_plotting`

**场景简述**: 在仿真过程中绘制物理量时序曲线。

**LCS 实现要点**:
- 记录 per-step 的 energy、joint angle、contact count 等
- 用 matplotlib 绘制
- 新增脚本 `test_basic_plotting.py`

---

## Robot Examples

---

### R01 — robot_cartpole (小推车摆)

- **状态**: `proxy`
- **缺口**: `batch, assertion`
- **LCS 脚本**: `PythonBindings/robotics/test_cartpole.py`
- **Newton 源文件**: `newton/examples/robot/example_robot_cartpole.py`
- **运行命令**: `python -m newton.examples robot_cartpole --world-count 100`
- **求解器**: `SolverMuJoCo`
- **资产**: `newton/examples/assets/cartpole.usda`

**场景配置**:
| 参数 | 值 |
|------|-----| 
| shape density | 100.0 |
| joint armature | 0.1 |
| body armature | 0.1（每个 body inertia + armature*I） |
| world_count | 100 |
| initial joint q | 最后 3 个 DOF = [0.0, 0.3, 0.0] |
| collapse_fixed_joints | True |

**测试断言** (`test_final`):
- cart z=0 且 orientation=identity
- cart 只在 y 方向运动（|v_y| > 0.05, v_x=v_z=0, angular=0）
- pole1: y 线速度 + x 角速度 > 0.3
- pole2: yz 面运动 + x 角速度 > 0.2
- 所有 worlds 的 cart/pole velocities 一致

**LCS 实现要点**:
- 与阶段 1 的 cartpole 一致，但需要 USD 导入或手工构造
- batched worlds 速度一致性测试
- **这是阶段 1 的核心目标 demo**
- 已有脚本 `PythonBindings/robotics/test_cartpole.py`

**Parity 验收**:
| 项目 | Newton 断言 | LCS 当前断言 |
|------|-------------|-------------|
| cart z | z=0, orientation=identity | Z-drift ≤ 5e-4（无显式 z=0 检查）|
| cart 运动方向 | 仅 y 方向运动 | cart 沿 X 轴运动（不同于 Newton Y-up）|
| pole1 速度 | y 线速度 + x 角速度 > 0.3 | 无显式速度断言 |
| pole2 速度 | yz 面运动 + x 角速度 > 0.2 | 无 |
| world 一致性 | 所有 worlds cart/pole 速度一致 | 无（仅 world_count=1）|
| world_count | 100 | 1 |

---

### R02 — robot_g1 (Unitree G1 人形)

- **状态**: `smoke`
- **缺口**: `importer, floating_base, contact, assertion, joint_drive`
- **LCS 脚本**: `PythonBindings/robotics/test_robot_g1_stand.py`

- **Newton 源文件**: `newton/examples/robot/example_robot_g1.py`
- **运行命令**: `python -m newton.examples robot_g1 --world-count 4`
- **求解器**: `SolverMuJoCo（solver="newton", integrator="implicitfast"）`
- **资产**: `unitree_g1/usd_structured/g1_29dof_with_hand_rev_1_0.usda`

**场景配置**:
| 参数 | 值 |
|------|-----| 
| joint limit_ke | 1.0e3 |
| joint limit_kd | 1.0e1 |
| joint friction | 1e-5 |
| shape ke | 1.0e3 |
| shape kd | 2.0e2 |
| shape kf | 1.0e3 |
| shape mu | 0.75 |
| collapse_fixed_joints | True |
| enable_self_collisions | False |
| mesh approx | bounding_box |
| joint target_ke | 500.0 (DOF 7+) |
| joint target_kd | 10.0 (DOF 7+) |
| world_count | 4 |
| solver iterations | 100 |
| solver ls_iterations | 50 |
| solver impratio | 100 |
| cone | elliptic |

**测试断言** (`test_final`):
- 所有 bodies z > 0
- 所有 body velocities < 0.015

**LCS 实现要点**:
- **复杂**：URDF/USD importer + floating base + 29 DOF joints + 接触稳定性
- 需要 joint position drive 保持初始姿态
- mesh approximation: bounding box
- 大量 solver iterations（100）和 line search（50）
- **阶段 4 目标 demo**
- 已有脚本 `PythonBindings/robotics/test_robot_g1_stand.py`

**Parity 验收**:
| 项目 | Newton 断言 | LCS 当前断言 |
|------|-------------|-------------|
| 所有 bodies z | > 0 | 通过（无显式检查）|
| 所有 body velocities | < 0.015 | 无 velocity 断言 |
| world_count | 4 | 1 |
| joint target_ke | 500.0 (DOF 7+) | 默认值（未对齐）|
| joint target_kd | 10.0 (DOF 7+) | 默认值（未对齐）|
| solver iterations | 100 | 默认值 |
| ls_iterations | 50 | 默认值 |

---

### R03 — robot_h1 (Unitree H1 人形)

- **状态**: `smoke`
- **缺口**: `importer, floating_base, contact, assertion, joint_drive`
- **LCS 脚本**: `PythonBindings/robotics/test_robot_h1_stand.py`
- **Newton 源文件**: `newton/examples/robot/example_robot_h1.py`
- **运行命令**: `python -m newton.examples robot_h1 --world-count 4`
- **求解器**: `SolverMuJoCo（iterations=100, ls_iterations=50）`
- **资产**: `unitree_h1/usd_structured/h1.usda`

**场景配置**:
| 参数 | 值 |
|------|-----| 
| joint limit_ke | 1.0e3 |
| joint limit_kd | 1.0e1 |
| joint friction | 1e-5 |
| shape ke | 2.0e3 |
| shape kd | 1.0e2 |
| shape kf | 1.0e3 |
| shape mu | 0.75 |
| enable_self_collisions | False |
| mesh approx | bounding_box |
| joint target_ke | 150 |
| joint target_kd | 5 |
| world_count | 4 |
| solver iterations | 100 |
| ls_iterations | 50 |
| njmax | 100 |
| nconmax | 210 |

**测试断言** (`test_final`):
- 所有 bodies z > 0
- 所有 body velocities < 0.005

**LCS 实现要点**:
- 与 G1 类似，但 H1 更高、重心更偏上
- 需要更严格的接触稳定性（velocities < 0.005）
- **阶段 4 目标 demo**
- 已有脚本 `PythonBindings/robotics/test_robot_h1_stand.py`

**Parity 验收**:
| 项目 | Newton 断言 | LCS 当前断言 |
|------|-------------|-------------|
| 所有 bodies z | > 0 | 通过（无显式检查）|
| 所有 body velocities | < 0.005 | 无 velocity 断言 |
| world_count | 4 | 1 |
| joint target_ke | 150 | 默认值（未对齐）|
| joint target_kd | 5 | 默认值（未对齐）|
| solver iterations | 100 | 默认值 |
| ls_iterations | 50 | 默认值 |

---

### R04 — robot_anymal_d (ANYmal D 四足)

- **状态**: `proxy`
- **缺口**: `asset, importer, floating_base, contact, assertion`
- **LCS 脚本**: `PythonBindings/robotics/test_robot_anymal.py`（procedural proxy）
- **运行命令**: `python -m newton.examples robot_anymal_d --world-count 16`
- **求解器**: `SolverMuJoCo（cone="elliptic", impratio=100）`
- **资产**: `anybotics_anymal_d/usd/anymal_d.usda`

**场景配置**:
| 参数 | 值 |
|------|-----| 
| up_axis | Z |
| joint limit_ke | 1.0e3 |
| joint limit_kd | 1.0e1 |
| joint friction | 1e-5 |
| shape ke | 2.0e3 |
| shape kd | 1.0e2 |
| shape kf | 1.0e3 |
| shape mu | 0.75 |
| collapse_fixed_joints | False |
| initial base height | 0.68 m |
| joint target_ke | 150 |
| joint target_kd | 5 |
| world_count | 16 |
| solver iterations | 100 |
| ls_iterations | 50 |
| nconmax | 45 |
| njmax | 100 |

**测试断言**: 隐式通过模型加载，无显式测试。

**LCS 实现要点**:
- floating base + 12 revolute joints
- up_axis=Z 显式设置
- collapse_fixed_joints=False：保留所有 joint
- `add_world()` 方式复制场景（非 `replicate`）
- **阶段 4 目标 demo**
- 已有脚本 `PythonBindings/robotics/test_robot_anymal.py`（procedural proxy）

**Parity 验收**:
| 项目 | Newton 断言 | LCS 当前断言 |
|------|-------------|-------------|
| base height | ~0.68 m | ~0.65 m（接近）|
| world_count | 16 | 1 |
| 真实资产 | ANYmal D USD | 手工 procedural proxy |
| up_axis | Z | Z（已对齐）|
| 断言 | 无显式测试 | 仅 drift 检查 |

---

### R05 — robot_anymal_c_walk (ANYmal C 行走)

- **状态**: `not_started`
- **缺口**: `asset, importer, floating_base, observation, batch, assertion`
- **LCS 脚本**: 待创建 `PythonBindings/robotics/test_robot_anymal_c_walk.py`
- **Newton 源文件**: `newton/examples/robot/example_robot_anymal_c_walk.py`
- **运行命令**: `python -m newton.examples robot_anymal_c_walk`
- **求解器**: `SolverMuJoCo（solver="newton", ls_iterations=50, njmax=50, nconmax=100）`
- **资产**: `anybotics_anymal_c/urdf/anymal.urdf`
- **Policy**: `anymal_walking_policy_physx.onnx`

**场景配置**:
| 参数 | 值 |
|------|-----| 
| joint armature | 0.06 |
| joint limit_ke | 1.0e3 |
| joint limit_kd | 1.0e1 |
| shape ke | 5.0e4 |
| shape kd | 5.0e2 |
| shape kf | 1.0e3 |
| shape mu | 0.75 |
| floating | True |
| collapse_fixed_joints | True |
| enable_self_collisions | False |
| joint target_ke | 150 |
| joint target_kd | 5 |
| initial joint q | 12 个指定值（命名 joint） |
| observation dim | 48 (3+3+3+3+12+12+12) |
| action_scale | 0.5 |
| reorder | lab_to_mujoco / mujoco_to_lab |

**测试断言** (`test_final`):
- 13 个 body names 匹配预期
- 13 个 joint names 匹配预期（包括 floating_base）
- 所有 bodies z > 0.1
- base 向 forward 方向移动 y > 9.0
- base 速度在 forward_vel_min/max 之间

**LCS 实现要点**:
- **最高复杂度 demo**
- 需要 URDF importer + floating base + policy inference + observation
- joint name-based 初始角度设置
- 需要 ONNX Runtime
- 需要 observation/action reordering
- **阶段 5 目标 demo**
- LCS 脚本: 待创建 `PythonBindings/robotics/test_robot_anymal_c_walk.py`

---

### R06 — robot_policy (通用 RL Policy)

- **状态**: `smoke`
- **缺口**: `observation, assertion, floating_base, batch`
- **LCS 脚本**: `PythonBindings/robotics/test_robot_policy.py`（ONNX inference smoke）
- **Newton 源文件**: `newton/examples/robot/example_robot_policy.py`
- **运行命令**: `python -m newton.examples robot_policy --robot g1_29dof`
- **求解器**: `SolverMuJoCo（solver="newton", nconmax=30, njmax=100）`
- **资产**: `unitree_g1`, `unitree_go2`, `anybotics_anymal_c`

**支持的 robot 和 policy**:
| Robot | Policy |
|-------|--------|
| anymal (mjw) | `rl_policies/mjw_anymal.onnx` |
| anymal (physx) | `rl_policies/physx_anymal.onnx` |
| go2 (mjw) | `rl_policies/mjw_go2.onnx` |
| go2 (physx) | `rl_policies/physx_go2.onnx` |
| g1_29dof (mjw) | `rl_policies/mjw_g1_29DOF.onnx` |
| g1_23dof (mjw) | `rl_policies/mjw_g1_23DOF.onnx` |
| g1_23dof (physx) | `rl_policies/physx_g1_23DOF.onnx` |

**场景配置**:
| 参数 | 值 |
|------|-----| 
| decimation | 4 |
| cycle_time | 0.02 s | 
| up_axis | Z |
| joint armature | 0.1 |
| joint limit_ke | 1.0e2 |
| joint limit_kd | 1.0e0 |
| shape ke | 5.0e4 |
| shape kd | 5.0e2 |
| shape kf | 1.0e3 |
| shape mu | 0.75 |
| mesh approx | convex_hull |
| collapse_fixed_joints | False |
| enable_self_collisions | False |
| initial base height | 0.8 m |
| gravity | (0,0,-9.81) |
| observation dim | 12 + 3 × num_dofs |

**测试断言** (`test_final`):
- 所有 bodies z > 0

**LCS 实现要点**:
- 需要通用 policy 框架（多个 robot 类型 + 多种 policy 后端）
- 键盘控制 (i/j/k/l/u/o) 驱动 command velocity
- 按 "p" reset
- YAML config 加载 joint stiffness/damping/armature/action_scale
- joint name reordering (physx vs mujoco)
- **阶段 5 目标 demo**
- 已有脚本 `PythonBindings/robotics/test_robot_policy.py`（ONNX inference smoke）

**Parity 验收**:
| 项目 | Newton 断言 | LCS 当前断言 |
|------|-------------|-------------|
| 所有 bodies z | > 0 | 无显式检查 |
| observation | 12 + 3 × num_dofs | 零向量占位 |
| action | action_scale + default pose offset | ONNX inference only |
| policy decimation | 4 | 1（每步推理）|
| world_count | 可变 | 1 |
| 多 robot 支持 | anymal/go2/g1_29dof/g1_23dof | 仅 G1 23-DOF |

---

### R07 — robot_ur10 (UR10 机械臂)

- **状态**: `smoke`
- **缺口**: `batch, assertion`
- **LCS 脚本**: `PythonBindings/robotics/test_robot_arm.py`（smoke, 非 UR10）
- **Newton 源文件**: `newton/examples/robot/example_robot_ur10.py`
- **运行命令**: `python -m newton.examples robot_ur10 --world-count 100`
- **求解器**: `SolverMuJoCo（disable_contacts=True）`
- **资产**: `universal_robots_ur10/usd/ur10_instanceable.usda`

**场景配置**:
| 参数 | 值 |
|------|-----| 
| base height | 1.2 m |
| pedestal | cylinder, half_height=0.6, radius=0.08 |
| joint target_ke | 500 |
| joint target_kd | 50 |
| joint target_mode | POSITION |
| collapse_fixed_joints | False |
| enable_self_collisions | False |
| joint trajectory | sinusoidal, 每个 DOF 独立 |
| world_count | 100 |
| world spacing | (2, 2, 0) |
| contacts | disabled |

**测试断言**: 无显式测试（纯 visual demo）。

**LCS 实现要点**:
- 无碰撞（disable_contacts），纯 joint dynamics
- 需要 sinusoidal joint trajectory 生成
- batched worlds with spacing
- `ArticulationView` 批量 joint target 设置
- **阶段 2 目标 demo**
- 已有脚本 `PythonBindings/robotics/test_robot_arm.py`（smoke, 非 UR10）

---

### R08 — robot_panda_hydro (Panda 水弹性抓取)

- **状态**: `not_started`
- **缺口**: `asset, importer, contact`
- **LCS 脚本**: 待创建
- **Newton 源文件**: `newton/examples/robot/example_robot_panda_hydro.py`
- **运行命令**: `python -m newton.examples robot_panda_hydro --scene pen --world-count 1`
- **求解器**: `SolverMuJoCo + HydroelasticSDF contact`
- **资产**: `franka_emika_panda/urdf/fr3_franka_hand.urdf`, `manipulation_objects/pad`, `manipulation_objects/cup`

**场景配置**:
| 参数 | 值 |
|------|-----|
| fps | 60 |
| substeps | 10 |
| collide_substeps | 2（每 2 步做碰撞检测） |
| sim_dt | 1/600 s |
| sdf_max_resolution | 64 |
| sdf_narrow_band_range | (-0.01, 0.01) |
| shape kh | 1.0e11 |
| shape gap | 0.01 |
| shape mu_torsional | 0.0 |
| shape mu_rolling | 0.0 |
| enable_self_collisions | False |
| parse_visuals_as_colliders | True |
| hand shapes | HYDROELASTIC + mesh SDF |
| non-hand shapes | convex_hull |
| gripper pads | mesh SDF |
| table | box, mesh SDF |
| object | pen (capsule) or cube (box), both SDF |
| joint target_ke | 650.0 |
| joint target_kd | 100.0 |
| joint effort_limit | 80.0 (arm), 20.0 (gripper) |
| joint armature | 0.1 (arm), 0.5 (gripper) |

**测试断言**: 无显式测试。

**LCS 实现要点**:
- **最高接触复杂度**：SDF/hydroelastic contact 不是当前 IPC/barrier 模型
- **不建议第一阶段做**
- 可降级为普通 contact（不用 hydroelastic）
- 需要 IK solver
- 需要 gripper finger effort limits
- pick-and-place 闭环
- **阶段 7 目标 demo**
- LCS 脚本: 待创建

---

### R09 — robot_allegro_hand (Allegro 灵巧手)

- **状态**: `proxy`
- **缺口**: `batch, assertion, contact`
- **LCS 脚本**: `PythonBindings/robotics/test_robot_allegro.py`
- **Newton 源文件**: `newton/examples/robot/example_robot_allegro_hand.py`
- **运行命令**: `python -m newton.examples robot_allegro_hand --world-count 100`
- **求解器**: `SolverMuJoCo（solver="newton", integrator="implicitfast", iterations=100, ls_iterations=50）`
- **资产**: `wonik_allegro/usd/allegro_left_hand_with_cube.usda`

**场景配置**:
| 参数 | 值 |
|------|-----| 
| shape ke | 1.0e3 |
| shape kd | 1.0e2 |
| shape margin | 0.005 |
| shape gap | 0.015 |
| enable_self_collisions | False |
| joint target_ke | 150 |
| joint target_kd | 5 |
| joint armature (revolute) | 1.0e-2 |
| finger joint targets | sin(t + i*0.6) * 0.08 + 0.3, 在 limits 内 clamp |
| root joint | 动态 X 轴旋转：sin(t) * 0.1 |
| hand rotation | (0.21643, 0.706218, -0.648166, 0.185191) |
| world_count | 100 |
| solver njmax | 200 |
| solver nconmax | 300 |
| solver impratio | 20.0 |
| solver cone | elliptic |

**测试断言** (`test_final`):
- hand bodies 靠近初始位置
- ≥ 50% cubes 仍在手内（z > 0.9 且 xy 在 ±0.5 范围内）

**LCS 实现要点**:
- **需要动态 joint parent transform**（`SolverNotifyFlags.JOINT_PROPERTIES`）
- 20+ revolute joints：需要高刚度、大 iteration 数
- 物体抓取接触稳定性
- 3 个 per-world 测试断言：
  - hand 未掉落
  - cube 未穿透地面
  - cube 仍在手内比例
- **阶段 6 目标 demo**
- 已有脚本 `PythonBindings/robotics/test_robot_allegro.py`

**Parity 验收**:
| 项目 | Newton 断言 | LCS 当前断言 |
|------|-------------|-------------|
| hand 位置 | 靠近初始位置 | 仅 drift 检查 |
| cube 保留率 | ≥ 50% cubes in hand | 无 |
| world_count | 100 | 1 |
| finger targets | sin(t+i*0.6)*0.08+0.3 | 固定初始值 |
| root joint | 动态旋转 | 无 |

---

## 汇总表：实现优先级

| 优先级 | 示例 | 理由 |
|--------|------|------|
| P0（阶段 1） | basic_pendulum, robot_cartpole | 无 importer 依赖，核心验证 joint 能力 |
| P0（阶段 1） | basic_joints | 验证 ball/prismatic/revolute joint 类型全覆盖 |
| P1（阶段 2） | robot_ur10 | 需要 URDF importer + joint drive，无接触需求 |
| P1（阶段 2） | basic_urdf | 需要 URDF + floating base + batched env |
| P2（阶段 3） | basic_shapes | 需要 primitive collision shapes |
| P2（阶段 4） | robot_g1, robot_h1, robot_anymal_d | standing demos，需要精确接触 + mesh approx |
| P3（阶段 4） | basic_conveyor | 需要 kinematic body + collision groups |
| P3（阶段 5） | robot_policy, robot_anymal_c_walk | RL policy inference 全链路 |
| P4（阶段 6） | robot_allegro_hand | 大量 joint + 抓取接触 |
| P5（阶段 7） | robot_panda_hydro | IK + gripper + hydroelastic contact |
| 低优先 | basic_heightfield | 需要 heightfield 碰撞，可用 mesh terrain 替代 |
| 低优先 | basic_viewer, recording, replay_viewer, basic_plotting | 工具类，已有或易实现 |

---

## 每个示例的 LCS 实现检查清单

| 示例 | 所需能力 | Parity 状态 |
|------|----------|-------------|
| basic_pendulum | rigid body + revolute + ground | `not_started` |
| basic_urdf | URDF import + floating base + batched env | `not_started` |
| basic_shapes | primitive collision shapes (box/sphere/capsule/cylinder/cone) | `not_started` |
| basic_joints | ball joint + static bodies（ball joint 已实现）| `not_started` |
| basic_conveyor | kinematic body + collision filter + procedural mesh | `not_started` |
| basic_heightfield | heightfield collision | `not_started` |
| recording | state serialization | `proxy` |
| replay_viewer | state deserialization + replay | `not_started` |
| basic_viewer | Polyscope integration | `proxy` |
| basic_plotting | per-step data logging | `not_started` |
| robot_cartpole | prismatic + revolute joints + limits + batched env | `proxy` |
| robot_g1 | URDF + floating base + 29 DOF + mesh approx + standing | `smoke` |
| robot_h1 | URDF + floating base + mesh approx + standing | `smoke` |
| robot_anymal_d | URDF/USD + floating base + 12 DOF + standing | `proxy` |
| robot_anymal_c_walk | URDF + floating base + policy + observation | `not_started` |
| robot_policy | USD + policy + 多 robot 类型 | `smoke` |
| robot_ur10 | URDF/USD + position drive + sinusoidal trajectory | `smoke` |
| robot_panda_hydro | SDF contact + IK + gripper | `not_started` |
| robot_allegro_hand | 20+ revolute + 动态 transform + 抓取 | `proxy` |
