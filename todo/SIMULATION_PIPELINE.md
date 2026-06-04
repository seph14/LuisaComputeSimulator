# NewtonSolver 仿真管线全流程

> 本文档梳理了 LuisaComputeSimulator 中 `NewtonSolver` 从初始化到每一步
> 仿真推进的完整数据流。基于对 H1 机器人 floating_base 重力问题的调试过程。

---

## 0. 数据架构概览

### 关键数据结构

```
SceneParams (全局单例, weak_ptr)
├── gravity, floor, floor_normal
├── implicit_dt, num_substep
├── nonlinear_iter_count, pcg_iter_count
├── use_floor, use_gpu, use_self_collision
├── use_energy_linesearch (=false), use_ccd_linesearch (=true)
├── fix_scene (=false), damping_rate
└── up_axis (Y_UP / Z_UP)

WorldData (每个注册的物体一个)
├── mesh vertices/faces (表面网格)
├── material_type (Rigid / Soft / Cloth...)
├── physics_material (variant: RigidMaterial / ClothMaterial / ...)
│   ├── model (ConstitutiveModelRigid: Orthogonality/StableNeoHookean/...)
│   ├── density, mass, thickness, stiffness
│   └── is_shell (=true 默认)
├── fixed_point_indices (固定顶点列表)
└── 变换: translation, rotation, scale

MeshData (从所有 WorldData 合并)
├── sa_vert_mass, sa_vert_mass_inv (每顶点质量)
├── sa_body_mass (每物体总质量)
├── sa_is_fixed (每顶点是否固定)
├── sa_rest_vert_volume, sa_rest_body_volume
└── prefix_num_verts/faces/edges

SimulationData (求解器运行时数据)
├── sa_q            (DOF 变量 - 当前求解状态, 被 Newton 迭代更新)
├── sa_q_step_start (每步/每帧开始时的 DOF 值)
├── sa_q_tilde      (预测位置 = q + dt*v + dt²*g)
├── sa_q_v          (DOF 速度)
├── sa_q_iter_start (Newton 迭代开始时的 DOF 值)
├── sa_dq, sa_dx    (Newton 搜索方向)
├── sa_cgB          (线性系统右端项 = 负梯度组装)
├── sa_cgA_diag     (线性系统对角线 Hessian)
├── sa_cgA_offdiag  (线性系统非对角线 Hessian)
│
├── sa_q_outer      (对外暴露的 DOF - get_rigid_body_translation 读这里)
├── sa_q_v_outer    (对外暴露的速度)
├── sa_x_outer      (对外暴露的顶点位置)
│
├── sa_x            (顶点位置)
├── sa_x_step_start (每步开始时顶点位置)
├── sa_x_to_dof_map (顶点→DOF 映射)
├── sa_q_property   (DOF 属性: is_fixed, is_rigid, is_translation_dof)
└── sa_q_is_fixed   (DOF 是否固定)

AbdInertiaData (每个刚体 4 个 DOF)
├── constraint_indices[i] = uint4(dof_trans, dof_rot0, dof_rot1, dof_rot2)
├── sa_affine_bodies_mass_matrix[i] = float4x4 (压缩的 12x12 质量矩阵)
├── sa_stiffness_dirichlet[i] = has_fixed_vert ? 1e5 : 1.0
├── constraint_gradients[4*i .. 4*i+3]
└── constraint_hessians[16*i .. 16*i+15]
```

### DOF 索引规则

```
num_verts_soft   = 软体顶点数 (每个顶点 1 DOF)
num_affine_bodies = 刚体数 (每个刚体 4 DOF)
num_dof           = num_verts_soft + num_affine_bodies * 4

刚体 body_idx 的 DOF:
  dof_start = num_verts_soft + body_idx * 4
  dof_start + 0 → 平移向量
  dof_start + 1 → 旋转矩阵第1列
  dof_start + 2 → 旋转矩阵第2列
  dof_start + 3 → 旋转矩阵第3列
```

---

## 1. 初始化阶段

### 1.1 Python 侧配置

```python
# test_robot_h1_stand.py / RobotSolver.setup_z_up()
rs = RobotSolver(backend_name="metal")
rs.init_device()
rs.setup_z_up(dt=1.0/300.0)
    # → config.set_up_axis(Z_UP)
    #     → gravity = (0, 0, -9.8)
    #     → floor_normal = (0, 0, 1)
    # → config.set_implicit_dt(1/300)
    # → config.set_num_substep(3)
    # → config.set_nonlinear_iter_count(5)
    # → config.set_pcg_iter_count(200)
```

**关键**: `set_up_axis(Z_UP)` 在 C++ 层 (`scene_params.h:102-117`) 设置:
- `gravity = {0, 0, -9.8}`
- `floor = {0, 0, 0}`
- `floor_normal = {0, 0, 1}`

默认是 `Y_UP`: `gravity = {0, -9.8, 0}`, `floor_normal = {0, 1, 0}`.

### 1.2 注册物体 (WorldData → register_world_data)

```python
# add_rigid_body() → create_world_data_from_array()
body = solver.create_world_data_from_array(name, verts, faces)
body.set_simulation_type(MaterialType.Rigid)
body.set_physics_material_rigid(model, thickness, stiffness, density, mass, ...)
#    → RigidMaterial 存入 physics_material variant
#    → 如果 mass != 0, 后续 init_mesh_data 使用显式 mass
#    → 如果 mass == 0, 使用 volume * density
body.set_translation(tx, ty, tz)
if fixed:
    body.add_fixed_point_by_indices([0])  # 固定第0号顶点
rid = solver.register_world_data(body)
```

**质量问题**: Python `add_rigid_body` 中 `model = "stable_neohookean"` (全小写),
`parse_rigid_model()` 查表失败, 回退到默认 `ConstitutiveModelRigid::Orthogonality`.
**不影响功能**, 但应修复为 `"StableNeoHookean"` 或直接用 `"Orthogonality"`.

### 1.3 init_solver()

触发 `init_mesh_data()` → `init_sim_data()`:

**init_mesh_data.cpp:**
1. 合并所有 WorldData 的网格
2. 计算每个物体的体积和 mass:
   ```cpp
   // line 584-586
   input_mass = shell_info.get_mass();     // 从 RigidMaterial.mass
   input_density = shell_info.get_density(); // 从 RigidMaterial.density
   sa_body_mass[meshIdx] = input_mass != 0 ? input_mass : sum_volume * input_density;
   ```
3. 按体积权重分配顶点质量:
   ```cpp
   // line 615-630
   vert_mass = (vert_volume / body_volume) * body_mass;
   sa_vert_mass_inv[vid] = is_fixed ? 0.0f : 1.0f / vert_mass;
   ```
4. 固定点传播: `sa_is_fixed[vid] = true`

**init_sim_data.cpp:**
1. 建立顶点→DOF 映射 (`sa_x_to_dof_map`)
2. 设置 DOF 属性:
   - 刚体 DOF: `is_rigid = true`, 平移 DOF 额外标记 `is_translation_dof = true`
   - 固定点: `is_fixed = true` → 所有 4 个 DOF 都标记为 fixed
3. 组装刚体质量矩阵 (12x12 → 压缩为 4x4):
   ```cpp
   // line 1057-1169
   M_body  = Σ vert_mass           // 总质量
   MI_body = Σ vert_mass * vert_pos // 一阶矩
   I_body  = Σ vert_mass * vert_pos * vert_pos^T  // 二阶矩
   // 组装 12x12 → 压缩为 float4x4
   ```
4. 设置 `sa_stiffness_dirichlet[body_idx] = has_fixed_vert ? 1e5 : 1.0`

**关键**: `sa_stiffness_dirichlet = 1.0` 对浮动刚体是正确的惯性缩放因子.
如果为 0, 惯性能量为 0, 重力完全不起作用.

### 1.4 init_solver() 后的状态

- `sa_q = sa_q_step_start = sa_q_outer` = 初始位姿
- `sa_q_v = sa_q_v_outer` = 零向量
- `sa_q_tilde` = 尚未计算

---

## 2. 每帧主循环 (physics_step_cpu)

```
physics_step_cpu()
├── 2.1 physics_step_prev_operation()  ← 帧前操作
├── 2.2 host_apply_q_to_x() + buffer_copy()  ← 初始化 q/x
├── 2.3 LBVH 构建 (如有自碰撞)
├── 2.4 [Substep 循环]  ← 注意: CPU 路径 substep 循环被注释!
│   ├── 2.4.1 host_predict_position() → sa_q_tilde
│   ├── 2.4.2 [Newton 迭代循环] (最多 100 次)
│   │   ├── 2.4.2.1 保存 iter_start
│   │   ├── 2.4.2.2 重置 cgB/cgX/diagA/offdiag
│   │   ├── 2.4.2.3 能量 evaluation
│   │   ├── 2.4.2.4 host_material_energy_assembly()
│   │   ├── 2.4.2.5 接触 evaluation
│   │   ├── 2.4.2.6 linear_solver (PCG)
│   │   └── 2.4.2.7 line_search()
│   └── 2.4.3 host_update_velocity()
└── 2.5 physics_step_post_operation()  ← 帧后操作
```

### 2.1 physics_step_prev_operation()

```cpp
// solver_interface.cpp:348-349
buffer_copy(host_sim_data->sa_q_outer, host_sim_data->sa_q_step_start);
buffer_copy(host_sim_data->sa_q_v_outer, host_sim_data->sa_q_v);
```

将当前帧起始状态保存到 `_outer` 缓冲区, 供外部查询.

### 2.2 初始化 q/x

```cpp
// newton_solver.cpp:2113-2115
host_apply_q_to_x(sa_q_step_start, sa_x_step_start);
buffer_copy(sa_q_step_start, sa_q);
buffer_copy(sa_x_step_start, sa_x);
```

**关键**: `sa_q` 初始化为 `sa_q_step_start` (上一帧结束时的位置),
**不是** `sa_q_tilde` (预测位置). 重力通过惯性能量的梯度间接引入.

---

## 3. Predict Position (重力注入点)

### 3.1 host_predict_position()

```cpp
// newton_solver.cpp:877-905
CpuParallel::parallel_for(0, num_dof, [...] {
    const auto dof_property = sa_q_property[vid];
    const bool is_rigid = dof_property.is_rigid();
    const bool is_translation_dof = dof_property.is_translation_dof();
    const bool is_fixed = dof_property.is_fixed();

    float3 x_prev = sa_q_step_start[vid];
    float3 v_prev = sa_q_v[vid];
    float3 v_pred = v_prev;

    // 仅对非固定的刚体平移 DOF 施加重力
    if ((!is_fixed) & ((!is_rigid) | (is_rigid & is_translation_dof)))
    {
        v_pred += substep_dt * gravity;  // ← 重力在这里注入!
    };
    float3 x_pred = x_prev + substep_dt * v_pred;
    sa_q_tilde[vid] = x_pred;
});
```

**数学**: `q_tilde = q_n + dt * v_n + dt² * g`

**关键条件**: 重力只在 `!is_fixed` 时施加. 对于:
- `floating_base` (无固定点): `is_fixed = false` → 重力生效
- `fixed_base` (根链接有固定点): `is_fixed = true` → 重力被跳过

### 3.2 substep_dt 的计算

```cpp
// scene_params.h
float get_substep_dt() const {
    return implicit_dt / static_cast<float>(num_substep);
}
```

`implicit_dt = 1/300`, `num_substep = 3` → `substep_dt = 1/900 ≈ 0.00111s`

**注意**: CPU 路径中 `for (uint substep = 0; ...)` 循环被注释掉了
(newton_solver.cpp:2127), 所以每帧只执行 **1 次** substep.
这意味着有效仿真时间是 `1/900` 秒/帧, 而非预期的 `1/300` 秒/帧.

---

## 4. 能量 Evaluation

Newton 迭代中依次评估每种能量项, 填充梯度 (`constraint_gradients`) 和 Hessian (`constraint_hessians`).

### 4.1 AbdInertia (刚体惯性)

**代码**: `abd_inertia_energy.cpp:138-189`

```cpp
// host_evaluate()
const float h = substep_dt;
const float h_2_inv = 1.0f / (h * h);

// 对每个刚体的 4 DOF:
delta_q[i] = sa_q[indices[i]] - sa_q_tilde[indices[i]];

// scaled_stiffness = stiffness_dirichlet / dt²
// 浮动刚体: stiffness_dirichlet = 1.0, scaled_stiffness = 1/dt²
scaled_stiffness = abd_stiffness_dirichlet[body_idx] * h_2_inv;

eval = detail::abd_inertia_energy::evaluate(delta_q, mass_matrix, scaled_stiffness, I);
```

**能量公式** (detail/abd_inertia_energy.hpp):
```
E = 0.5 * (stiffness_dirichlet/dt²) * Σᵢⱼ Mᵢⱼ * ⟨deltaᵢ, deltaⱼ⟩

梯度: gₖ = (stiffness_dirichlet/dt²) * Σⱼ Mₖⱼ * deltaⱼ
Hessian: Hₖⱼ = (stiffness_dirichlet/dt²) * Mₖⱼ * I₃
```

**数值示例** (第一帧, v=0):
```
delta[0] = q - q_tilde = q_n - (q_n + 0 + dt² * g)
         = -dt² * g = -(1/900)² * (0, 0, -9.8)
         = (0, 0, 1.21e-5)

g[0] = (1/dt²) * M_body * delta[0]
     = 810000 * M_body * (0, 0, 1.21e-5)
     = M_body * (0, 0, 9.8)

H[0][0] = (1/dt²) * M_body * I₃
       = 810000 * M_body * I₃
```

### 4.2 AbdOrthogonality (刚体正交约束)

惩罚旋转矩阵列偏离正交性: `E = stiffness * ||RᵀR - I||²`

**对平移 DOF 无直接影响** (梯度/Hessian 的平移分量为零).

### 4.3 JointConstraint (关节约束)

关节能量: 惩罚两个刚体之间相对位姿偏离关节配置.

**相对约束**: 关节只约束相对位姿, 不约束绝对位姿.
两个连接体同步平移时关节能量不变.

### 4.4 其他能量

- **SoftInertia**: 软体顶点惯性 (类似 AbdInertia, 但有 `sa_x_tilde`)
- **StretchSpring/StretchFace/Bending/StressTet**: 软体弹性力
- **GroundCollision**: 地面碰撞 (如果 `use_floor=true`)
- **SelfCollision**: 自碰撞 (如果 `use_self_collision=true`)

---

## 5. 能量组装 (host_material_energy_assembly)

**代码**: `newton_solver.cpp:1123-1211`

```cpp
// 对每个能量项, 遍历其涉及的 DOF, 调用 assembly_template2()
assembly_template2(vid, constraint_data, adj_verts, sa_cgB, sa_cgA_diag, sa_cgA_offdiag);

// assembly_template2() 内部:
// 1. 找到该 DOF 在约束中的 offset
// 2. 读取梯度和 Hessian
// 3. 组装到全局线性系统:
buffer_add(sa_cgB, vid, -grad);                    // ← 注意负号!
buffer_add(sa_cgA_diag, vid, diag_hess);
// 4. 处理非对角块
add_triplet_matrix(triplet, offdiag_hess);
```

**线性系统**: `A * dq = b` 其中:
- `b = -∇E` (cgB)
- `A = ∇²E` (cgA)

**对于浮动单刚体的平移 DOF**:
```
cgB[0] = -(-M_body * g) = M_body * g
cgA[0] = M_body / dt² * I₃
→ dq = A⁻¹ * b = dt² * g ≈ 1.21e-5 m
```

---

## 6. 线性求解 (PCG)

```cpp
// newton_solver.cpp:2101-2105
pcg_solver->host_solve(stream, pcg_spmv_interface, []() { return 0.0; });
// 求解 A * x = b, 结果存入 sa_cgX
```

然后:
```cpp
buffer_copy(host_sim_data->sa_cgX, host_sim_data->sa_dq); // dq = cgX
host_apply_q_to_x(sa_dq, sa_dx);
```

---

## 7. 线搜索 (line_search)

**代码**: `newton_solver.cpp:1797-1990`

### 7.1 收敛检查

```cpp
float max_move = 1e-2;
float curr_max_step = fast_infinity_norm(sa_dq);  // ||dq||_∞
if (curr_max_step < max_move * implicit_dt)  // < 3.33e-5
{
    apply_final_dx(alpha);  // 应用步长后直接返回
    dirichlet_converged = true;
    global_converged = true;
    return;
}
```

**影响**: 前两个 substep 中 `dq ≈ 1.2e-5 ~ 2.4e-5 < 3.33e-5`,
触发收敛 → 只做 1 次 Newton 迭代. **这不影响正确性** (对线性系统一次迭代就精确),
但**跳过能量线搜索**.

从第 3 个 substep 开始 `dq > 3.33e-5`, 不再触发.

### 7.2 CCD 线搜索 (默认启用)

```cpp
if (use_ccd_linesearch)  // 默认 true
{
    device_apply_dq_dx(stream, alpha=1.0);  // 在设备上应用全步长
    ccd_toi = ccd_get_toi();                // 读取首次碰撞时间
    alpha = ccd_toi;                         // 如有碰撞则缩减步长
}
```

### 7.3 能量线搜索 (默认禁用)

```cpp
if (use_energy_linesearch)  // 默认 false
{
    // 从 alpha=1.0 开始, 每次减半
    // 检查 E(x + alpha*dx) < E(x) + ε
    // 最多 20 次尝试
}
```

### 7.4 应用最终步长

```cpp
apply_final_dx(alpha);  // 同时更新设备和主机
// host: sa_q = sa_q_iter_start + alpha * sa_dq
// host: sa_x = sa_x_iter_start + alpha * sa_dx
```

---

## 8. 速度更新 (host_update_velocity)

**代码**: `newton_solver.cpp:906-937`

```cpp
CpuParallel::parallel_for(0, num_dof, [...] {
    x_step_begin = sa_q_step_start[vid];  // 本 substep 开始时的位置
    x_step_end = sa_q[vid];               // Newton 求解后的位置
    dx = x_step_end - x_step_begin;
    vel = dx / substep_dt;

    if (fix_scene)  // 默认 false
    {
        dx = 0; vel = 0;
        sa_q[vid] = x_step_begin;  // 回退!
        return;
    }

    vel *= exp(-damping * substep_dt);  // 阻尼衰减

    sa_q_v[vid] = vel;
    sa_q_step_start[vid] = x_step_end;  // 更新下一步起始位置
});
```

**关键**: `fix_scene=true` 会**清除所有运动** (位置和速度归零).
默认 `fix_scene=false`, 所以不影响.

### 8.1 速度累积示例

```
第1 substep: dq = dt²*g, v₁ = dt²*g/dt = dt*g
第2 substep: dq = dt*v₁ + dt²*g = 2*dt²*g, v₂ = 2*dt*g
...
第N substep: vₙ = N*dt*g, 总位移 = g*dt²*N*(N+1)/2
```

在只有 1 substep/帧 (CPU 路径), 120 帧后:
- `v = 120 * (1/900) * 9.8 = 1.31 m/s`
- `位移 = 9.8 * (1/900)² * 120*121/2 ≈ 0.088 m`

如果 3 substep/帧:
- `位移 = 9.8 * (1/900)² * 360*361/2 ≈ 0.786 m`

---

## 9. 帧后操作 (physics_step_post_operation)

**代码**: `solver_interface.cpp:351-357`

```cpp
buffer_copy(sa_x, sa_x_outer);    // 顶点 → 对外
buffer_copy(sa_v, sa_v_outer);    // 顶点速度 → 对外
buffer_copy(sa_q, sa_q_outer);    // DOF → 对外 (get_rigid_body_translation 读这里)
buffer_copy(sa_q_v, sa_q_v_outer);// DOF 速度 → 对外 (get_rigid_body_velocity 读这里)
current_frame += 1;
```

---

## 10. 外部查询接口

### get_rigid_body_translation()
```cpp
// solver_interface.cpp:658
out_translation = host_sim_data->sa_q_outer[dof_start + 0];  // ← 注意: 读 _outer
```

### get_rigid_body_velocity()
```cpp
// solver_interface.cpp:911-914
const auto& qv = host_sim_data->sa_q_v_outer;  // ← 读 _outer
lin_vel = qv[dof_start];
```

### save_result (OBJ)
```cpp
// solver_interface.cpp:611
output_positions = host_sim_data->sa_x_outer[prefix_vert + vid];  // ← 读 _outer
```

---

## 11. 发现的问题总结

### 🔴 问题1: CPU 路径 substep 循环被注释

**位置**: `newton_solver.cpp:2127`
```cpp
// for (uint substep = 0; substep < get_scene_params().num_substep; substep++)
```
导致每帧只执行 1 次 substep (而非配置的 3 次), 有效仿真时间为 `1/900` 秒/帧.

**影响**: 重力效果约为预期的 1/9 (位移正比于时间²).

### 🔴 问题2: 关节约束导致系统凝滞 (用户发现)

JointConstraint 能量项在特定条件下可能产生绝对约束效果,
阻止整个系统的平移运动. 需要进一步排查关节约束的梯度/Hessian 组装.

### 🟡 问题3: model 名称大小写不匹配

`robot_solver.py:82`: `model = "stable_neohookean"` (全小写)
`parse_rigid_model()` 查表失败, 回退到 `Orthogonality`. 功能正确但应修正.

### 🟡 问题4: 收敛检查阈值可能过松

`max_move = 1e-2`: 对于慢速运动, 每次 Newton 只做 1 轮迭代即收敛.
虽然对线性问题无害, 但对非线性问题可能不够.

---

## 12. 调试技巧

### 追踪数据流的关键点

1. **确认重力设置**: 打印 `config.get_gravity()` 输出
2. **确认质量**: 查看 `init_mesh_data` 日志: `body mass = X.XX`
3. **确认固定状态**: 检查 `sa_is_fixed` / `sa_q_is_fixed`
4. **追踪 DOF 变化**: 在 `host_evaluate` / `host_update_velocity` 中加 `LUISA_INFO`
5. **验证 _outer 更新**: `physics_step_post_operation` 是否正确同步
6. **对比 OBJ**: `save_result` 导出的 OBJ 直接显示顶点变化

### 关键数值

| 参数 | 值 | 计算 |
|------|-----|------|
| implicit_dt | 1/300 ≈ 0.00333s | `set_implicit_dt` |
| num_substep | 3 | `set_num_substep` |
| substep_dt | 1/900 ≈ 0.00111s | implicit_dt / num_substep |
| gravity (Z-up) | (0, 0, -9.8) | `set_up_axis(Z_UP)` |
| 单步位移 (v=0) | dt²*g ≈ 1.21e-5 m | 第一 substep |
| 收敛阈值 | 3.33e-5 | 1e-2 * implicit_dt |
| stiffness_dirichlet | 1.0 (浮动) / 1e5 (固定) | `init_sim_data.cpp:1183` |
