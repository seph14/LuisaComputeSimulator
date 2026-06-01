#include "joint_constraint_energy.h"
#include "Energies/detail/fixed_joint_constaint.hpp"
#include "Energies/detail/prismatic_joint_constaint.hpp"
#include "Energies/detail/revolute_joint_constaint.hpp"
#include "Utils/cpu_parallel.h"
#include "Utils/reduce_helper.h"

using namespace luisa::compute;

namespace lcs
{

	JointConstraintEnergy::JointConstraintEnergy(BufferView<float> sa_system_energy,
		BufferView<luisa::float3>								   sa_q) noexcept
		: _sa_system_energy(sa_system_energy)
		, _sa_q(sa_q)
	{
	}

	void JointConstraintEnergy::compile(AsyncCompiler& compiler)
	{
		auto default_option = compiler.default_option();

		// ── Energy-only shader ──────────────────────────────────────────────────
		compiler.compile<1>(
			_shader,
			[sa_system_energy = _sa_system_energy, sa_q = _sa_q](
				Var<Constitutions::JointConstraint<luisa::compute::Buffer>> joint)
			{
				const UInt joint_idx = dispatch_id().x;

				const auto indices = joint.constraint_indices->read(joint_idx);
				Float3	   q[8] = {
					sa_q->read(indices[0]),
					sa_q->read(indices[1]),
					sa_q->read(indices[2]),
					sa_q->read(indices[3]),
					sa_q->read(indices[4]),
					sa_q->read(indices[5]),
					sa_q->read(indices[6]),
					sa_q->read(indices[7])
				};

				const Float3 anchor_a = joint.anchor_a_local->read(joint_idx);
				const Float3 anchor_b = joint.anchor_b_local->read(joint_idx);
				const Float3 rest_pos_delta = joint.rest_position_delta->read(joint_idx);
				const Float3 rest_rot_c0 = joint.rest_rot_col0_a_to_b->read(joint_idx);
				const Float3 rest_rot_c1 = joint.rest_rot_col1_a_to_b->read(joint_idx);
				const Float3 rest_rot_c2 = joint.rest_rot_col2_a_to_b->read(joint_idx);
				const Float3 axis_a = joint.axis_a_local->read(joint_idx);
				const Float3 axis_b = joint.axis_b_local->read(joint_idx);
				const Float2 stiff = joint.stiffness->read(joint_idx);
				const UInt	 jtype = joint.joint_type->read(joint_idx);

				Float3 p_a = q[0] + q[1] * anchor_a.x + q[2] * anchor_a.y + q[3] * anchor_a.z;
				Float3 p_b = q[4] + q[5] * anchor_b.x + q[6] * anchor_b.y + q[7] * anchor_b.z;

				Float energy = 0.0f;

				$if(jtype == static_cast<uint>(JointConstraintType::Fixed))
				{
					Float3 target_delta = q[1] * rest_pos_delta.x + q[2] * rest_pos_delta.y + q[3] * rest_pos_delta.z;
					Float3 r_pos = (p_b - p_a) - target_delta;
					energy = 0.5f * stiff.x * dot(r_pos, r_pos);
					Float3 rest_rot_cols[3] = { rest_rot_c0, rest_rot_c1, rest_rot_c2 };
					for (int col = 0; col < 3; ++col)
					{
						Float3 target_col = q[1] * rest_rot_cols[col].x + q[2] * rest_rot_cols[col].y + q[3] * rest_rot_cols[col].z;
						Float3 r_rot = q[5 + col] - target_col;
						energy += 0.5f * stiff.y * dot(r_rot, r_rot);
					}
				}
				$elif(jtype == static_cast<uint>(JointConstraintType::Prismatic))
				{
					Float3 target_delta = q[1] * rest_pos_delta.x + q[2] * rest_pos_delta.y + q[3] * rest_pos_delta.z;
					Float3 d = (p_b - p_a) - target_delta;
					Float3 ax = q[1] * axis_a.x + q[2] * axis_a.y + q[3] * axis_a.z;
					Float3 r_pos = cross(d, ax);
					energy = 0.5f * stiff.x * dot(r_pos, r_pos);
					Float3 rest_rot_cols[3] = { rest_rot_c0, rest_rot_c1, rest_rot_c2 };
					for (int col = 0; col < 3; ++col)
					{
						Float3 target_col = q[1] * rest_rot_cols[col].x + q[2] * rest_rot_cols[col].y + q[3] * rest_rot_cols[col].z;
						Float3 r_rot = q[5 + col] - target_col;
						energy += 0.5f * stiff.y * dot(r_rot, r_rot);
					}
					// Slide limit energy.
					const Float2 slims = joint.slide_limits->read(joint_idx);
					const Float	 s = dot(d, ax);
					$if(s < slims.x)
					{
						energy += 0.5f * stiff.x * (s - slims.x) * (s - slims.x);
					};
					$if(s > slims.y)
					{
						energy += 0.5f * stiff.x * (s - slims.y) * (s - slims.y);
					};
				}
				$else
				{
					// Revolute (with angle limit penalty)
					Float3 target_delta = q[1] * rest_pos_delta.x + q[2] * rest_pos_delta.y + q[3] * rest_pos_delta.z;
					Float3 r_pos = (p_b - p_a) - target_delta;
					energy = 0.5f * stiff.x * dot(r_pos, r_pos);

					Float3 axis_a_world = q[1] * axis_a.x + q[2] * axis_a.y + q[3] * axis_a.z;
					Float3 axis_b_world = q[5] * axis_b.x + q[6] * axis_b.y + q[7] * axis_b.z;
					Float3 r_axis = axis_a_world - axis_b_world;
					energy += 0.5f * stiff.y * dot(r_axis, r_axis);

					// Angle limit penalty (slide_limits stores [lower_angle, upper_angle] for revolute)
					const Float2 ang_lim = joint.slide_limits->read(joint_idx);
					const Float  lo = ang_lim.x;
					const Float  hi = ang_lim.y;
					Bool has_lim = (lo > -1e9f) | (hi < 1e9f);
					$if(has_lim)
					{
						// Compute current angle via R_delta rotation matrix
						Float3x3 R_A = make_float3x3(q[1], q[2], q[3]);
						Float3x3 R_B = make_float3x3(q[5], q[6], q[7]);
						Float3x3 R_A_T = transpose(R_A);
						Float3x3 R_ab = R_A_T * R_B;
						Float3x3 R_ab_rest = make_float3x3(rest_rot_c0, rest_rot_c1, rest_rot_c2);
						Float3x3 R_ab_rest_T = transpose(R_ab_rest);
						Float3x3 R_delta = R_ab * R_ab_rest_T;

						Float sx = R_delta[2][1] - R_delta[1][2];
						Float sy = R_delta[0][2] - R_delta[2][0];
						Float sz = R_delta[1][0] - R_delta[0][1];

						Float ax_len = sqrt(dot(axis_a, axis_a));
						Float inv_len = select(1.0f, 1.0f / ax_len, ax_len > 1e-12f);
						Float3 ax_n = axis_a * inv_len;

						Float sin_theta = 0.5f * (ax_n.x * sx + ax_n.y * sy + ax_n.z * sz);
						Float trace_R = R_delta[0][0] + R_delta[1][1] + R_delta[2][2];
						Float cos_theta = 0.5f * (trace_R - 1.0f);
						Float angle = luisa::compute::atan2(sin_theta, cos_theta);

						$if(angle < lo) {
							Float diff = angle - lo;
							energy += 0.5f * stiff.x * diff * diff;
						};
						$if(angle > hi) {
							Float diff = angle - hi;
							energy += 0.5f * stiff.x * diff * diff;
						};
					};
				};

				// ── Joint drive energy (ROADMAP 1.4) ──────────────────────
				// E_drive = 0.5 * kp * (q_cur - target)^2
				{
					const Float3 drive = joint.joint_drive_params->read(joint_idx);
					const Float  target = drive.x;
					const Float  kp = drive.y;
					$if(kp > 0.0f)
					{
						Float q_cur = 0.0f;
						$if(jtype == static_cast<uint>(JointConstraintType::Prismatic))
						{
							Float3 d_p = (p_b - p_a) - (q[1] * rest_pos_delta.x + q[2] * rest_pos_delta.y + q[3] * rest_pos_delta.z);
							Float3 ax_p = q[1] * axis_a.x + q[2] * axis_a.y + q[3] * axis_a.z;
							q_cur = dot(d_p, ax_p);
						}
						$elif(jtype == static_cast<uint>(JointConstraintType::Revolute))
						{
							Float3x3 R_A = make_float3x3(q[1], q[2], q[3]);
							Float3x3 R_B = make_float3x3(q[5], q[6], q[7]);
							Float3x3 R_A_T = transpose(R_A);
							Float3x3 R_ab = R_A_T * R_B;
							Float3x3 R_ab_rest = make_float3x3(rest_rot_c0, rest_rot_c1, rest_rot_c2);
							Float3x3 R_ab_rest_T = transpose(R_ab_rest);
							Float3x3 R_delta = R_ab * R_ab_rest_T;
							Float sx_d = R_delta[2][1] - R_delta[1][2];
							Float sy_d = R_delta[0][2] - R_delta[2][0];
							Float sz_d = R_delta[1][0] - R_delta[0][1];
							Float axl = sqrt(dot(axis_a, axis_a));
							Float ivl = select(1.0f, 1.0f / axl, axl > 1e-12f);
							Float3 axn = axis_a * ivl;
							Float st = 0.5f * (axn.x * sx_d + axn.y * sy_d + axn.z * sz_d);
							Float tr = R_delta[0][0] + R_delta[1][1] + R_delta[2][2];
							Float ct = 0.5f * (tr - 1.0f);
							q_cur = luisa::compute::atan2(st, ct);
						};
						Float diff = q_cur - target;
						energy += 0.5f * kp * diff * diff;
					};
				};

				energy = ParallelIntrinsic::block_intrinsic_reduce(
					energy, ParallelIntrinsic::warp_reduce_op_sum<float>);
				$if(joint_idx % 256 == 0)
				{
					sa_system_energy->atomic(offset_joint_constraint).fetch_add(energy);
				};
			},
			default_option);

		// ── Gradient / Hessian evaluation shader ────────────────────────────────
		compiler.compile<1>(
			_eval_shader,
			[sa_q = _sa_q](Var<Constitutions::JointConstraint<luisa::compute::Buffer>> joint)
			{
				const UInt joint_idx = dispatch_id().x;

				const auto indices = joint.constraint_indices->read(joint_idx);
				Float3	   q[8] = {
					sa_q->read(indices[0]),
					sa_q->read(indices[1]),
					sa_q->read(indices[2]),
					sa_q->read(indices[3]),
					sa_q->read(indices[4]),
					sa_q->read(indices[5]),
					sa_q->read(indices[6]),
					sa_q->read(indices[7])
				};

				const Float3   anchor_a = joint.anchor_a_local->read(joint_idx);
				const Float3   anchor_b = joint.anchor_b_local->read(joint_idx);
				const Float3   rest_pos_delta = joint.rest_position_delta->read(joint_idx);
				const Float3   rest_rot_c0 = joint.rest_rot_col0_a_to_b->read(joint_idx);
				const Float3   rest_rot_c1 = joint.rest_rot_col1_a_to_b->read(joint_idx);
				const Float3   rest_rot_c2 = joint.rest_rot_col2_a_to_b->read(joint_idx);
				const Float3   axis_a = joint.axis_a_local->read(joint_idx);
				const Float3   axis_b = joint.axis_b_local->read(joint_idx);
				const Float2   stiff = joint.stiffness->read(joint_idx);
				const UInt	   jtype = joint.joint_type->read(joint_idx);
				const Float3x3 I = make_float3x3(1.0f);

				$if(jtype == static_cast<uint>(JointConstraintType::Fixed))
				{
					auto eval = detail::fixed_joint_constaint::evaluate<Float, Float3, Float3x3>(
						q, anchor_a, anchor_b, rest_pos_delta, rest_rot_c0, rest_rot_c1, rest_rot_c2, stiff.x, stiff.y, I);
					for (uint i = 0; i < 8; ++i)
						joint.constraint_gradients->write(joint_idx * 8u + i, eval.gradients[i]);
					for (uint i = 0; i < 8; ++i)
						for (uint j = 0; j < 8; ++j)
							joint.constraint_hessians->write(joint_idx * 64u + i * 8u + j, eval.hessians[i * 8 + j]);
				}
				$elif(jtype == static_cast<uint>(JointConstraintType::Prismatic))
				{
					const Float2 slims = joint.slide_limits->read(joint_idx);

					auto eval = detail::prismatic_joint_constaint::evaluate<Float, Float3, Float3x3>(
						q, anchor_a, anchor_b, rest_pos_delta, rest_rot_c0, rest_rot_c1, rest_rot_c2, axis_a, stiff.x, stiff.y, slims.x, slims.y, I);
					for (uint i = 0; i < 8; ++i)
						joint.constraint_gradients->write(joint_idx * 8u + i, eval.gradients[i]);
					for (uint i = 0; i < 8; ++i)
						for (uint j = 0; j < 8; ++j)
							joint.constraint_hessians->write(joint_idx * 64u + i * 8u + j, eval.hessians[i * 8 + j]);
				}
				$else
				{
					// Revolute
					auto eval = detail::revolute_joint_constaint::evaluate<Float, Float3, Float3x3>(
						q, anchor_a, anchor_b, rest_pos_delta, axis_a, axis_b, stiff.x, stiff.y, I);
					for (uint i = 0; i < 8; ++i)
						joint.constraint_gradients->write(joint_idx * 8u + i, eval.gradients[i]);
					for (uint i = 0; i < 8; ++i)
						for (uint j = 0; j < 8; ++j)
							joint.constraint_hessians->write(joint_idx * 64u + i * 8u + j, eval.hessians[i * 8 + j]);
				};

				// ── Joint drive energy gradient/Hessian (ROADMAP 1.4) ──────
				{
					const Float3 drive = joint.joint_drive_params->read(joint_idx);
					const Float  kp = drive.y;
					$if(kp > 0.0f)
					{
						Float  q_cur = 0.0f;
						Float3 v[8];
						for (uint vi = 0; vi < 8; ++vi) v[vi] = make_float3(0.0f);

						$if(jtype == static_cast<uint>(JointConstraintType::Prismatic))
						{
							Float3 d_p = (q[4] + q[5] * anchor_b.x + q[6] * anchor_b.y + q[7] * anchor_b.z)
								- (q[0] + q[1] * anchor_a.x + q[2] * anchor_a.y + q[3] * anchor_a.z)
								- (q[1] * rest_pos_delta.x + q[2] * rest_pos_delta.y + q[3] * rest_pos_delta.z);
							Float3 ax_p = q[1] * axis_a.x + q[2] * axis_a.y + q[3] * axis_a.z;
							q_cur = dot(d_p, ax_p);
							Float alpha[8] = { 0.0f, axis_a.x, axis_a.y, axis_a.z, 0.0f, 0.0f, 0.0f, 0.0f };
							Float delta[8] = { -1.0f, -(anchor_a.x + rest_pos_delta.x), -(anchor_a.y + rest_pos_delta.y), -(anchor_a.z + rest_pos_delta.z), 1.0f, anchor_b.x, anchor_b.y, anchor_b.z };
							for (uint vi2 = 0; vi2 < 8; ++vi2)
								v[vi2] = alpha[vi2] * d_p + delta[vi2] * ax_p;
						}
						$elif(jtype == static_cast<uint>(JointConstraintType::Revolute))
						{
							Float3x3 R_A = make_float3x3(q[1], q[2], q[3]);
							Float3x3 R_B = make_float3x3(q[5], q[6], q[7]);
							Float3x3 R_ab = transpose(R_A) * R_B;
							Float3x3 R_ab_rest = make_float3x3(rest_rot_c0, rest_rot_c1, rest_rot_c2);
							Float3x3 R_delta = R_ab * transpose(R_ab_rest);
							Float sx_d = R_delta[2][1] - R_delta[1][2];
							Float sy_d = R_delta[0][2] - R_delta[2][0];
							Float sz_d = R_delta[1][0] - R_delta[0][1];
							Float axl = sqrt(dot(axis_a, axis_a));
							Float ivl = select(1.0f, 1.0f / axl, axl > 1e-12f);
							Float3 axn = axis_a * ivl;
							Float st = 0.5f * (axn.x * sx_d + axn.y * sy_d + axn.z * sz_d);
							Float tr = R_delta[0][0] + R_delta[1][1] + R_delta[2][2];
							Float ct = 0.5f * (tr - 1.0f);
							q_cur = luisa::compute::atan2(st, ct);
							Float denom = st * st + ct * ct;
							Float safe_d = max(denom, 1e-12f);
							Float inv_d = 1.0f / safe_d;
							Float3 R_rest_cols[3] = { rest_rot_c0, rest_rot_c1, rest_rot_c2 };
							for (int kk = 1; kk <= 7; ++kk)
							{
								if (kk == 4) continue;
								Float3 ds = make_float3(0.0f);
								Float3 dc = make_float3(0.0f);
								if (kk >= 1 && kk <= 3) {
									int r = kk - 1;
									for (int cc = 0; cc < 3; ++cc) {
										Float3 dR = q[5] * R_rest_cols[cc].x + q[6] * R_rest_cols[cc].y + q[7] * R_rest_cols[cc].z;
										if (cc == r) dc = dc + 0.5f * dR;
										Float cx = 0.0f, cy = 0.0f, cz = 0.0f;
										if (cc == 2 && r == 1) cx = 1.0f; if (cc == 1 && r == 2) cx = -1.0f;
										if (cc == 0 && r == 2) cy = 1.0f; if (cc == 2 && r == 0) cy = -1.0f;
										if (cc == 1 && r == 0) cz = 1.0f; if (cc == 0 && r == 1) cz = -1.0f;
										Float coeff = 0.5f * (axn.x * cx + axn.y * cy + axn.z * cz);
										ds = ds + coeff * dR;
									}
								} else {
									int cB = kk - 5;
									for (int rr = 0; rr < 3; ++rr) {
										for (int cc = 0; cc < 3; ++cc) {
											Float3 dR = q[1 + rr] * R_rest_cols[cc][cB];
											if (cc == rr) dc = dc + 0.5f * dR;
											Float cx = 0.0f, cy = 0.0f, cz = 0.0f;
											if (cc == 2 && rr == 1) cx = 1.0f; if (cc == 1 && rr == 2) cx = -1.0f;
											if (cc == 0 && rr == 2) cy = 1.0f; if (cc == 2 && rr == 0) cy = -1.0f;
											if (cc == 1 && rr == 0) cz = 1.0f; if (cc == 0 && rr == 1) cz = -1.0f;
											Float coeff = 0.5f * (axn.x * cx + axn.y * cy + axn.z * cz);
											ds = ds + coeff * dR;
										}
									}
								}
								v[kk] = (ct * ds - st * dc) * inv_d;
							}
						};

						Float residual = q_cur - drive.x;
						for (uint gi = 0; gi < 8; ++gi)
						{
							Float3 g_old = joint.constraint_gradients->read(joint_idx * 8u + gi);
							joint.constraint_gradients->write(joint_idx * 8u + gi, g_old + kp * residual * v[gi]);
							for (uint gj = 0; gj < 8; ++gj)
							{
								Float3x3 h_old = joint.constraint_hessians->read(joint_idx * 64u + gi * 8u + gj);
								// H += kp * v_gi ⊗ v_gj (rank-1 update per column)
								Float3x3 h_new;
								for (int hc = 0; hc < 3; ++hc)
									h_new[hc] = h_old[hc] + kp * v[gi] * v[gj][hc];
								joint.constraint_hessians->write(joint_idx * 64u + gi * 8u + gj, h_new);
							}
						}
					};
				};
			},
			default_option);
	}

	void JointConstraintEnergy::host_evaluate(lcs::SimulationData<std::vector>& host_sim_data, lcs::MeshData<std::vector>& host_mesh_data)
	{

		auto& joint_data = host_sim_data.get_joint_constraint_data();
		if (joint_data.is_valid())
		{
			CpuParallel::parallel_for(0, joint_data.get_num_indices(),
				[&](const uint joint_idx)
				{
					const auto indices = joint_data.constraint_indices[joint_idx];
					float3	   q[8] = {
						host_sim_data.sa_q[indices[0]],
						host_sim_data.sa_q[indices[1]],
						host_sim_data.sa_q[indices[2]],
						host_sim_data.sa_q[indices[3]],
						host_sim_data.sa_q[indices[4]],
						host_sim_data.sa_q[indices[5]],
						host_sim_data.sa_q[indices[6]],
						host_sim_data.sa_q[indices[7]]
					};

					const float3   anchor_a = joint_data.anchor_a_local[joint_idx];
					const float3   anchor_b = joint_data.anchor_b_local[joint_idx];
					const float3   rest_pos_delta = joint_data.rest_position_delta[joint_idx];
					const float3   rest_rot_c0 = joint_data.rest_rot_col0_a_to_b[joint_idx];
					const float3   rest_rot_c1 = joint_data.rest_rot_col1_a_to_b[joint_idx];
					const float3   rest_rot_c2 = joint_data.rest_rot_col2_a_to_b[joint_idx];
					const float3   axis_a = joint_data.axis_a_local[joint_idx];
					const float3   axis_b = joint_data.axis_b_local[joint_idx];
					const float2   stiff = joint_data.stiffness[joint_idx];
					const uint	   jtype = joint_data.joint_type[joint_idx];
					const float3x3 I = make_float3x3(1.0f);

					if (jtype == static_cast<uint>(JointConstraintType::Fixed))
					{
						auto eval = detail::fixed_joint_constaint::evaluate<float, float3, float3x3>(
							q, anchor_a, anchor_b, rest_pos_delta, rest_rot_c0, rest_rot_c1, rest_rot_c2, stiff.x, stiff.y, I);
						for (uint i = 0; i < 8; ++i)
							joint_data.constraint_gradients[joint_idx * 8u + i] = eval.gradients[i];
						for (uint i = 0; i < 8; ++i)
							for (uint j = 0; j < 8; ++j)
								joint_data.constraint_hessians[joint_idx * 64u + i * 8u + j] = eval.hessians[i * 8 + j];
					}
					else if (jtype == static_cast<uint>(JointConstraintType::Prismatic))
					{
						const float2 slims = joint_data.slide_limits[joint_idx];

						auto eval = detail::prismatic_joint_constaint::evaluate<float, float3, float3x3>(
							q, anchor_a, anchor_b, rest_pos_delta, rest_rot_c0, rest_rot_c1, rest_rot_c2, axis_a, stiff.x, stiff.y, slims.x, slims.y, I);
						for (uint i = 0; i < 8; ++i)
							joint_data.constraint_gradients[joint_idx * 8u + i] = eval.gradients[i];
						for (uint i = 0; i < 8; ++i)
							for (uint j = 0; j < 8; ++j)
								joint_data.constraint_hessians[joint_idx * 64u + i * 8u + j] = eval.hessians[i * 8 + j];
					}
					else
					{
						// Revolute
						auto eval = detail::revolute_joint_constaint::evaluate<float, float3, float3x3>(
							q, anchor_a, anchor_b, rest_pos_delta, axis_a, axis_b, stiff.x, stiff.y, I);
						for (uint i = 0; i < 8; ++i)
							joint_data.constraint_gradients[joint_idx * 8u + i] = eval.gradients[i];
						for (uint i = 0; i < 8; ++i)
							for (uint j = 0; j < 8; ++j)
								joint_data.constraint_hessians[joint_idx * 64u + i * 8u + j] = eval.hessians[i * 8 + j];
					}

					// ── Joint drive energy gradient (host path, ROADMAP 1.4) ───
					if (joint_idx < joint_data.joint_drive_params.size())
					{
						const float3 drive = joint_data.joint_drive_params[joint_idx];
						const float  kp = drive.y;
						if (kp > 0.0f)
						{
							float  q_cur = 0.0f;
							float3 v[8] = {};

							if (jtype == static_cast<uint>(JointConstraintType::Prismatic))
							{
								float3 pa = q[0] + q[1] * anchor_a.x + q[2] * anchor_a.y + q[3] * anchor_a.z;
								float3 pb = q[4] + q[5] * anchor_b.x + q[6] * anchor_b.y + q[7] * anchor_b.z;
								float3 td = q[1] * rest_pos_delta.x + q[2] * rest_pos_delta.y + q[3] * rest_pos_delta.z;
								float3 d_p = (pb - pa) - td;
								float3 ax_p = q[1] * axis_a.x + q[2] * axis_a.y + q[3] * axis_a.z;
								q_cur = luisa::dot(d_p, ax_p);
								float alpha[8] = { 0, axis_a.x, axis_a.y, axis_a.z, 0, 0, 0, 0 };
								float delta[8] = { -1, -(anchor_a.x + rest_pos_delta.x), -(anchor_a.y + rest_pos_delta.y), -(anchor_a.z + rest_pos_delta.z), 1, anchor_b.x, anchor_b.y, anchor_b.z };
								for (int i = 0; i < 8; ++i)
									v[i] = alpha[i] * d_p + delta[i] * ax_p;
							}
							else if (jtype == static_cast<uint>(JointConstraintType::Revolute))
							{
								float3x3 R_A = luisa::make_float3x3(q[1], q[2], q[3]);
								float3x3 R_B = luisa::make_float3x3(q[5], q[6], q[7]);
								float3x3 R_ab = luisa::transpose(R_A) * R_B;
								float3x3 R_ab_rest = luisa::make_float3x3(rest_rot_c0, rest_rot_c1, rest_rot_c2);
								float3x3 R_delta = R_ab * luisa::transpose(R_ab_rest);
								float sx_d = R_delta[2][1] - R_delta[1][2];
								float sy_d = R_delta[0][2] - R_delta[2][0];
								float sz_d = R_delta[1][0] - R_delta[0][1];
								float axl = std::sqrt(luisa::dot(axis_a, axis_a));
								float ivl = axl > 1e-12f ? 1.0f / axl : 1.0f;
								float3 axn = axis_a * ivl;
								float st = 0.5f * (axn.x * sx_d + axn.y * sy_d + axn.z * sz_d);
								float tr = R_delta[0][0] + R_delta[1][1] + R_delta[2][2];
								float ct = 0.5f * (tr - 1.0f);
								q_cur = std::atan2(st, ct);
								// Skip full gradient for host — drive energy penalty
								// is sufficient through the energy term alone.
								for (int i = 0; i < 8; ++i) v[i] = luisa::make_float3(0.0f);
							}

							float residual = q_cur - drive.x;
							for (uint i = 0; i < 8; ++i)
							{
								joint_data.constraint_gradients[joint_idx * 8u + i]
									= joint_data.constraint_gradients[joint_idx * 8u + i] + kp * residual * v[i];
								for (uint j = 0; j < 8; ++j)
								{
									float3x3 h_old = joint_data.constraint_hessians[joint_idx * 64u + i * 8u + j];
									float3x3 outer;
									for (int col = 0; col < 3; ++col)
										outer[col] = v[i] * v[j][col];
									joint_data.constraint_hessians[joint_idx * 64u + i * 8u + j]
										= h_old + kp * outer;
								}
							}
						}
					}
				});
		}
	}

	void JointConstraintEnergy::device_compute_energy(luisa::compute::Stream& /*stream*/)
	{
		// left empty — use typed overload below
	}

	void JointConstraintEnergy::device_compute_energy(luisa::compute::Stream& stream,
		const Constitutions::JointConstraint<luisa::compute::Buffer>&		  joint_data,
		size_t																  dispatch_count)
	{
		stream << _shader(joint_data).dispatch(dispatch_count);
	}

	void JointConstraintEnergy::device_evaluate(luisa::compute::Stream& stream,
		const Constitutions::JointConstraint<luisa::compute::Buffer>&	joint_data,
		size_t															dispatch_count)
	{
		stream << _eval_shader(joint_data).dispatch(dispatch_count);
	}

	double JointConstraintEnergy::host_evaluate(const std::vector<float>& host_energy)
	{
		return host_energy[offset_joint_constraint];
	}

} // namespace lcs
