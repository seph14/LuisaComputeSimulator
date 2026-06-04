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
				$elif(jtype == static_cast<uint>(JointConstraintType::Ball))
				{
					// Ball joint: positional anchor coincidence only
					Float3 r_pos_ball = (p_b - p_a);
					energy = 0.5f * stiff.x * dot(r_pos_ball, r_pos_ball);
				}
				$elif(jtype == static_cast<uint>(JointConstraintType::Free))
				{
					// Free joint: no constraint energy
					energy = 0.0f;
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
					const Float	 angle = detail::revolute_joint_constaint::compute_angle<Float, Float3>(q, rest_rot_c0, rest_rot_c1, rest_rot_c2, axis_a);
					const Float	 diff = detail::revolute_joint_constaint::limit_residual<Float>(angle, ang_lim.x, ang_lim.y);
					energy += 0.5f * stiff.x * diff * diff;
				};

				// ── Joint drive energy (ROADMAP 1.4) ──────────────────────
				// E_drive = 0.5 * kp * (q_cur - target)^2
				{
					const Float3 drive = joint.joint_drive_params->read(joint_idx);
					const Float	 target = drive.x;
					const Float	 kp = drive.y;
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
							q_cur = detail::revolute_joint_constaint::compute_angle<Float, Float3>(q, rest_rot_c0, rest_rot_c1, rest_rot_c2, axis_a);
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
				$elif(jtype == static_cast<uint>(JointConstraintType::Ball))
				{
					Float3 rp = q[0] + q[1] * anchor_a.x + q[2] * anchor_a.y + q[3] * anchor_a.z;
					Float3 rp2 = q[4] + q[5] * anchor_b.x + q[6] * anchor_b.y + q[7] * anchor_b.z;
					Float3 r_pos_ball = rp2 - rp;
					Float3 f_ball = stiff.x * r_pos_ball;
					Float3 gb[8];
					gb[0] = -f_ball;
					gb[1] = -anchor_a.x * f_ball;
					gb[2] = -anchor_a.y * f_ball;
					gb[3] = -anchor_a.z * f_ball;
					gb[4] = f_ball;
					gb[5] = anchor_b.x * f_ball;
					gb[6] = anchor_b.y * f_ball;
					gb[7] = anchor_b.z * f_ball;
					for (uint bi = 0; bi < 8; ++bi)
						joint.constraint_gradients->write(joint_idx * 8u + bi, gb[bi]);
					for (uint bi = 0; bi < 8; ++bi)
						for (uint bj = 0; bj < 8; ++bj)
							joint.constraint_hessians->write(joint_idx * 64u + bi * 8u + bj,
								outer_product(gb[bi], gb[bj]) / stiff.x);
				}
				$elif(jtype == static_cast<uint>(JointConstraintType::Free))
				{
					for (uint fi = 0; fi < 8; ++fi)
						joint.constraint_gradients->write(joint_idx * 8u + fi, make_float3(0.0f));
					for (uint fi = 0; fi < 8; ++fi)
						for (uint fj = 0; fj < 8; ++fj)
							joint.constraint_hessians->write(joint_idx * 64u + fi * 8u + fj, make_float3x3(0.0f));
				}
				$else
				{
					// Revolute
					auto eval = detail::revolute_joint_constaint::evaluate<Float, Float3, Float3x3>(
						q, anchor_a, anchor_b, rest_pos_delta, axis_a, axis_b, stiff.x, stiff.y, I);
					const Float2 ang_lim = joint.slide_limits->read(joint_idx);
					detail::revolute_joint_constaint::add_angle_penalty<Float, Float3, Float3x3>(
						eval, q, rest_rot_c0, rest_rot_c1, rest_rot_c2, axis_a, ang_lim.x, ang_lim.y, stiff.x);
					for (uint i = 0; i < 8; ++i)
						joint.constraint_gradients->write(joint_idx * 8u + i, eval.gradients[i]);
					for (uint i = 0; i < 8; ++i)
						for (uint j = 0; j < 8; ++j)
							joint.constraint_hessians->write(joint_idx * 64u + i * 8u + j, eval.hessians[i * 8 + j]);
				};

				// ── Joint drive energy gradient/Hessian (ROADMAP 1.4) ──────
				{
					const Float3 drive = joint.joint_drive_params->read(joint_idx);
					const Float	 kp = drive.y;
					$if(kp > 0.0f)
					{
						Float  q_cur = 0.0f;
						Float3 v[8];
						for (uint vi = 0; vi < 8; ++vi)
							v[vi] = make_float3(0.0f);

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
							const auto angle_eval = detail::revolute_joint_constaint::evaluate_angle<Float, Float3>(q, rest_rot_c0, rest_rot_c1, rest_rot_c2, axis_a);
							q_cur = angle_eval.angle;
							for (uint vi2 = 0; vi2 < 8; ++vi2)
								v[vi2] = angle_eval.gradients[vi2];
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
					else if (jtype == static_cast<uint>(JointConstraintType::Ball))
					{
						// Ball joint: positional anchor coincidence
						float3 rp = q[0] + q[1] * anchor_a.x + q[2] * anchor_a.y + q[3] * anchor_a.z;
						float3 rp2 = q[4] + q[5] * anchor_b.x + q[6] * anchor_b.y + q[7] * anchor_b.z;
						float3 r_pos_ball = rp2 - rp;
						float3 f_ball = stiff.x * r_pos_ball;
						float3 gb[8];
						gb[0] = -f_ball;
						gb[1] = -anchor_a.x * f_ball;
						gb[2] = -anchor_a.y * f_ball;
						gb[3] = -anchor_a.z * f_ball;
						gb[4] = f_ball;
						gb[5] = anchor_b.x * f_ball;
						gb[6] = anchor_b.y * f_ball;
						gb[7] = anchor_b.z * f_ball;
						for (uint bi = 0; bi < 8; ++bi)
							joint_data.constraint_gradients[joint_idx * 8u + bi] = gb[bi];
						for (uint bi = 0; bi < 8; ++bi)
							for (uint bj = 0; bj < 8; ++bj)
							{
								float3x3 outer;
								for (int col = 0; col < 3; ++col)
									outer[col] = gb[bi] * gb[bj][col];
								joint_data.constraint_hessians[joint_idx * 64u + bi * 8u + bj] = outer / stiff.x;
							}
					}
					else if (jtype == static_cast<uint>(JointConstraintType::Free))
					{
						for (uint fi = 0; fi < 8; ++fi)
							joint_data.constraint_gradients[joint_idx * 8u + fi] = luisa::make_float3(0.0f);
						for (uint fi = 0; fi < 8; ++fi)
							for (uint fj = 0; fj < 8; ++fj)
								joint_data.constraint_hessians[joint_idx * 64u + fi * 8u + fj] = luisa::make_float3x3(0.0f);
					}
					else
					{
						// Revolute
						auto eval = detail::revolute_joint_constaint::evaluate<float, float3, float3x3>(
							q, anchor_a, anchor_b, rest_pos_delta, axis_a, axis_b, stiff.x, stiff.y, I);
						const float2 ang_lim = joint_data.slide_limits[joint_idx];
						detail::revolute_joint_constaint::add_angle_penalty<float, float3, float3x3>(
							eval, q, rest_rot_c0, rest_rot_c1, rest_rot_c2, axis_a, ang_lim.x, ang_lim.y, stiff.x);
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
						const float	 kp = drive.y;
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
								const auto angle_eval = detail::revolute_joint_constaint::evaluate_angle<float, float3>(q, rest_rot_c0, rest_rot_c1, rest_rot_c2, axis_a);
								q_cur = angle_eval.angle;
								for (int i = 0; i < 8; ++i)
									v[i] = angle_eval.gradients[i];
							}

							float residual = q_cur - drive.x;
							for (uint i = 0; i < 8; ++i)
							{
								joint_data.constraint_gradients[joint_idx * 8u + i] = joint_data.constraint_gradients[joint_idx * 8u + i] + kp * residual * v[i];
								for (uint j = 0; j < 8; ++j)
								{
									float3x3 h_old = joint_data.constraint_hessians[joint_idx * 64u + i * 8u + j];
									float3x3 outer;
									for (int col = 0; col < 3; ++col)
										outer[col] = v[i] * v[j][col];
									joint_data.constraint_hessians[joint_idx * 64u + i * 8u + j] = h_old + kp * outer;
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
