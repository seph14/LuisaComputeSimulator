#pragma once

#include "Core/float_nxn.h"
#include "Core/scalar.h"
#include "Energies/detail/energy_detail_common.hpp"
#include <array>
#include <cmath>
#include <type_traits>

namespace lcs::detail::revolute_joint_constaint
{
	template <typename ScalarT, typename Vec3T, typename Mat3T>
	using RevoluteJointEvalResult = EnergyEvalResult<8, 64, Vec3T, Mat3T>;

	template <typename ScalarT, typename Vec3T>
	struct RevoluteAngleEvalResult
	{
		ScalarT				 angle = ScalarT(0.0f);
		std::array<Vec3T, 8> gradients{};
	};

	template <typename ScalarT>
	[[nodiscard]] inline ScalarT angle_atan2(const ScalarT y, const ScalarT x)
	{
		if constexpr (std::is_floating_point_v<ScalarT>)
		{
			return static_cast<ScalarT>(std::atan2(y, x));
		}
		else
		{
			return luisa::compute::atan2(y, x);
		}
	}

	template <typename ScalarT, typename Vec3T>
	[[nodiscard]] inline ScalarT compute_angle(
		const Vec3T (&q)[8],
		const Vec3T& rest_rot_col0_a_to_b,
		const Vec3T& rest_rot_col1_a_to_b,
		const Vec3T& rest_rot_col2_a_to_b,
		const Vec3T& axis_a_local)
	{
		const Vec3T rest_cols[3] = { rest_rot_col0_a_to_b, rest_rot_col1_a_to_b, rest_rot_col2_a_to_b };

		auto rest_col_in_b = [&](const int col)
		{
			return q[5] * rest_cols[col].x + q[6] * rest_cols[col].y + q[7] * rest_cols[col].z;
		};
		auto R_delta = [&](const int row, const int col)
		{
			return dot(q[1 + row], rest_col_in_b(col));
		};

		const ScalarT sx = R_delta(1, 2) - R_delta(2, 1);
		const ScalarT sy = R_delta(2, 0) - R_delta(0, 2);
		const ScalarT sz = R_delta(0, 1) - R_delta(1, 0);

		const ScalarT axis_len = sqrt_scalar(dot(axis_a_local, axis_a_local));
		const ScalarT inv_len = ScalarT(1.0f) / max_scalar(axis_len, ScalarT(1.0e-12f));
		const Vec3T	  axis_n = axis_a_local * inv_len;

		const ScalarT sin_theta = ScalarT(0.5f) * (axis_n.x * sx + axis_n.y * sy + axis_n.z * sz);
		const ScalarT cos_theta = ScalarT(0.5f) * (R_delta(0, 0) + R_delta(1, 1) + R_delta(2, 2) - ScalarT(1.0f));
		return angle_atan2(sin_theta, cos_theta);
	}

	template <typename ScalarT, typename Vec3T>
	[[nodiscard]] inline auto evaluate_angle(
		const Vec3T (&q)[8],
		const Vec3T& rest_rot_col0_a_to_b,
		const Vec3T& rest_rot_col1_a_to_b,
		const Vec3T& rest_rot_col2_a_to_b,
		const Vec3T& axis_a_local)
	{
		RevoluteAngleEvalResult<ScalarT, Vec3T> out{};
		for (auto& g : out.gradients)
		{
			g = zero3;
		}

		const Vec3T rest_cols[3] = { rest_rot_col0_a_to_b, rest_rot_col1_a_to_b, rest_rot_col2_a_to_b };
		out.angle = compute_angle<ScalarT, Vec3T>(q, rest_rot_col0_a_to_b, rest_rot_col1_a_to_b, rest_rot_col2_a_to_b, axis_a_local);

		auto rest_col_in_b = [&](const int col)
		{
			return q[5] * rest_cols[col].x + q[6] * rest_cols[col].y + q[7] * rest_cols[col].z;
		};
		auto R_delta = [&](const int row, const int col)
		{
			return dot(q[1 + row], rest_col_in_b(col));
		};

		const ScalarT sx = R_delta(1, 2) - R_delta(2, 1);
		const ScalarT sy = R_delta(2, 0) - R_delta(0, 2);
		const ScalarT sz = R_delta(0, 1) - R_delta(1, 0);
		const ScalarT axis_len = sqrt_scalar(dot(axis_a_local, axis_a_local));
		const ScalarT inv_len = ScalarT(1.0f) / max_scalar(axis_len, ScalarT(1.0e-12f));
		const Vec3T	  axis_n = axis_a_local * inv_len;
		const ScalarT sin_theta = ScalarT(0.5f) * (axis_n.x * sx + axis_n.y * sy + axis_n.z * sz);
		const ScalarT cos_theta = ScalarT(0.5f) * (R_delta(0, 0) + R_delta(1, 1) + R_delta(2, 2) - ScalarT(1.0f));
		const ScalarT inv_den = ScalarT(1.0f) / max_scalar(sin_theta * sin_theta + cos_theta * cos_theta, ScalarT(1.0e-12f));

		for (int kk = 1; kk <= 7; ++kk)
		{
			if (kk == 4)
				continue;
			Vec3T ds = zero3;
			Vec3T dc = zero3;
			if (kk >= 1 && kk <= 3)
			{
				const int row = kk - 1;
				for (int col = 0; col < 3; ++col)
				{
					const Vec3T dR = rest_col_in_b(col);
					if (col == row)
						dc += ScalarT(0.5f) * dR;

					ScalarT cx = ScalarT(0.0f), cy = ScalarT(0.0f), cz = ScalarT(0.0f);
					if (col == 2 && row == 1)
						cx = ScalarT(1.0f);
					if (col == 1 && row == 2)
						cx = ScalarT(-1.0f);
					if (col == 0 && row == 2)
						cy = ScalarT(1.0f);
					if (col == 2 && row == 0)
						cy = ScalarT(-1.0f);
					if (col == 1 && row == 0)
						cz = ScalarT(1.0f);
					if (col == 0 && row == 1)
						cz = ScalarT(-1.0f);
					const ScalarT coeff = ScalarT(0.5f) * (axis_n.x * cx + axis_n.y * cy + axis_n.z * cz);
					ds += coeff * dR;
				}
			}
			else
			{
				const int col_b = kk - 5;
				for (int row = 0; row < 3; ++row)
				{
					for (int col = 0; col < 3; ++col)
					{
						const Vec3T dR = q[1 + row] * rest_cols[col][col_b];
						if (col == row)
							dc += ScalarT(0.5f) * dR;

						ScalarT cx = ScalarT(0.0f), cy = ScalarT(0.0f), cz = ScalarT(0.0f);
						if (col == 2 && row == 1)
							cx = ScalarT(1.0f);
						if (col == 1 && row == 2)
							cx = ScalarT(-1.0f);
						if (col == 0 && row == 2)
							cy = ScalarT(1.0f);
						if (col == 2 && row == 0)
							cy = ScalarT(-1.0f);
						if (col == 1 && row == 0)
							cz = ScalarT(1.0f);
						if (col == 0 && row == 1)
							cz = ScalarT(-1.0f);
						const ScalarT coeff = ScalarT(0.5f) * (axis_n.x * cx + axis_n.y * cy + axis_n.z * cz);
						ds += coeff * dR;
					}
				}
			}
			out.gradients[kk] = (cos_theta * ds - sin_theta * dc) * inv_den;
		}

		return out;
	}

	template <typename ScalarT>
	[[nodiscard]] inline ScalarT limit_residual(const ScalarT angle, const ScalarT lower_angle, const ScalarT upper_angle)
	{
		return max_scalar(angle - upper_angle, ScalarT(0.0f)) + min_scalar(angle - lower_angle, ScalarT(0.0f));
	}

	template <typename ScalarT, typename Vec3T, typename Mat3T>
	inline void add_angle_penalty(
		RevoluteJointEvalResult<ScalarT, Vec3T, Mat3T>& out,
		const Vec3T (&q)[8],
		const Vec3T&  rest_rot_col0_a_to_b,
		const Vec3T&  rest_rot_col1_a_to_b,
		const Vec3T&  rest_rot_col2_a_to_b,
		const Vec3T&  axis_a_local,
		const ScalarT lower_angle,
		const ScalarT upper_angle,
		const ScalarT stiffness)
	{
		const auto angle_eval = evaluate_angle<ScalarT, Vec3T>(q, rest_rot_col0_a_to_b, rest_rot_col1_a_to_b, rest_rot_col2_a_to_b, axis_a_local);
		const auto residual = limit_residual(angle_eval.angle, lower_angle, upper_angle);
		const auto active = select(abs_scalar(residual) > ScalarT(0.0f), ScalarT(1.0f), ScalarT(0.0f));
		for (int i = 0; i < 8; ++i)
		{
			out.gradients[i] += stiffness * residual * angle_eval.gradients[i];
			for (int j = 0; j < 8; ++j)
			{
				out.hessians[i * 8 + j] = out.hessians[i * 8 + j]
					+ stiffness * active * outer_product(angle_eval.gradients[i], angle_eval.gradients[j]);
			}
		}
	}

	// ── Base constraint evaluate (no angle limits) ────────────────────

	template <typename ScalarT, typename Vec3T, typename Mat3T>
	[[nodiscard]] inline auto evaluate(
		const Vec3T (&q)[8],
		const Vec3T&  anchor_a_local,
		const Vec3T&  anchor_b_local,
		const Vec3T&  rest_position_delta_local_a,
		const Vec3T&  axis_a_local,
		const Vec3T&  axis_b_local,
		const ScalarT stiffness_pos,
		const ScalarT stiffness_axis,
		const Mat3T&  identity)
	{
		RevoluteJointEvalResult<ScalarT, Vec3T, Mat3T> out{};
		for (auto& g : out.gradients)
		{
			g = zero3;
		}
		for (auto& H : out.hessians)
		{
			H = zero3x3;
		}

		auto add_linear_term = [&](const Mat3T(&coeff)[8], const Vec3T& bias, const ScalarT stiffness)
		{
			Vec3T r = bias;
			for (int i = 0; i < 8; ++i)
				r += coeff[i] * q[i];
			for (int i = 0; i < 8; ++i)
			{
				out.gradients[i] += stiffness * (coeff[i] * r);
				for (int j = 0; j < 8; ++j)
					out.hessians[i * 8 + j] = out.hessians[i * 8 + j]
						+ stiffness * (coeff[i] * coeff[j]);
			}
		};

		const Mat3T I = identity;
		const Mat3T Z = 0.0f * identity;

		// Anchor delta preservation
		{
			Mat3T coeff[8] = { (-1.0f) * I,
				-(anchor_a_local.x + rest_position_delta_local_a.x) * I,
				-(anchor_a_local.y + rest_position_delta_local_a.y) * I,
				-(anchor_a_local.z + rest_position_delta_local_a.z) * I,
				I,
				anchor_b_local.x * I, anchor_b_local.y * I, anchor_b_local.z * I };
			add_linear_term(coeff, zero3, stiffness_pos);
		}

		// Hinge axis consistency
		{
			Mat3T coeff[8] = { Z,
				axis_a_local.x * I, axis_a_local.y * I, axis_a_local.z * I,
				Z,
				-axis_b_local.x * I, -axis_b_local.y * I, -axis_b_local.z * I };
			add_linear_term(coeff, zero3, stiffness_axis);
		}

		return out;
	}

	// ── Base constraint energy (no angle limits) ─────────────────────

	template <typename ScalarT, typename Vec3T, typename Mat3T>
	[[nodiscard]] inline ScalarT compute_energy(
		const Vec3T (&q)[8],
		const Vec3T&  anchor_a_local,
		const Vec3T&  anchor_b_local,
		const Vec3T&  rest_position_delta_local_a,
		const Vec3T&  axis_a_local,
		const Vec3T&  axis_b_local,
		const ScalarT stiffness_pos,
		const ScalarT stiffness_axis,
		const Mat3T& /*identity*/)
	{
		Vec3T p_a = q[0] + q[1] * anchor_a_local.x + q[2] * anchor_a_local.y + q[3] * anchor_a_local.z;
		Vec3T p_b = q[4] + q[5] * anchor_b_local.x + q[6] * anchor_b_local.y + q[7] * anchor_b_local.z;
		Vec3T target_delta = q[1] * rest_position_delta_local_a.x + q[2] * rest_position_delta_local_a.y + q[3] * rest_position_delta_local_a.z;
		Vec3T r_pos = (p_b - p_a) - target_delta;

		Vec3T axis_a_world = q[1] * axis_a_local.x + q[2] * axis_a_local.y + q[3] * axis_a_local.z;
		Vec3T axis_b_world = q[5] * axis_b_local.x + q[6] * axis_b_local.y + q[7] * axis_b_local.z;
		Vec3T r_axis = axis_a_world - axis_b_world;

		ScalarT energy = ScalarT(0.5f) * stiffness_pos * dot(r_pos, r_pos);
		energy += ScalarT(0.5f) * stiffness_axis * dot(r_axis, r_axis);
		return energy;
	}

} // namespace lcs::detail::revolute_joint_constaint
