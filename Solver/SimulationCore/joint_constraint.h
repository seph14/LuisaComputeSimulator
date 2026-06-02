#pragma once

#include <luisa/core/basic_types.h>
#include <cstdint>
#include <cstddef>
#include <limits>

namespace lcs
{
	enum class JointConstraintType : uint32_t
	{
		Fixed,
		Prismatic,
		Revolute,
		Ball, // spherical: constrains two anchor points to coincide
		Free  // floating base: no constraint (root link of free-floating robots)
	};

	struct FixedJointConstraintDesc
	{
		uint32_t	  body_a_registration = 0u;
		uint32_t	  body_b_registration = 0u;
		luisa::float3 anchor_a_local = luisa::make_float3(0.0f);
		luisa::float3 anchor_b_local = luisa::make_float3(0.0f);
		float		  stiffness_pos = 1.0e4f;
		float		  stiffness_rot = 1.0e3f;
	};

	struct PrismaticJointConstraintDesc
	{
		uint32_t	  body_a_registration = 0u;
		uint32_t	  body_b_registration = 0u;
		luisa::float3 anchor_a_local = luisa::make_float3(0.0f);
		luisa::float3 anchor_b_local = luisa::make_float3(0.0f);
		luisa::float3 axis_world = luisa::make_float3(1.0f, 0.0f, 0.0f);
		float		  stiffness_pos = 1.0e4f;
		float		  stiffness_rot = 1.0e3f;
		// Sliding distance limits along the free axis (in world units). Use ±FLT_MAX to disable.
		float slide_min = -std::numeric_limits<float>::max();
		float slide_max = std::numeric_limits<float>::max();
	};

	struct RevoluteJointConstraintDesc
	{
		uint32_t	  body_a_registration = 0u;
		uint32_t	  body_b_registration = 0u;
		luisa::float3 anchor_a_local = luisa::make_float3(0.0f);
		luisa::float3 anchor_b_local = luisa::make_float3(0.0f);
		luisa::float3 axis_world = luisa::make_float3(1.0f, 0.0f, 0.0f);
		luisa::float3 axis_a_local = luisa::make_float3(1.0f, 0.0f, 0.0f);
		luisa::float3 axis_b_local = luisa::make_float3(1.0f, 0.0f, 0.0f);
		float		  stiffness_pos = 1.0e4f;
		float		  stiffness_axis = 1.0e3f;
		float		  lower_angle = -std::numeric_limits<float>::max();
		float		  upper_angle = std::numeric_limits<float>::max();
	};

	// Ball (spherical) joint: constrains two anchor points to coincide.
	// Allows free relative rotation between the two bodies.
	struct BallJointConstraintDesc
	{
		uint32_t	  body_a_registration = 0u;
		uint32_t	  body_b_registration = 0u;
		luisa::float3 anchor_a_local = luisa::make_float3(0.0f);
		luisa::float3 anchor_b_local = luisa::make_float3(0.0f);
		float		  stiffness_pos = 1.0e4f;
	};

	// Free joint: no constraint (used for floating base robots).
	// This is a placeholder — the body is simply registered without any joint.
	struct FreeJointConstraintDesc
	{
		uint32_t body_a_registration = 0u;
		uint32_t body_b_registration = 0u;
	};

} // namespace lcs
