#include "Initializer/init_collision_data.h"
#include "CollisionDetector/cipc_kernel.hpp"
#include "CollisionDetector/distance.hpp"
#include "SimulationCore/scene_params.h"
#include "Utils/cpu_parallel.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace lcs::Initializer
{
	namespace
	{
		struct CpuAabb
		{
			float3 min = luisa::make_float3(std::numeric_limits<float>::max());
			float3 max = luisa::make_float3(std::numeric_limits<float>::lowest());
		};

		struct CpuBvhPrimitive
		{
			uint	face_id = 0u;
			uint	mesh_id = 0u;
			CpuAabb aabb;
			float3	centroid = luisa::make_float3(0.0f);
		};

		struct CpuBvhNode
		{
			CpuAabb aabb;
			int		left = -1;
			int		right = -1;
			uint	begin = 0u;
			uint	end = 0u;

			bool is_leaf() const { return left < 0 && right < 0; }
		};

		static float component(const float3& v, const uint axis)
		{
			return axis == 0u ? v.x : (axis == 1u ? v.y : v.z);
		}

		static CpuAabb make_empty_aabb()
		{
			return {};
		}

		static void expand_aabb(CpuAabb& aabb, const float3& p)
		{
			aabb.min = min_vec(aabb.min, p);
			aabb.max = max_vec(aabb.max, p);
		}

		static CpuAabb merge_aabb(const CpuAabb& a, const CpuAabb& b)
		{
			CpuAabb out;
			out.min = min_vec(a.min, b.min);
			out.max = max_vec(a.max, b.max);
			return out;
		}

		static bool overlaps_aabb(const CpuAabb& a, const CpuAabb& b)
		{
			return a.min.x <= b.max.x && a.max.x >= b.min.x
				&& a.min.y <= b.max.y && a.max.y >= b.min.y
				&& a.min.z <= b.max.z && a.max.z >= b.min.z;
		}

		static CpuAabb make_face_aabb(const MeshData<std::vector>* mesh_data, const uint face_id)
		{
			const uint3 face = mesh_data->sa_faces[face_id];
			CpuAabb		out = make_empty_aabb();
			expand_aabb(out, mesh_data->sa_rest_x[face[0]]);
			expand_aabb(out, mesh_data->sa_rest_x[face[1]]);
			expand_aabb(out, mesh_data->sa_rest_x[face[2]]);
			return out;
		}

		static Eigen::Vector3f to_eigen3f(const float3& v)
		{
			return Eigen::Vector3f(v.x, v.y, v.z);
		}

		static float point_triangle_distance_sq(const float3& p,
			const float3&									  t0,
			const float3&									  t1,
			const float3&									  t2)
		{
			return host_distance::point_triangle_distance_squared_unclassified(
				to_eigen3f(p), to_eigen3f(t0), to_eigen3f(t1), to_eigen3f(t2));
		}

		static float edge_edge_distance_sq(const float3& a0,
			const float3&								 a1,
			const float3&								 b0,
			const float3&								 b1)
		{
			return host_distance::edge_edge_distance_squared_unclassified(
				to_eigen3f(a0), to_eigen3f(a1), to_eigen3f(b0), to_eigen3f(b1));
		}

		static bool triangles_overlap_or_touch(const MeshData<std::vector>* mesh_data,
			const uint														face_a_id,
			const uint														face_b_id)
		{
			constexpr float overlap_eps_sq = 1.0e-12f;
			const uint3		face_a = mesh_data->sa_faces[face_a_id];
			const uint3		face_b = mesh_data->sa_faces[face_b_id];
			const float3	a[3] = {
				   mesh_data->sa_rest_x[face_a[0]],
				   mesh_data->sa_rest_x[face_a[1]],
				   mesh_data->sa_rest_x[face_a[2]],
			};
			const float3 b[3] = {
				mesh_data->sa_rest_x[face_b[0]],
				mesh_data->sa_rest_x[face_b[1]],
				mesh_data->sa_rest_x[face_b[2]],
			};

			for (uint i = 0u; i < 3u; ++i)
			{
				if (point_triangle_distance_sq(a[i], b[0], b[1], b[2]) <= overlap_eps_sq)
					return true;
				if (point_triangle_distance_sq(b[i], a[0], a[1], a[2]) <= overlap_eps_sq)
					return true;
			}

			for (uint i = 0u; i < 3u; ++i)
			{
				const float3& a0 = a[i];
				const float3& a1 = a[(i + 1u) % 3u];
				for (uint j = 0u; j < 3u; ++j)
				{
					const float3& b0 = b[j];
					const float3& b1 = b[(j + 1u) % 3u];
					if (edge_edge_distance_sq(a0, a1, b0, b1) <= overlap_eps_sq)
						return true;
				}
			}
			return false;
		}

		class CpuBvh
		{
		public:
			std::vector<CpuBvhPrimitive> primitives;
			std::vector<CpuBvhNode>		 nodes;
			int							 root = -1;

			void build()
			{
				nodes.clear();
				root = primitives.empty() ? -1 : build_node(0u, static_cast<uint>(primitives.size()));
			}

		private:
			int build_node(const uint begin, const uint end)
			{
				CpuBvhNode node;
				node.begin = begin;
				node.end = end;
				for (uint i = begin; i < end; ++i)
				{
					node.aabb = (i == begin) ? primitives[i].aabb : merge_aabb(node.aabb, primitives[i].aabb);
				}

				const uint node_idx = static_cast<uint>(nodes.size());
				nodes.push_back(node);

				constexpr uint leaf_size = 8u;
				if (end - begin <= leaf_size)
				{
					return static_cast<int>(node_idx);
				}

				CpuAabb centroid_aabb = make_empty_aabb();
				for (uint i = begin; i < end; ++i)
				{
					expand_aabb(centroid_aabb, primitives[i].centroid);
				}
				const float3 extent = centroid_aabb.max - centroid_aabb.min;
				uint		 axis = 0u;
				if (extent.y > extent.x && extent.y >= extent.z)
					axis = 1u;
				else if (extent.z > extent.x && extent.z > extent.y)
					axis = 2u;

				const uint mid = begin + (end - begin) / 2u;
				std::nth_element(primitives.begin() + begin,
					primitives.begin() + mid,
					primitives.begin() + end,
					[axis](const CpuBvhPrimitive& a, const CpuBvhPrimitive& b)
					{ return component(a.centroid, axis) < component(b.centroid, axis); });

				nodes[node_idx].left = build_node(begin, mid);
				nodes[node_idx].right = build_node(mid, end);
				nodes[node_idx].aabb = merge_aabb(nodes[nodes[node_idx].left].aabb, nodes[nodes[node_idx].right].aabb);
				return static_cast<int>(node_idx);
			}
		};

		static void check_primitive_pair(const std::vector<WorldData>& world_data,
			const MeshData<std::vector>*							   mesh_data,
			const SimulationData<std::vector>*						   sim_data,
			const CpuBvhPrimitive&									   a,
			const CpuBvhPrimitive&									   b)
		{
			if (a.face_id >= b.face_id || a.mesh_id == b.mesh_id || !overlaps_aabb(a.aabb, b.aabb))
				return;
			const uint a_prefix_vid = mesh_data->prefix_num_verts[a.mesh_id];
			const uint b_prefix_vid = mesh_data->prefix_num_verts[b.mesh_id];
			if (sim_data->sa_x_property[a_prefix_vid].has_same_collision_group(sim_data->sa_x_property[b_prefix_vid]))
				return;
			if (!triangles_overlap_or_touch(mesh_data, a.face_id, b.face_id))
				return;

			LUISA_ERROR(
				"Initial mesh overlap detected between mesh {} ('{}') face {} and mesh {} ('{}') face {}. "
				"Move objects apart before init_solver(); initial overlap/contact is not supported.",
				a.mesh_id,
				world_data[a.mesh_id].get_model_name(),
				a.face_id,
				b.mesh_id,
				world_data[b.mesh_id].get_model_name(),
				b.face_id);
		}

		static void query_bvh_pair(const std::vector<WorldData>& world_data,
			const MeshData<std::vector>*						 mesh_data,
			const SimulationData<std::vector>*					 sim_data,
			const CpuBvh&										 bvh,
			const int											 node_a_idx,
			const int											 node_b_idx)
		{
			const CpuBvhNode& node_a = bvh.nodes[node_a_idx];
			const CpuBvhNode& node_b = bvh.nodes[node_b_idx];
			if (!overlaps_aabb(node_a.aabb, node_b.aabb))
				return;

			if (node_a.is_leaf() && node_b.is_leaf())
			{
				for (uint i = node_a.begin; i < node_a.end; ++i)
				{
					for (uint j = node_b.begin; j < node_b.end; ++j)
					{
						check_primitive_pair(world_data, mesh_data, sim_data, bvh.primitives[i], bvh.primitives[j]);
					}
				}
				return;
			}

			if (node_a_idx == node_b_idx)
			{
				if (!node_a.is_leaf())
				{
					query_bvh_pair(world_data, mesh_data, sim_data, bvh, node_a.left, node_a.left);
					query_bvh_pair(world_data, mesh_data, sim_data, bvh, node_a.left, node_a.right);
					query_bvh_pair(world_data, mesh_data, sim_data, bvh, node_a.right, node_a.right);
				}
				return;
			}

			if (node_a.is_leaf())
			{
				query_bvh_pair(world_data, mesh_data, sim_data, bvh, node_a_idx, node_b.left);
				query_bvh_pair(world_data, mesh_data, sim_data, bvh, node_a_idx, node_b.right);
			}
			else if (node_b.is_leaf())
			{
				query_bvh_pair(world_data, mesh_data, sim_data, bvh, node_a.left, node_b_idx);
				query_bvh_pair(world_data, mesh_data, sim_data, bvh, node_a.right, node_b_idx);
			}
			else
			{
				query_bvh_pair(world_data, mesh_data, sim_data, bvh, node_a.left, node_b.left);
				query_bvh_pair(world_data, mesh_data, sim_data, bvh, node_a.left, node_b.right);
				query_bvh_pair(world_data, mesh_data, sim_data, bvh, node_a.right, node_b.left);
				query_bvh_pair(world_data, mesh_data, sim_data, bvh, node_a.right, node_b.right);
			}
		}

		static void validate_initial_mesh_overlaps(const std::vector<WorldData>& world_data,
			const MeshData<std::vector>*										 mesh_data,
			const SimulationData<std::vector>*									 sim_data,
			const std::vector<uint>&											 surface_face_indices)
		{
			CpuBvh bvh;
			bvh.primitives.reserve(surface_face_indices.size());
			for (const uint face_id : surface_face_indices)
			{
				const uint3		face = mesh_data->sa_faces[face_id];
				CpuBvhPrimitive primitive;
				primitive.face_id = face_id;
				primitive.mesh_id = mesh_data->sa_face_mesh_id[face_id];
				primitive.aabb = make_face_aabb(mesh_data, face_id);
				primitive.centroid = (mesh_data->sa_rest_x[face[0]] + mesh_data->sa_rest_x[face[1]] + mesh_data->sa_rest_x[face[2]]) / 3.0f;
				bvh.primitives.push_back(primitive);
			}

			bvh.build();
			if (bvh.root >= 0)
			{
				query_bvh_pair(world_data, mesh_data, sim_data, bvh, bvh.root, bvh.root);
			}
		}
	} // namespace

	void init_collision_data(std::vector<lcs::Initializer::WorldData>& world_data,
		lcs::MeshData<std::vector>*									   mesh_data,
		lcs::SimulationData<std::vector>*							   sim_data,
		lcs::CollisionData<std::vector>*							   collision_data)
	{

		std::vector<uint> surface_vert_indices = fn_get_active_indices(
			[&](const uint vid)
			{
				const uint mesh_type = mesh_data->sa_vert_mesh_type[vid];
				if (mesh_type == uint(Material::MaterialType::Tetrahedral))
				{
					const auto& adj_faces = mesh_data->vert_adj_faces[vid];
					if (!adj_faces.empty())
						return 1; // Surface vertex
					return 0;	  // Internal vertex
				}
				else
				{
					return 1; // Surface vertex
				}
			},
			mesh_data->num_verts);

		std::vector<bool> is_surface_vert(mesh_data->num_verts, false);
		for (const auto vid : surface_vert_indices)
		{
			is_surface_vert[vid] = true;
		}

		std::vector<uint> surface_edge_indices = fn_get_active_indices(
			[&](const uint eid)
			{
				const uint2 edge = mesh_data->sa_edges[eid];
				if (is_surface_vert[edge[0]] && is_surface_vert[edge[1]])
					return 1; // Surface edge
				else
					return 0; // Internal edge
			},
			mesh_data->num_edges);

		std::vector<uint> surface_face_indices = fn_get_active_indices(
			[&](const uint fid)
			{
				const uint	mesh_idx = mesh_data->sa_face_mesh_id[fid];
				const auto& shell_info = world_data[mesh_idx];
				return 1; // All faces are surface faces
			},
			mesh_data->num_faces);

		const uint num_surface_verts = static_cast<uint>(surface_vert_indices.size());
		const uint num_surface_edges = static_cast<uint>(surface_edge_indices.size());
		const uint num_surface_faces = static_cast<uint>(surface_face_indices.size());
		LUISA_INFO("Surface verts count = {} (Total verts = {})", num_surface_verts, mesh_data->num_verts);
		LUISA_INFO("Surface edges count = {} (Total edges = {})", num_surface_edges, mesh_data->num_edges);
		LUISA_INFO("Surface faces count = {} (Total faces = {})", num_surface_faces, mesh_data->num_faces);
		validate_initial_mesh_overlaps(world_data, mesh_data, sim_data, surface_face_indices);
		sim_data->sa_contact_active_verts.resize(num_surface_verts);
		sim_data->sa_contact_active_edges.resize(num_surface_edges);
		sim_data->sa_contact_active_faces.resize(num_surface_faces);
		std::transform(surface_vert_indices.begin(),
			surface_vert_indices.end(),
			sim_data->sa_contact_active_verts.begin(),
			[&](uint vid)
			{ return vid; });
		std::transform(surface_edge_indices.begin(),
			surface_edge_indices.end(),
			sim_data->sa_contact_active_edges.begin(),
			[&](uint eid)
			{ return mesh_data->sa_edges[eid]; });
		std::transform(surface_face_indices.begin(),
			surface_face_indices.end(),
			sim_data->sa_contact_active_faces.begin(),
			[&](uint fid)
			{ return mesh_data->sa_faces[fid]; });

		// TODO: Replace contact list with active contact lists

		std::vector<float> mesh_min_dist(mesh_data->num_meshes, 1e10f);
		CpuParallel::single_thread_for(0,
			mesh_data->num_edges,
			[&](const uint eid)
			{
				const uint2 edge = mesh_data->sa_edges[eid];
				if (!mesh_data->sa_is_fixed[edge[0]] && !mesh_data->sa_is_fixed[edge[1]])
				{
					const float rest_length = luisa::length(
						mesh_data->sa_rest_x[edge[0]] - mesh_data->sa_rest_x[edge[1]]);
					const uint mesh_idx = mesh_data->sa_edge_mesh_id[eid];
					mesh_min_dist[mesh_idx] = min_scalar(mesh_min_dist[mesh_idx], rest_length);
					//    CpuParallel::spin_atomic<float>::fetch_min(mesh_min_dist[mesh_idx], rest_length);
				}
			});
		CpuParallel::single_thread_for(0,
			mesh_data->num_faces,
			[&](const uint fid)
			{
				const uint3 face = mesh_data->sa_faces[fid];
				if (!mesh_data->sa_is_fixed[face[0]] && !mesh_data->sa_is_fixed[face[1]] && !mesh_data->sa_is_fixed[face[2]])
				{
					float3 vert_pos[3] = { mesh_data->sa_rest_x[face[0]],
						mesh_data->sa_rest_x[face[1]],
						mesh_data->sa_rest_x[face[2]] };

					auto rest_dist = 10000.0f;
					for (int i = 0; i < 3; i++)
					{
						const float3& v0 = vert_pos[i];
						const float3& v1 = vert_pos[(i + 1) % 3];
						const float3& v2 = vert_pos[(i + 2) % 3];

						auto bary = host_distance::point_edge_distance_coeff(
							float3_to_eigen3(v0), float3_to_eigen3(v1), float3_to_eigen3(v2));
						const float curr_dist = length_vec(bary[0] * v1 + bary[1] * v2 - v0);
						rest_dist = min_scalar(rest_dist, curr_dist);
					}
					const uint mesh_idx = mesh_data->sa_face_mesh_id[fid];
					mesh_min_dist[mesh_idx] = min_scalar(mesh_min_dist[mesh_idx], rest_dist);
					//    CpuParallel::spin_atomic<float>::fetch_min(mesh_min_dist[mesh_idx], rest_dist);
				}
			});

		std::vector<float> mesh_scaled_offset(mesh_data->num_meshes);
		std::vector<float> mesh_scaled_d_hat(mesh_data->num_meshes);
		for (uint mesh_idx = 0; mesh_idx < mesh_data->num_meshes; mesh_idx++)
		{
			const bool is_rigid_body = world_data[mesh_idx].holds<Material::RigidMaterial>();

			float thickness = world_data[mesh_idx].get_contact_offset();
			float d_hat = world_data[mesh_idx].get_d_hat();

			float scaled_offset = 0.5f * thickness;
			float scaled_d_hat = d_hat;

			float		min_dist = mesh_min_dist[mesh_idx];
			const float safe_dist = is_rigid_body ? 1e8f : 0.9f * min_dist;

			if (safe_dist < 1e-4 && !is_rigid_body) // Soft-body exist penetration in rest state
			{
				// Note: We add this condition just for s
				LUISA_WARNING("Sub-milimeter simulation is not stable, due to small distance gap (Mesh {}, Min dist = {})",
					world_data[mesh_idx].get_model_name(),
					safe_dist);
			}
			if (scaled_offset + d_hat < safe_dist)
			{
			}
			else
			{
				if (scaled_offset > 0.5f * safe_dist)
				{
					LUISA_INFO("For mesh {}: Mesh offset scaled from {} to the safe distance {}",
						mesh_idx,
						scaled_offset,
						0.5f * safe_dist);
					scaled_offset = 0.5f * safe_dist;
				}
				if (scaled_d_hat > safe_dist - scaled_offset)
				{
					LUISA_INFO("For mesh {}: Mesh d_hat scaled from {} to the safe distance {}",
						mesh_idx,
						scaled_d_hat,
						safe_dist - scaled_offset);
					scaled_d_hat = safe_dist - scaled_offset;
				}
			}
			mesh_scaled_offset[mesh_idx] = scaled_offset;
			mesh_scaled_d_hat[mesh_idx] = scaled_d_hat;
			LUISA_INFO("Mesh {}: min_dist = {}, scaled_offset = {}, scaled_d_hat = {}",
				mesh_idx,
				min_dist,
				mesh_scaled_offset[mesh_idx],
				mesh_scaled_d_hat[mesh_idx]);
			// mesh_scaled_offset[mesh_idx] = 0.000f;
			// mesh_scaled_d_hat[mesh_idx]  = 0.003f;
		}

		sim_data->sa_contact_active_verts_d_hat.resize(mesh_data->num_verts);
		sim_data->sa_contact_active_verts_offset.resize(mesh_data->num_verts);
		sim_data->sa_contact_active_verts_friction_coeff.resize(mesh_data->num_verts);
		CpuParallel::parallel_for(0,
			mesh_data->num_verts,
			[&](const uint vid)
			{
				const uint mesh_idx = mesh_data->sa_vert_mesh_id[vid];
				sim_data->sa_contact_active_verts_d_hat[vid] = mesh_scaled_d_hat[mesh_idx];
				sim_data->sa_contact_active_verts_offset[vid] = mesh_scaled_offset[mesh_idx];
				sim_data->sa_contact_active_verts_friction_coeff[vid] =
					world_data[mesh_idx].get_friction_mu();
				//   LUISA_INFO("Vertex {}: d_hat = {}, offset = {}",
				//              vid,
				//              sim_data->sa_contact_active_verts_d_hat[vid],
				//              sim_data->sa_contact_active_verts_offset[vid]);
			});

		// Init stiffness
		{
			const float default_stiffness = get_scene_params().stiffness_collision;
			float		min_dist = 1e-5f;
			float		d_hat = 1e-3f;
			float		max_k1 = -default_stiffness * ipc::barrier_first_derivative(min_dist, d_hat);
			float		avg_area = std::reduce(mesh_data->sa_rest_vert_area.begin(), mesh_data->sa_rest_vert_area.end())
				/ static_cast<float>(mesh_data->num_verts);
			// In contact, we need to multiply contact stiffness by element area, about 5e-5
			LUISA_INFO("Max force per area can provided by kappa {:1.2e} in d_hat {:1.2e}m and min_dist {:1.2e}m = {:1.2e}N (Multiplied by avgArea {:6.5f} = {}N) ",
				default_stiffness,
				d_hat,
				min_dist,
				max_k1,
				avg_area,
				avg_area * max_k1);

			// const float max_mass = *std::min_element(mesh_data->sa_vert_mass.begin(), mesh_data->sa_vert_mass.end());
			// const float max_body_mass =
			//     *std::min_element(mesh_data->sa_body_mass.begin(), mesh_data->sa_body_mass.end());
			// float h2_inv = 1.0f / (get_scene_params().get_substep_dt() * get_scene_params().get_substep_dt());
			// float max_delta = max_k1 / (max_body_mass * h2_inv);
		}
	}
	void upload_collision_buffers(luisa::compute::Device& device,
		luisa::compute::Stream&							  stream,
		lcs::SimulationData<std::vector>*				  input_sim_data,
		lcs::SimulationData<luisa::compute::Buffer>*	  output_sim_data,
		lcs::CollisionData<std::vector>*				  input_collision_data,
		lcs::CollisionData<luisa::compute::Buffer>*		  output_collision_data)
	{
		stream << upload_buffer(device, output_sim_data->sa_contact_active_verts, input_sim_data->sa_contact_active_verts)
			   << upload_buffer(device, output_sim_data->sa_contact_active_faces, input_sim_data->sa_contact_active_faces)
			   << upload_buffer(device, output_sim_data->sa_contact_active_edges, input_sim_data->sa_contact_active_edges);

		stream
			<< upload_buffer(device, output_sim_data->sa_contact_active_verts_d_hat, input_sim_data->sa_contact_active_verts_d_hat)
			<< upload_buffer(device, output_sim_data->sa_contact_active_verts_offset, input_sim_data->sa_contact_active_verts_offset)
			<< upload_buffer(device, output_sim_data->sa_contact_active_verts_friction_coeff, input_sim_data->sa_contact_active_verts_friction_coeff)
			<< luisa::compute::synchronize();
	}

} // namespace lcs::Initializer
