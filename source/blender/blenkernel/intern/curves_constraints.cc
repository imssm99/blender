/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_curves_constraints.hh"

#include "BLI_index_mask_ops.hh"

#include "BKE_bvhutils.h"
#include "BKE_mesh_runtime.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"

namespace blender::bke::curves {

using blender::bke::CurvesGeometry;
using threading::EnumerableThreadSpecific;

static const int curves_grain_size = 64;

int ConstraintSolver::substep_count() const
{
  return substep_count_;
}

void ConstraintSolver::set_substep_count(int substep_count)
{
  substep_count_ = substep_count;
}

float ConstraintSolver::max_travel_distance() const
{
  return max_travel_distance_;
}

void ConstraintSolver::set_max_travel_distance(float max_travel_distance)
{
  max_travel_distance_ = max_travel_distance;
}

int ConstraintSolver::solver_iterations() const
{
  return solver_iterations_;
}

void ConstraintSolver::set_solver_iterations(int solver_iterations)
{
  solver_iterations_ = solver_iterations;
}

int ConstraintSolver::max_contacts_per_point() const
{
  return max_contacts_per_point_;
}

void ConstraintSolver::set_max_contacts_per_point(int max_contacts_per_point)
{
  max_contacts_per_point_ = max_contacts_per_point;
}

void ConstraintSolver::initialize(const CurvesGeometry &curves)
{
  Span<float3> positions_cu = curves.positions();

  segment_lengths_cu_.reinitialize(curves.points_num());
  threading::parallel_for(curves.curves_range(), curves_grain_size, [&](const IndexRange range) {
    for (const int curve_i : range) {
      const IndexRange points = curves.points_for_curve(curve_i);

      float length_cu = 0.0f, prev_length_cu;
      for (const int point_i : points.drop_back(1)) {
        const float3 &p1_cu = positions_cu[point_i];
        const float3 &p2_cu = positions_cu[point_i + 1];
        prev_length_cu = length_cu;
        length_cu = math::distance(p1_cu, p2_cu);
        segment_lengths_cu_[point_i] = length_cu;
      }
    }
  });

  contacts_num_.reinitialize(curves.points_num());
  contacts_.reinitialize(max_contacts_per_point_ * curves.points_num());
}

void ConstraintSolver::step(const Depsgraph &depsgraph,
                            Object &object,
                            CurvesGeometry &curves,
                            const Object *surface_ob,
                            const CurvesSurfaceTransforms &transforms,
                            Span<float3> orig_positions,
                            VArray<int> changed_curves)
{
  MutableSpan<float3> positions = curves.positions_for_write();

  const float max_substep_travel_distance = max_travel_distance_ / substep_count_;
  const float max_collision_distance = 2.0f * max_substep_travel_distance;
  const float max_travel_distance_sq = max_travel_distance_ * max_travel_distance_;

  const bool clamp_travel = true;
#ifdef DEBUG
  bool max_travel_exceeded = false;
#endif

  /* Compute position delta per substep ("velocity") */
  Array<float3> dX(curves.points_num());
  threading::parallel_for(
      changed_curves.index_range(), curves_grain_size, [&](const IndexRange range) {
        for (const int i : range) {
          const int curve_i = changed_curves[i];
          const IndexRange points = curves.points_for_curve(curve_i);
          for (const int point_i : points.drop_front(1)) {
            float3 delta_step = positions[point_i] - orig_positions[point_i];

            if (clamp_travel) {
              float travel_sq = len_squared_v3(delta_step);
              if (travel_sq > max_travel_distance_sq) {
                normalize_v3(delta_step);
                delta_step *= max_travel_distance_;
              }
            }
#ifdef DEBUG
            else {
              max_travel_exceeded |= len_squared_v3(delta_step) > max_travel_distance_sq;
            }
#endif

            dX[point_i] = delta_step / substep_count_;
            positions[point_i] = orig_positions[point_i];
          }
        }
      });
#ifdef DEBUG
  /* TODO warning */
  //BLI_assert(!max_travel_exceeded);
  if (max_travel_exceeded) {
    std::cout << "Max. Travel Distance Exceeded!" << std::endl;
  }
#endif

  for (int substep : IndexRange(substep_count_)) {
    /* Set unconstrained position: x <- x + v*dt */
    threading::parallel_for(
        changed_curves.index_range(), curves_grain_size, [&](const IndexRange range) {
          for (const int i : range) {
            const int curve_i = changed_curves[i];
            const IndexRange points = curves.points_for_curve(curve_i);
            for (const int point_i : points.drop_front(1)) {
              positions[point_i] += dX[point_i];
            }
          }
        });

    /* Solve constraints */
    find_contact_points(
        depsgraph, object, curves, surface_ob, transforms, max_collision_distance, changed_curves);
    solve_constraints(curves, changed_curves);
  }
}

void ConstraintSolver::find_contact_points(const Depsgraph &depsgraph,
                                           Object &object,
                                           const CurvesGeometry &curves,
                                           const Object *surface_ob,
                                           const CurvesSurfaceTransforms &transforms,
                                           float max_dist,
                                           VArray<int> changed_curves)
{
  /* Should be set when initializing constraints */
  BLI_assert(contacts_num_.size() == curves.points_num());
  BLI_assert(contacts_.size() == curves.points_num() * max_contacts_per_point_);

  contacts_num_.fill(0);

  if (surface_ob == nullptr || surface_ob->type != OB_MESH) {
    contacts_.reinitialize(0);
    return;
  }

  const float curves_to_surface_scale = mat4_to_scale(transforms.curves_to_surface.ptr());
  const float surface_to_curves_scale = mat4_to_scale(transforms.curves_to_surface.ptr());
  const float max_dist_su = curves_to_surface_scale * max_dist;
  const float max_dist_sq_su = max_dist_su * max_dist_su;

  const Mesh *surface = static_cast<Mesh *>(surface_ob->data);
  Span<MLoopTri> surface_looptris = {BKE_mesh_runtime_looptri_ensure(surface),
                                     BKE_mesh_runtime_looptri_len(surface)};

  /** BVH tree of the surface mesh for finding collisions. */
  BVHTreeFromMesh surface_bvh;
  BKE_bvhtree_from_mesh_get(&surface_bvh, surface, BVHTREE_FROM_LOOPTRI, 2);
  BLI_SCOPED_DEFER([&]() { free_bvhtree_from_mesh(&surface_bvh); });

  threading::parallel_for(
      changed_curves.index_range(), curves_grain_size, [&](const IndexRange range) {
        for (const int i : range) {
          const int curve_i = changed_curves[i];
          const IndexRange points = curves.points_for_curve(curve_i);

          /* First point is anchored to the surface, ignore collisions. */
          for (const int point_i : points.drop_front(1)) {
            const float3 &new_p = curves.positions()[point_i];

            MutableSpan<Contact> contacts(contacts_.begin() + max_contacts_per_point_ * point_i,
                                          max_contacts_per_point_);

            float3 p_su = transforms.curves_to_surface * new_p;
            BLI_bvhtree_range_query_cpp(
                *surface_bvh.tree,
                p_su,
                max_dist_su,
                [&](const int triangle_i, const float3 &co_su, float dist_sq_su) {
                  const MLoopTri &looptri = surface_looptris[triangle_i];
                  const float3 v0_su = surface->mvert[surface->mloop[looptri.tri[0]].v].co;
                  const float3 v1_su = surface->mvert[surface->mloop[looptri.tri[1]].v].co;
                  const float3 v2_su = surface->mvert[surface->mloop[looptri.tri[2]].v].co;
                  float3 closest_su;
                  closest_on_tri_to_point_v3(closest_su, co_su, v0_su, v1_su, v2_su);
                  dist_sq_su = len_squared_v3v3(co_su, closest_su);
                  if (dist_sq_su <= max_dist_sq_su) {
                    const int contacts_num = contacts_num_[point_i];
                    int insert_i;
                    if (contacts_num < max_contacts_per_point_) {
                      insert_i = contacts_num;
                      ++contacts_num_[point_i];
                    }
                    else {
                      /* Replace the contact with the largest distance. */
                      insert_i = -1;
                      float max_dist_sq_su = dist_sq_su;
                      for (int contact_i : IndexRange(4)) {
                        if (contacts[contact_i].dist_sq_ > max_dist_sq_su) {
                          insert_i = contact_i;
                          max_dist_sq_su = contacts[contact_i].dist_sq_;
                        }
                      }
                    }
                    if (insert_i >= 0) {
                      float3 normal_su;
                      normal_tri_v3(normal_su, v0_su, v1_su, v2_su);
                      contacts[insert_i] = Contact{
                          max_dist_sq_su,
                          transforms.surface_to_curves_normal * float3{normal_su},
                          transforms.surface_to_curves * float3{closest_su}};
                    }
                  }
                });
          }
        }
      });
}

void ConstraintSolver::solve_constraints(CurvesGeometry &curves, VArray<int> changed_curves) const
{
  /* Gauss-Seidel method for solving length and contact constraints.
   * See for example "Position-Based Simulation Methods in Computer Graphics"
   * by Mueller et. al. for an in-depth description.
   */

  const Span<float> expected_lengths_cu = segment_lengths_cu_;
  MutableSpan<float3> positions_cu = curves.positions_for_write();
  VArray<float> radius = curves.attributes().lookup_or_default<float>(
      "radius", ATTR_DOMAIN_POINT, 0.0f);

  // PROBLEM: Even with travel distance clamping, if point n is moved at max travel speed
  // its follower may end up below the surface
  
  threading::parallel_for(
      changed_curves.index_range(), curves_grain_size, [&](const IndexRange range) {
        for (const int i : range) {
          const int curve_i = changed_curves[i];
          const IndexRange points = curves.points_for_curve(curve_i);

          /* First point is anchored to surface, contact and length constraints to no apply. */
          for (const int point_i : points.drop_front(1)) {
            float3 &p = positions_cu[point_i];
            const int contacts_num = contacts_num_[point_i];
            for (int solver_i : IndexRange(solver_iterations_)) {
              /* Solve contact constraints */
              Span<Contact> contacts(contacts_.begin() + max_contacts_per_point_ * point_i,
                                     contacts_num);
              for (const Contact &c : contacts) {
                /* Lagrange multiplier for solving a single contact constraint.
                 * Note: The contact point is already offset from the surface by the radius due to
                 * the raycast callback, no need to subtract the radius from lambda. */
                const float lambda = dot_v3v3(p - c.point_, c.normal_);
                if (lambda < 0.0f) {
                  p -= lambda * c.normal_;
                }
              }

              /* Solve distance constraint */
              {
                const float3 &p_prev = positions_cu[point_i - 1];
                const float3 direction = math::normalize(p - p_prev);
                const float expected_length_cu = expected_lengths_cu[point_i - 1];
                p = p_prev + direction * expected_length_cu;
              }
            }
          }
        }
      });
}

}  // namespace blender::bke::curves
