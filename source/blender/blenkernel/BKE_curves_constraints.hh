/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_bvhutils.h"
#include "BKE_curves.hh"

    /** \file
 * \ingroup bke
 * \brief Constraint solver for curve deformations.
 */

namespace blender::bke::curves {

class ConstraintSolver {
 private:
  /* Number of substeps to perform.
   * More substeps can be faster overall because of reduced search radius for collisions. */
  int substep_count_ = 20;

  /* Maximum overall distance a particle can move during a step.
   * Divide by substep count to get max substep travel.
   * This determines a the search radius for collisions.
   * A larger travel distance means the point can move faster,
   * but it can take longer to find collisions. */
  float max_travel_distance_ = 0.1f;

  /* Number of iterations to satisfy constraints. */
  int solver_iterations_ = 5;

  /* Maximum number of simultaneous contacts to record per point. */
  int max_contacts_per_point_ = 4;

  /** Length of each segment indexed by the index of the first point in the segment. */
  Array<float> segment_lengths_cu_;

  struct Contact {
    float dist_sq_;
    float3 normal_;
    float3 point_;
  };

  Array<int> contacts_num_;
  Array<Contact> contacts_;

 public:
  int substep_count() const;
  void set_substep_count(int substep_count);

  float max_travel_distance() const;
  void set_max_travel_distance(float max_travel_distance);

  int solver_iterations() const;
  void set_solver_iterations(int solver_iterations);

  int max_contacts_per_point() const;
  void set_max_contacts_per_point(int max_contacts_per_point);

  /* Remember the initial length of all curve segments. This allows restoring the length after
   * combing.
   */
  void initialize(const CurvesGeometry &curves);

  void step(const Depsgraph &depsgraph,
            Object &object,
            CurvesGeometry &curves,
            const Object *surface_ob,
            const CurvesSurfaceTransforms &transforms,
            Span<float3> orig_positions,
            VArray<int> changed_curves);

 private:
  void find_contact_points(const Depsgraph &depsgraph,
                           Object &object,
                           const CurvesGeometry &curves,
                           const Object *surface_ob,
                           const CurvesSurfaceTransforms &transforms,
                           float max_dist,
                           VArray<int> changed_curves);

  /**
   * Satisfy constraints on curve points based on initial deformation.
   */
  void solve_constraints(CurvesGeometry &curves, VArray<int> changed_curves) const;

  /**
   * Satisfy constraints on curve points based on initial deformation.
   */
  inline void solve_constraints(CurvesGeometry &curves, IndexRange changed_curves) const
  {
    solve_constraints(curves,
                      VArray<int>::ForFunc(changed_curves.size(),
                                           [changed_curves](int64_t i) { return (int)i; }));
  }
};

}  // namespace blender::bke::curves
