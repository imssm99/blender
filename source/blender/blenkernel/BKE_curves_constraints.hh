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
  /* Number of iterations to satisfy constraints. */
  int solver_iterations_ = 5;

  /* Maximum number of simultaneous contacts to record per point. */
  int max_contacts_per_point_ = 4;

  /* Default radius to offset curves from the surface, if curve radius attribute is not defined. */
  float default_curve_radius_ = 0.05f;

  /** Length of each segment indexed by the index of the first point in the segment. */
  Array<float> segment_lengths_cu_;

  struct Contact {
    float dist_;
    float3 normal_;
    float3 point_;
  };

  Array<int> contacts_num_;
  Array<Contact> contacts_;

 public:
  int solver_iterations() const;
  void set_solver_iterations(int solver_iterations);

  int max_contacts_per_point() const;
  void set_max_contacts_per_point(int max_contacts_per_point);

  float default_curve_radius() const;
  void set_default_curve_radius(float default_curve_radius);

  /* Remember the initial length of all curve segments. This allows restoring the length after
   * combing.
   */
  void initialize(const CurvesGeometry &curves);

  void find_contact_points(const Depsgraph *depsgraph,
                           Object *object,
                           const CurvesGeometry *curves,
                           const Object *surface,
                           const CurvesSurfaceTransforms &transforms,
                           Span<float3> orig_positions,
                           Span<int> changed_curves);

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
