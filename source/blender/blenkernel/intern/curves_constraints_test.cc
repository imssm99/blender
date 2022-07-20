/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_curves.hh"
#include "BKE_curves_constraints.hh"

#include "testing/testing.h"

namespace blender::bke::tests {

using curves::ConstraintSolver;

static CurvesGeometry create__curves(const int points_size, const int curves_size)
{
}

TEST(curves_constraints, EmptyCurve)
{
  CurvesGeometry curves;

  ConstraintSolver solver;
  solver.initialize(curves);

  solver.solve_constraints(curves, curves.curves_range());
}

TEST(curves_constraints, Unconstrained)
{
  CurvesGeometry curves(10, 5);
  curves.fill_curve_types(CURVE_TYPE_POLY);
  curves.resolution_for_write().fill(8);

  MutableSpan<int> offsets = curves.offsets_for_write();
  offsets[1] = 0; /* Curve 0: empty */
  offsets[2] = 1; /* Curve 1: single point */
  offsets[3] = 3; /* Curve 2: one segment */
  offsets[4] = 6; /* Curve 3: two segments */
  offsets[5] = 10; /* Curve 4: three segments */

  MutableSpan<float3> positions = curves.positions_for_write();
  /* Curve 1 */
  positions[0] = float3{1, 0, 0};
  /* Curve 2 */
  positions[1] = float3{1, 0, 1};
  positions[2] = float3{0, 2, 1};
  /* Curve 3 */
  positions[3] = float3{1, 0, 2};
  positions[4] = float3{0, 2, 2};
  positions[5] = float3{-1, 0, 2};
  /* Curve 4 */
  positions[6] = float3{1, 0, 3};
  positions[7] = float3{0, 2, 3};
  positions[5] = float3{-1, 0, 3};
  positions[9] = float3{1, -2, 3};

  ConstraintSolver solver;
  solver.initialize(curves);

  solver.solve_constraints(curves, curves.curves_range());
}

TEST(curves_constraints, SingleSegments)
{
}

TEST(curves_constraints, StretchCompressCurve)
{
}

TEST(curves_constraints, UnconstrainedCurveXXX)
{
  CurvesGeometry curves(3, 1);
  curves.fill_curve_types(CURVE_TYPE_POLY);
  curves.resolution_for_write().fill(8);
  curves.offsets_for_write().last() = 3;

  MutableSpan<float3> positions = curves.positions_for_write();
  positions[0] = {-1, 0, 0};
  positions[1] = {1, 0, 0};
  positions[2] = {2, 1, 0};

  //Span<float3> evaluated_positions = curves.evaluated_positions();
  //static const Array<float3> result_1{{
  //    {-1.0f, 0.0f, 0.0f},
  //    {-0.955078f, 0.287109f, 0.0f},
  //    {-0.828125f, 0.421875f, 0.0f},
  //    {-0.630859f, 0.439453f, 0.0f},
  //    {-0.375f, 0.375f, 0.0f},
  //    {-0.0722656f, 0.263672f, 0.0f},
  //    {0.265625f, 0.140625f, 0.0f},
  //    {0.626953f, 0.0410156f, 0.0f},
  //    {1.0f, 0.0f, 0.0f},
  //    {1.28906f, 0.0429688f, 0.0f},
  //    {1.4375f, 0.15625f, 0.0f},
  //    {1.49219f, 0.316406f, 0.0f},
  //    {1.5f, 0.5f, 0.0f},
  //    {1.50781f, 0.683594f, 0.0f},
  //    {1.5625f, 0.84375f, 0.0f},
  //    {1.71094f, 0.957031f, 0.0f},
  //    {2.0f, 1.0f, 0.0f},
  //}};
  //for (const int i : evaluated_positions.index_range()) {
  //  EXPECT_V3_NEAR(evaluated_positions[i], result_1[i], 1e-5f);
  //}

  //Array<float> radii{{0.0f, 1.0f, 2.0f}};
  //Array<float> evaluated_radii(17);
  //curves.interpolate_to_evaluated(0, radii.as_span(), evaluated_radii.as_mutable_span());
  //static const Array<float> result_2{{
  //    0.0f,
  //    0.125f,
  //    0.25f,
  //    0.375f,
  //    0.5f,
  //    0.625f,
  //    0.75f,
  //    0.875f,
  //    1.0f,
  //    1.125f,
  //    1.25f,
  //    1.375f,
  //    1.5f,
  //    1.625f,
  //    1.75f,
  //    1.875f,
  //    2.0f,
  //}};
  //for (const int i : evaluated_radii.index_range()) {
  //  EXPECT_NEAR(evaluated_radii[i], result_2[i], 1e-6f);
  //}
}


}  // namespace blender::bke::tests
