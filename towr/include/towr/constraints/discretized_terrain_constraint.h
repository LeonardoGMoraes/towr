/*
 * discretized_terrain_constraint.h
 *
 *  Created on: Jan 16, 2023
 *      Author: vivian
 */

#ifndef TOWR_CONSTRAINTS_DISCRETIZED_TERRAIN_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_DISCRETIZED_TERRAIN_CONSTRAINT_H_

#include <ifopt/constraint_set.h>

#include <towr/variables/spline.h>
#include <towr/variables/spline_holder.h>

#include <towr/terrain/height_map.h>

namespace towr {

/** @brief Ensures the endeffectors always lays on or above terrain height.
  *
  * The difference between this constraint and the TerrainConstraint is that
  * this one is enforced in discretized times along the feet trajectories, not
  * only at the spline nodes.
  *
  * @ingroup Constraints
  */
class DiscretizedTerrainConstraint : public ifopt::ConstraintSet {
public:
  using EE = uint;
  using Vector3d = Eigen::Vector3d;

  /**
   * @brief Constructs a constraint instance.
   * @param T   The total duration of the optimization.
   * @param dt  The discretization interval at which to enforce constraints.
   * @param ee  The endeffector for which to constrain the motion.
   * @param terrain  The terrain height value and slope for each position x,y.
   * @param spline_holder Pointer to the current variables.
   */
  DiscretizedTerrainConstraint(double T, double dt,
                          	   const EE& ee,
                               const HeightMap::Ptr& terrain,
                               const SplineHolder& spline_holder);
  virtual ~DiscretizedTerrainConstraint () = default;

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

private:
  NodeSpline::Ptr ee_motion_;
  EE ee_;
  SplineHolder spline_holder_;
  HeightMap::Ptr terrain_;

  std::vector<double> dts_;

  void GetDiscreteTimes (double T, double dt);

  void UpdateConstraintAtInstance (double t, int k, VectorXd& g) const;
  void UpdateBoundsAtInstance (double t, int k, VecBound&) const;
  void UpdateJacobianAtInstance(double t, int k, std::string, Jacobian&) const;

  Jacobian GetJacobianTerrainHeight(double x, double y) const;

};

} /* namespace towr */



#endif /* TOWR_CONSTRAINTS_DISCRETIZED_TERRAIN_CONSTRAINT_H_ */
