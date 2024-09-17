/*
 * safe_foothold_constraint.h
 *
 *  Created on: Jan 17, 2023
 *      Author: vivian
 */

#ifndef TOWR_CONSTRAINTS_SAFE_FOOTHOLD_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_SAFE_FOOTHOLD_CONSTRAINT_H_

#include <ifopt/constraint_set.h>

#include <towr/variables/nodes_variables_phase_based.h>

#include <towr/terrain/height_map.h>

namespace towr {

class SafeFootholdConstraint : public ifopt::ConstraintSet {
public:
  using Vector3d = Eigen::Vector3d;
  using EE = uint;

  SafeFootholdConstraint (const HeightMap::Ptr& terrain,
                   	   	  double safety_radius, EE endeffector_id);
  virtual ~SafeFootholdConstraint () = default;

  void InitVariableDependedQuantities(const VariablesPtr& x) override;

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

private:
  NodesVariablesPhaseBased::Ptr ee_motion_; ///< the current xyz foot positions.

  HeightMap::Ptr terrain_; 		///< gradient information at every position (x,y).
  int n_constraints_per_node_; 	///< number of constraints for each node.
  EE ee_;                 		///< The endeffector to be constrained.
  double safety_radius_;		///< radius distance for safety check


  std::vector<int> pure_stance_node_ids_;
};

} /* namespace towr */



#endif /* TOWR_CONSTRAINTS_SAFE_FOOTHOLD_CONSTRAINT_H_ */
