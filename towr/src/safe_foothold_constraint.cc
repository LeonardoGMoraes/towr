/*
 * safe_foothold_constraint.cc
 *
 *  Created on: Jan 17, 2023
 *      Author: vivian
 */

#include <towr/constraints/safe_foothold_constraint.h>

#include <towr/variables/variable_names.h>

#include <iostream>
#include <cmath>

namespace towr {


SafeFootholdConstraint::SafeFootholdConstraint (const HeightMap::Ptr& terrain,
												double safety_radius, EE ee)
    :ifopt::ConstraintSet(kSpecifyLater, "safety-" + id::EEMotionNodes(ee))
{
  terrain_ = terrain;
  ee_      = ee;
  safety_radius_ = safety_radius;

  n_constraints_per_node_ = 1; 	// derivative before and after contact point
}

void
SafeFootholdConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  ee_motion_ = x->GetComponent<NodesVariablesPhaseBased>(id::EEMotionNodes(ee_));

  for (int id=0; id<ee_motion_->GetNodes().size(); ++id) {
	  if (ee_motion_->IsConstantNode(id))
		  pure_stance_node_ids_.push_back(id);
  }

  int constraint_count = pure_stance_node_ids_.size()*n_constraints_per_node_;
  SetRows(constraint_count);
}

Eigen::VectorXd
SafeFootholdConstraint::GetValues () const
{
  VectorXd g(GetRows());

  int row=0;
  auto motion_nodes = ee_motion_->GetNodes();
  for (int node_id : pure_stance_node_ids_) {
    Vector3d p = motion_nodes.at(node_id).p();

//    Vector3d n_a = terrain_->GetNormalizedBasis(HeightMap::Normal, p.x()+safety_radius_, p.y());
//    Vector3d n_b = terrain_->GetNormalizedBasis(HeightMap::Normal, p.x()-safety_radius_, p.y());
//    g(row++) = n_a(Z) - n_b(Z);

//    double dh_a = terrain_->GetDerivativeOfHeightWrt(X_, p.x()+safety_radius_, p.y());
//    double dh_b = terrain_->GetDerivativeOfHeightWrt(X_, p.x()-safety_radius_, p.y());
//    g(row++) = dh_a - dh_b;

    double h_a = terrain_->GetHeight(p.x()+2.0*safety_radius_, p.y());
    double h_b = terrain_->GetHeight(p.x()-safety_radius_, p.y());
    g(row++) = h_a - h_b;

    //std::cout << "n_a: " << n_a(Z) << ", n_b: " << n_b(Z) << " -> g: " << n_a(Z) - n_b(Z) << std::endl;

  }

  return g;
}

SafeFootholdConstraint::VecBound
SafeFootholdConstraint::GetBounds () const
{
  VecBound bounds;

  for (int node_id : pure_stance_node_ids_) {
    bounds.push_back(ifopt::BoundZero);
  }

  return bounds;
}

void
SafeFootholdConstraint::FillJacobianBlock (std::string var_set,
                                    	   Jacobian& jac) const
{
  if (var_set == ee_motion_->GetName()) {
    int row = 0;
    auto motion_nodes = ee_motion_->GetNodes();
    for (int node_id : pure_stance_node_ids_) {
      Vector3d p = motion_nodes.at(node_id).p();

//      for (auto dim : {X_,Y_}) {
//    	int idx = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(node_id, kPos, dim));
//        Vector3d dn_a  = terrain_->GetDerivativeOfNormalizedBasisWrt(HeightMap::Normal, dim, p.x()+safety_radius_, p.y());
//        Vector3d dn_b  = terrain_->GetDerivativeOfNormalizedBasisWrt(HeightMap::Normal, dim, p.x()-safety_radius_, p.y());
//		jac.coeffRef(row, idx) = dn_a(Z) - dn_b(Z);
//		//jac.coeffRef(row+1, idx) = dn_b(Z);
//      }

//      for (auto dim : {X,Y}) {
//        int idx = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(node_id, kPos, dim));
//        jac.coeffRef(row, idx) = terrain_->GetSecondDerivativeOfHeightWrt(To2D(dim), X_, p.x()+safety_radius_, p.y()) -
//        						 terrain_->GetSecondDerivativeOfHeightWrt(To2D(dim), X_, p.x()-safety_radius_, p.y());
//      }

      for (auto dim : {X,Y}) {
        int idx = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(node_id, kPos, dim));
        jac.coeffRef(row, idx) = terrain_->GetDerivativeOfHeightWrt(To2D(dim), p.x()+safety_radius_, p.y()) -
        						 terrain_->GetDerivativeOfHeightWrt(To2D(dim), p.x()-safety_radius_, p.y());
      }

      row += n_constraints_per_node_;
    }
  }
}

} /* namespace towr */





