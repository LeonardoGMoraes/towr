/*
 * discretized_terrain_constraint.cc
 *
 *  Created on: Jan 16, 2023
 *      Author: vivian
 */

#include <iostream>
#include <towr/constraints/discretized_terrain_constraint.h>
#include <towr/variables/variable_names.h>

namespace towr {

DiscretizedTerrainConstraint::DiscretizedTerrainConstraint (double T, double dt,
                                                  	  	  	const EE& ee,
															const HeightMap::Ptr& terrain,
															const SplineHolder& spline_holder)
    : ConstraintSet(kSpecifyLater, "terrain-height-" + std::to_string(ee))
{
	ee_motion_ = spline_holder.ee_motion_.at(ee);
	ee_ = ee;
	spline_holder_ = spline_holder;
	terrain_ = terrain;

	GetDiscreteTimes(T, dt);

	int n_constraints = dts_.size();
//	std::cout << "[DiscretizedTerrainConstraint " << ee << "] Number of constraints: " << n_constraints << std::endl;
//	for (double t : dts_)
//		std::cout << t << " ";
//	std::cout << std::endl;
	SetRows(n_constraints);  // one constraint per node
}

void
DiscretizedTerrainConstraint::GetDiscreteTimes(double T, double dt)
{
	double t = 0.0;
	dts_ = {t};

	for (int i=0; i<floor(T/dt); ++i) {
		t += dt;
		if (!spline_holder_.phase_durations_.at(ee_)->IsContactPhase(t))
			dts_.push_back(t);
	}

	dts_.push_back(T); // also ensure constraints at very last node/time.
}

void
DiscretizedTerrainConstraint::UpdateConstraintAtInstance (double t, int k, VectorXd& g) const
{
	Vector3d p = ee_motion_->GetPoint(t).p();

	g(k) = p.z() - terrain_->GetHeight(p.x(), p.y());
}

void
DiscretizedTerrainConstraint::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{
	double max_distance_above_terrain = 1e20; // [m]

//    if (spline_holder_.phase_durations_.at(ee_)->IsContactPhase(t))
//      bounds.at(k) = ifopt::BoundZero;
//    else
      bounds.at(k) = ifopt::Bounds(0.0001, max_distance_above_terrain);
}

DiscretizedTerrainConstraint::Jacobian
DiscretizedTerrainConstraint::GetJacobianTerrainHeight(double x, double y) const
{
  Jacobian jac = Eigen::Vector3d::Ones().transpose().sparseView();

  jac.coeffRef(0, X_) = -terrain_->GetDerivativeOfHeightWrt(X_, x, y);
  jac.coeffRef(0, Y_) = -terrain_->GetDerivativeOfHeightWrt(Y_, x, y);

  return jac;
}

void
DiscretizedTerrainConstraint::UpdateJacobianAtInstance (double t, int k,
                                                   	    std::string var_set,
														Jacobian& jac) const
{
	if (var_set == id::EEMotionNodes(ee_)) {
		Vector3d pos = ee_motion_->GetPoint(t).p();
		jac.row(k) = GetJacobianTerrainHeight(pos.x(), pos.y()) * ee_motion_->GetJacobianWrtNodes(t, kPos);
	}
}

DiscretizedTerrainConstraint::VectorXd
DiscretizedTerrainConstraint::GetValues () const
{
	VectorXd g = VectorXd::Zero(GetRows());

	int k = 0;
	for (double t : dts_)
		UpdateConstraintAtInstance(t, k++, g);

  return g;
}

DiscretizedTerrainConstraint::VecBound
DiscretizedTerrainConstraint::GetBounds () const
{
	VecBound bounds(GetRows());

	int k = 0;
	for (double t : dts_)
		UpdateBoundsAtInstance(t, k++, bounds);

	return bounds;
}

void
DiscretizedTerrainConstraint::FillJacobianBlock (std::string var_set,
                                                  Jacobian& jac) const
{
	int k = 0;
	for (double t : dts_)
		UpdateJacobianAtInstance(t, k++, var_set, jac);
}

} /* namespace xpp */



