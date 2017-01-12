/**
 @file    step_sequence_planner.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Declares the interface to the StepSequencePlanner.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STEP_SEQUENCE_PLANNER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STEP_SEQUENCE_PLANNER_H_

#include <xpp/utils/state.h>
#include <xpp/opt/contact.h>
#include <xpp/opt/motion_type.h>
#include <memory>

namespace xpp {
namespace opt {

/** Plans the sequence of steps (LH, LF, ...) for a given optimization problem.
  *
  * This is information that has to be passed to the NLP optimizer beforehand.
  * This class is specific for HyQ and must be adapted for different robots.
  * It contains all the information needed to determine the step sequence.
  */
class StepSequencePlanner {
public:
  using SwingLegsInPhase  = MotionType::Swinglegs;
  using AllPhaseSwingLegs = std::vector<SwingLegsInPhase>;
  using StartStance       = std::vector<xpp::opt::Contact>;
  using State             = xpp::utils::StateLin2d;
  using MotionTypePtr     = std::shared_ptr<MotionType>;

  StepSequencePlanner ();
  virtual ~StepSequencePlanner ();

  /** Necessary information to determine the step sequence.
    *
    * @param the current pos/vel/acc of the robot.
    * @param the desired pos/vel/acc that the robot should achieve.
    * @param start_stance the 4 footholds right before swinging the first leg.
    * @param robot_height the walking height [m] of the robot.
    */
  void Init(const State& curr, const State& goal,
            const StartStance& start_stance, double robot_height,
            int swingleg_of_last_spline);

  /** Defines the endeffectors in swing for each motion phase
    */
  AllPhaseSwingLegs DetermineStepSequence(const MotionTypePtr&);

private:
  State curr_state_;
  State goal_state_;
  StartStance start_stance_;
  double robot_height_;
  double max_step_length_;
  int curr_swingleg_; // this could also be no swingleg (stance phase)
};

} /* namespace opt */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_STEP_SEQUENCE_PLANNER_H_ */