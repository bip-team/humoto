/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define HUMOTO_TEST_MODULE_NAME     pepper_mpc
#include "test_helpers.h"


/**
 * @brief Hierarchy 0: basic
 *
 * @param[out] opt_problem       hierarchy
 * @param[in] motion_parameters  motion parameters
 */
void setupHierarchy_v0( humoto::OptimizationProblem &opt_problem,
                        const humoto::pepper_mpc::MotionParameters &motion_parameters)
{
    // tasks, which are used in the control problem
    humoto::TaskSharedPointer   task_base_vel_bounds(new humoto::pepper_mpc::TaskBaseVelocityBounds);
    humoto::TaskSharedPointer   task_body_pos_bounds(new humoto::pepper_mpc::TaskBodyPositionBounds);
    humoto::TaskSharedPointer   task_cop_pos_bounds (new humoto::pepper_mpc::TaskCoPPositionBounds);
    humoto::TaskSharedPointer   task_base_acc_bounds(new humoto::pepper_mpc::TaskBaseAccelerationBounds);

    humoto::TaskSharedPointer   task_base_ref_pos  (new humoto::pepper_mpc::TaskBasePositionReference);
    humoto::TaskSharedPointer   task_base_ref_vel  (new humoto::pepper_mpc::TaskBaseVelocityReference);
    humoto::TaskSharedPointer   task_body_ref_pos  (new humoto::pepper_mpc::TaskBodyPositionReference);
    humoto::TaskSharedPointer   task_base_jerk     (new humoto::pepper_mpc::TaskBaseJerkMinimization(0.0223606797749979));
    humoto::TaskSharedPointer   task_body_jerk     (new humoto::pepper_mpc::TaskBodyJerkMinimization(0.0223606797749979));
    humoto::TaskSharedPointer   task_cop_centering (new humoto::pepper_mpc::TaskCoPCentering);

    // reset the optimization problem
    opt_problem.reset(2);

    // push tasks into the stack/hierarchy
    opt_problem.pushTask(task_base_vel_bounds, 0);
    opt_problem.pushTask(task_body_pos_bounds, 0);
    opt_problem.pushTask(task_cop_pos_bounds, 0);
    opt_problem.pushTask(task_base_acc_bounds, 0);


    switch (motion_parameters.motion_mode_)
    {
        case humoto::pepper_mpc::MotionMode::MAINTAIN_POSITION:
            opt_problem.pushTask(task_base_ref_pos, 1);
            break;
        case humoto::pepper_mpc::MotionMode::MAINTAIN_VELOCITY:
            opt_problem.pushTask(task_base_ref_vel, 1);
            break;
        default:
            HUMOTO_THROW_MSG("Unsupported motion mode.");
    }
    //opt_problem.pushTask(task_body_ref_pos, 1);
    opt_problem.pushTask(task_base_jerk, 1);
    opt_problem.pushTask(task_body_jerk, 1);
    opt_problem.pushTask(task_cop_centering, 1);
}
