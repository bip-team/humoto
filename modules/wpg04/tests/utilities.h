/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define HUMOTO_TEST_MODULE_NAME     wpg04
#include "test_helpers.h"



/**
 * @brief Hierarchy 0: basic
 *
 * @param[in] opt_problem       hierarchy
 */
void setupHierarchy_v0(humoto::OptimizationProblem &opt_problem)
{
    // tasks, which are used in the control problem
    humoto::TaskSharedPointer   task_com_velocity    (new humoto::wpg04::TaskCoMVelocity   );
    humoto::TaskSharedPointer   task_cop_position    (new humoto::wpg04::TaskCoPPosition   );
    humoto::TaskSharedPointer   task_cop_velocity    (new humoto::wpg04::TaskCoPVelocity   );
    humoto::TaskSharedPointer   task_footstep_bounds (new humoto::wpg04::TaskFootstepBounds);
    humoto::TaskSharedPointer   task_cop_bounds      (new humoto::wpg04::TaskCoPBounds     );

    // reset the optimization problem
    opt_problem.reset(2);

    // push tasks into the stack/hierarchy
    opt_problem.pushTask(task_footstep_bounds, 0);
    opt_problem.pushTask(task_cop_bounds, 0);
    opt_problem.pushTask(task_com_velocity, 1);
    opt_problem.pushTask(task_cop_velocity, 1);
    opt_problem.pushTask(task_cop_position, 1);
}


/**
 * @brief Hierarchy 1: fixed footstep positions
 *
 * @param[in] opt_problem       hierarchy
 */
void setupHierarchy_v1(humoto::OptimizationProblem &opt_problem)
{
    // tasks, which are used in the control problem
    humoto::TaskSharedPointer    task_com_velocity    (new humoto::wpg04::TaskCoMVelocity   );
    humoto::TaskSharedPointer    task_cop_position    (new humoto::wpg04::TaskCoPPosition   );
    humoto::TaskSharedPointer    task_cop_velocity    (new humoto::wpg04::TaskCoPVelocity   );
    humoto::TaskSharedPointer    task_footstep_bounds (new humoto::wpg04::TaskFootstepBounds(true));
    humoto::TaskSharedPointer    task_cop_bounds      (new humoto::wpg04::TaskCoPBounds     );

    // reset the optimization problem
    opt_problem.reset(2);

    // push tasks into the stack/hierarchy
    opt_problem.pushTask(task_footstep_bounds, 0);
    opt_problem.pushTask(task_cop_bounds, 0);
    opt_problem.pushTask(task_com_velocity, 1);
    opt_problem.pushTask(task_cop_velocity, 1);
    opt_problem.pushTask(task_cop_position, 1);
}


/**
 * @brief Hierarchy 2: basic with terminal constraint
 *
 * @param[in] opt_problem       hierarchy
 */
void setupHierarchy_v2(humoto::OptimizationProblem &opt_problem)
{
    // tasks, which are used in the control problem
    humoto::TaskSharedPointer    task_com_velocity        (new humoto::wpg04::TaskCoMVelocity       );
    humoto::TaskSharedPointer    task_cop_position        (new humoto::wpg04::TaskCoPPosition       );
    humoto::TaskSharedPointer    task_cop_velocity        (new humoto::wpg04::TaskCoPVelocity       );
    humoto::TaskSharedPointer    task_footstep_bounds     (new humoto::wpg04::TaskFootstepBounds    );
    humoto::TaskSharedPointer    task_cop_bounds          (new humoto::wpg04::TaskCoPBounds         );
    humoto::TaskSharedPointer    task_terminal_constraint (new humoto::wpg04::TaskTerminalConstraint);

    // reset the optimization problem
    opt_problem.reset(2);

    // push tasks into the stack/hierarchy
    opt_problem.pushTask(task_footstep_bounds, 0);
    opt_problem.pushTask(task_cop_bounds, 0);
    opt_problem.pushTask(task_terminal_constraint, 0);
    opt_problem.pushTask(task_com_velocity, 1);
    opt_problem.pushTask(task_cop_velocity, 1);
    opt_problem.pushTask(task_cop_position, 1);
}

/**
 * @brief Hierarchy 3: Terminal constraint is on a separate second level
 *
 * @param[in] opt_problem       hierarchy
 */
void setupHierarchy_v3(humoto::OptimizationProblem &opt_problem)
{
    // tasks, which are used in the control problem
    humoto::TaskSharedPointer   task_com_velocity        (new humoto::wpg04::TaskCoMVelocity       );
    humoto::TaskSharedPointer   task_cop_position        (new humoto::wpg04::TaskCoPPosition       );
    humoto::TaskSharedPointer   task_cop_velocity        (new humoto::wpg04::TaskCoPVelocity       );
    humoto::TaskSharedPointer   task_footstep_bounds     (new humoto::wpg04::TaskFootstepBounds    );
    humoto::TaskSharedPointer   task_cop_bounds          (new humoto::wpg04::TaskCoPBounds         );
    humoto::TaskSharedPointer   task_terminal_constraint (new humoto::wpg04::TaskTerminalConstraint);

    // reset the optimization problem
    opt_problem.reset(3);

    // push tasks into the stack/hierarchy
    opt_problem.pushTask(task_footstep_bounds, 0);
    opt_problem.pushTask(task_cop_bounds, 0);
    opt_problem.pushTask(task_terminal_constraint, 1);
    opt_problem.pushTask(task_com_velocity, 2);
    opt_problem.pushTask(task_cop_velocity, 2);
    opt_problem.pushTask(task_cop_position, 2);
}


/**
 * @brief Hierarchy 4: with infeasible task
 *
 * @param[in] opt_problem       hierarchy
 */
void setupHierarchy_v4(humoto::OptimizationProblem &opt_problem)
{
    // tasks, which are used in the control problem
    humoto::TaskSharedPointer   task_com_velocity    (new humoto::wpg04::TaskCoMVelocity   );
    humoto::TaskSharedPointer   task_cop_position    (new humoto::wpg04::TaskCoPPosition   );
    humoto::TaskSharedPointer   task_cop_velocity    (new humoto::wpg04::TaskCoPVelocity   );
    humoto::TaskSharedPointer   task_footstep_bounds (new humoto::wpg04::TaskFootstepBounds);
    humoto::TaskSharedPointer   task_cop_bounds      (new humoto::wpg04::TaskCoPBounds     );
    humoto::TaskSharedPointer   task_infeasible      (new humoto::TaskInfeasibleInequality );

    // reset the optimization problem
    opt_problem.reset(2);

    // push tasks into the stack/hierarchy
    opt_problem.pushTask(task_footstep_bounds, 0);
    opt_problem.pushTask(task_cop_bounds, 0);
    opt_problem.pushTask(task_infeasible, 0);
    opt_problem.pushTask(task_com_velocity, 1);
    opt_problem.pushTask(task_cop_velocity, 1);
    opt_problem.pushTask(task_cop_position, 1);
}


/**
 * @brief Hierarchy 5: low gain for CoP centering (many active constraints)
 *
 * @param[in] opt_problem       hierarchy
 */
void setupHierarchy_v5(humoto::OptimizationProblem &opt_problem)
{
    // tasks, which are used in the control problem
    humoto::TaskSharedPointer   task_com_velocity    (new humoto::wpg04::TaskCoMVelocity   );
    humoto::TaskSharedPointer   task_cop_position    (new humoto::wpg04::TaskCoPPosition(0.707106781186548));
    humoto::TaskSharedPointer   task_cop_velocity    (new humoto::wpg04::TaskCoPVelocity   );
    humoto::TaskSharedPointer   task_footstep_bounds (new humoto::wpg04::TaskFootstepBounds);
    humoto::TaskSharedPointer   task_cop_bounds      (new humoto::wpg04::TaskCoPBounds     );

    // reset the optimization problem
    opt_problem.reset(2);

    // push tasks into the stack/hierarchy
    opt_problem.pushTask(task_footstep_bounds, 0);
    opt_problem.pushTask(task_cop_bounds, 0);
    opt_problem.pushTask(task_com_velocity, 1);
    opt_problem.pushTask(task_cop_velocity, 1);
    opt_problem.pushTask(task_cop_position, 1);
}
