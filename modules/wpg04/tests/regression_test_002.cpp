/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <iostream>
#include <limits>
#include <iomanip>

#define HUMOTO_GLOBAL_LOGGER_ENABLED

// common & abstract classes (must be first)
#include "humoto/humoto.h"
// specific solver (many can be included simultaneously)
#include "humoto/qpoases.h"
// specific control problem (many can be included simultaneously)
#include "humoto/wpg04.h"

#include "utilities.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);


/**
 * @brief Simple control loop
 *
 * @param[in] argc number of arguments
 * @param[in] argv arguments
 *
 * @return status
 */
int main(int argc, char **argv)
{
    try
    {
        // optimization problem (a stack of tasks / hierarchy)
        humoto::OptimizationProblem       opt_problem;
        // a solver which is giong to be used
        humoto::qpoases::Solver           solver;
        // solution
        humoto::qpoases::Solution         solution;
        // options for walking
        humoto::wpg04::WalkParameters        walk_parameters;
        walk_parameters.com_velocity_ << 0.1, 0.;
        walk_parameters.first_stance_com_velocity_ = walk_parameters.com_velocity_;
        walk_parameters.num_steps_ = 7;
        //FSM for walking
        humoto::walking::StanceFiniteStateMachine stance_fsm(walk_parameters);
        // model representing the controlled system
        humoto::wpg04::Model                    model;
        // parameters of the control problem
        humoto::wpg04::MPCParameters          wpg_parameters;
        // control problem, which is used to construct an optimization problem
        humoto::wpg04::MPCforWPG              wpg(wpg_parameters);

        setupHierarchy_v0(opt_problem);

        humoto::wpg04::ModelState model_state;

        humoto::Timer       timer;
        humoto::Logger      logger(std::cout);
        humoto::LogEntryName prefix;

        humoto::ActiveSet active_set_guess;
        humoto::ActiveSet active_set_actual;
        humoto::ActiveSet active_set_determined;

        humoto::Violations  violations;

        humoto::qpoases::Solution   old_solution;
        humoto::Solution            solution_guess;

        // run only one iteration
        for (unsigned int i = 0; i < 1; ++i)
        {
            prefix = humoto::LogEntryName("humoto").add(i);

            timer.start();
            // prepare control problem for new iteration
            if (wpg.update(model, stance_fsm, walk_parameters) != humoto::ControlProblemStatus::OK)
            {
                break;
            }

            // form an optimization problem
            opt_problem.form(solution, solution_guess, active_set_guess, model, wpg, old_solution);

            //solve an optimization problem
            solver.solve(solution, active_set_actual, opt_problem, solution_guess, active_set_guess);
            opt_problem.processActiveSet(active_set_actual);

            timer.stop();
            //HUMOTO_LOG_RAW(timer);
            opt_problem.determineActiveSet(active_set_determined, solution);
            opt_problem.computeViolations(violations, solution);


            old_solution = solution;

            //========================
            // logging
            violations.log(logger, prefix, "violations");
            active_set_guess.log(logger, prefix, "active_set_guess");
            active_set_actual.log(logger, prefix, "active_set_actual");
            active_set_determined.log(logger, prefix, "active_set_determined");
            opt_problem.log(logger, prefix);
            solver.log(logger, prefix);
            solution.log(logger, prefix);
            solution_guess.log(logger, prefix, "solution_guess");
            old_solution.log(logger, prefix, "old_solution");
            stance_fsm.log(logger, prefix);
            wpg.log(logger, prefix);
            model.log(logger, prefix);
            //========================

            // extract next model state from the solution and update model
            stance_fsm.shiftTime(wpg_parameters.sampling_time_ms_);
            model_state = wpg.getNextModelState(solution, stance_fsm, model);
            model.updateState(model_state);
        }
    }
    catch (const std::exception &e)
    {
        HUMOTO_LOG_RAW(e.what());
        exit(-1);
    }

    return (0);
}
