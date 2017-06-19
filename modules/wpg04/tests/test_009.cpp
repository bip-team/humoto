/**
    @file
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
#include "humoto/quadprogpp.h"
// specific control problem (many can be included simultaneously)
#include "humoto/wpg04.h"

#include "utilities.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);
//HUMOTO_INITIALIZE_GLOBAL_LOGGER("test_001.m");



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
    std::string     log_file_name = humoto_tests::getLogFileName(argc, argv);

    try
    {
        // optimization problem (a stack of tasks / hierarchy)
        humoto::OptimizationProblem               opt_problem;
        // a solver which is giong to be used
        humoto::quadprogpp::SolverParameters        solver_parameters;
        //solver_parameters.crash_on_any_failure_ = false;
        humoto::quadprogpp::Solver                   solver(solver_parameters);
        // solution
        humoto::quadprogpp::Solution                 solution;
        // options for walking
        humoto::wpg04::WalkParameters                walk_parameters;
        walk_parameters.com_velocity_ << 0.1, 0.;
        walk_parameters.first_stance_com_velocity_ = walk_parameters.com_velocity_;
        walk_parameters.num_steps_ = 7;
        //FSM for walking
        humoto::walking::StanceFiniteStateMachine stance_fsm(walk_parameters);
        // model representing the controlled system
        humoto::wpg04::Model                      model;
        // parameters of the control problem
        humoto::wpg04::MPCParameters              wpg_parameters;
        // control problem, which is used to construct an optimization problem
        humoto::wpg04::MPCforWPG                  wpg(wpg_parameters);


        setupHierarchy_v0(opt_problem);

        humoto::wpg04::ModelState model_state;


        humoto::Timer       timer;
        humoto::Logger      logger(log_file_name);
        humoto::LogEntryName    prefix;

        for (unsigned int i = 0;; ++i)
        {
            prefix = humoto::LogEntryName("humoto").add(i);

            timer.start();
            // prepare control problem for new iteration
            if (wpg.update(model, stance_fsm, walk_parameters) != humoto::ControlProblemStatus::OK)
            {
                break;
            }

            // form an optimization problem
            opt_problem.form(solution, model, wpg);

            // solve an optimization problem
            solver.solve(solution, opt_problem);

            timer.stop();
            HUMOTO_LOG_RAW(timer);

            //========================
            // logging
            opt_problem.log(logger, prefix);
            solver.log(logger, prefix);
            solution.log(logger, prefix);
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

