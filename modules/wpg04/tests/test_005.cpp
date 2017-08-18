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

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
// common & abstract classes
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
    std::string config_path = humoto_tests::getConfigPath(argc, argv, 1);
    std::string log_file_name = humoto_tests::getLogFileName(argc, argv, 2);

    try
    {
        // optimization problem (a stack of tasks / hierarchy)
        humoto::wpg04::ConfigurableOptimizationProblem               opt_problem;
        // a solver which is giong to be used
        humoto::qpoases::Solver                   solver;
        // solution
        humoto::qpoases::Solution                 solution;
        // options for walking
        humoto::wpg04::WalkParameters             walk_parameters;
        walk_parameters.readConfig<humoto::config::yaml::Reader>(config_path + "walk_parameters__stand_still.yaml");
        //FSM for walking
        humoto::walking::StanceFiniteStateMachine stance_fsm(walk_parameters);
        // model representing the controlled system
        humoto::walking::RobotFootParameters      robot_parameters;
        robot_parameters.readConfig<humoto::config::yaml::Reader>(config_path + "robot_hrp4.yaml");
        humoto::wpg04::Model                      model;
        // parameters of the control problem
        humoto::wpg04::MPCParameters              wpg_parameters;
        // control problem, which is used to construct an optimization problem
        humoto::wpg04::MPCforWPG                  wpg(wpg_parameters);

        opt_problem.readConfig<humoto::config::yaml::Reader>(config_path + "/hierarchies.yaml", "Hierarchy00");

        humoto::wpg04::ModelState                 model_state;
        model_state.readConfig<humoto::config::yaml::Reader>(config_path + "initial_state_hrp4.yaml");

        model.setFootParameters(robot_parameters);
        model.updateState(model_state);


        humoto::Timer       timer;

        humoto::Logger      logger(log_file_name);
        humoto::LogEntryName        prefix;

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
