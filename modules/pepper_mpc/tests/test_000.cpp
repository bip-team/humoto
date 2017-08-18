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

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
// common & abstract classes
#include "humoto/humoto.h"
// specific solver (many can be included simultaneously)
#include "humoto/qpoases.h"
// specific control problem (many can be included simultaneously)
#include "humoto/pepper_mpc.h"


#include "utilities.h"


HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);
//HUMOTO_INITIALIZE_GLOBAL_LOGGER("test_001.m");


/**
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
        humoto::pepper_mpc::ConfigurableOptimizationProblem     opt_problem;


        // parameters of the solver
        humoto::qpoases::SolverParameters   solver_parameters;
        // a solver which is giong to be used
        humoto::qpoases::Solver             solver (solver_parameters);
        // solution
        humoto::qpoases::Solution           solution;


        humoto::pepper_mpc::RobotParameters         robot_parameters;
        robot_parameters.readConfig<humoto::config::yaml::Reader>(config_path + "robot_parameters.yaml");
        // model representing the controlled system
        humoto::pepper_mpc::Model                   model(robot_parameters);

        // parameters of the control problem
        humoto::pepper_mpc::MPCParameters           mg_parameters;
        mg_parameters.readConfig<humoto::config::yaml::Reader>(config_path + "mpc_parameters.yaml");
        // control problem, which is used to construct an optimization problem
        humoto::pepper_mpc::MPCforMG                mg(mg_parameters);


        // options for walking
        humoto::pepper_mpc::MotionParameters        motion_parameters;
        motion_parameters.readConfig<humoto::config::yaml::Reader>(config_path + "motion_parameters_circle.yaml");

        switch (motion_parameters.motion_mode_)
        {
            case humoto::pepper_mpc::MotionMode::MAINTAIN_POSITION:
                opt_problem.readConfig<humoto::config::yaml::Reader>(config_path + "hierarchies.yaml", true, "Hierarchy01");
                break;
            case humoto::pepper_mpc::MotionMode::MAINTAIN_VELOCITY:
                opt_problem.readConfig<humoto::config::yaml::Reader>(config_path + "hierarchies.yaml", true, "Hierarchy00");
                break;
            default:
                HUMOTO_THROW_MSG("Unsupported motion mode.");
        }

        humoto::pepper_mpc::ModelState   model_state;
        model_state.readConfig<humoto::config::yaml::Reader>(config_path + "initial_state_pepper.yaml");
        model.updateState(model_state);


        /*
        model_state.writeConfig("initial_state_pepper.yaml");
        mg_parameters.writeConfig("mpc_parameters.yaml");
        motion_parameters.writeConfig("motion_parameters.yaml");
        robot_parameters.writeConfig("robot_parameters.yaml");
        */


        humoto::LogEntryName prefix;
        humoto::Timer       timer;

        humoto::Logger      logger(log_file_name);


        for ( std::size_t i = 0;; ++i)
        {
            prefix = humoto::LogEntryName("humoto").add(i);


            timer.start();
            // prepare control problem for new iteration
            if (mg.updateAndShift(  motion_parameters,
                                    model,
                                    mg_parameters.sampling_time_ms_) != humoto::ControlProblemStatus::OK)
            {
                break;
            }


            // form an optimization problem
            opt_problem.form(solution, model, mg);

            // solve an optimization problem
            solver.solve(solution, opt_problem);
            timer.stop();
            HUMOTO_LOG_RAW(timer);


            //========================
            // logging
            opt_problem.log(logger, prefix);
            solver.log(logger, prefix);
            mg.log(logger, prefix);
            model.log(logger, prefix);
            solution.log(logger, prefix);
            //========================


            // extract next model state from the solution and update model
            model_state = mg.getNextModelState(solution, model);
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
