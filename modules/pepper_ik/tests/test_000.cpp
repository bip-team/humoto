/**
    @file
    @author  Alexander Sherikov
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
//#include "humoto/kktsolver.h"
// specific control problem (many can be included simultaneously)
#include "humoto/pepper_ik.h"

#include "utilities.h"


#define MODEL_FEATURES humoto::pepper_ik::ModelFeatures::FIXED_WHEELS | humoto::pepper_ik::ModelFeatures::ROOT_TORSO


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
        humoto::pepper_ik::ConfigurableOptimizationProblem<MODEL_FEATURES>     opt_problem;
        opt_problem.readConfig<humoto::config::yaml::Reader>(config_path + "hierarchies_torso.yaml", true, "Hierarchy00");


        // parameters of the solver
        humoto::qpoases::SolverParameters   solver_parameters;
        // a solver which is giong to be used
        humoto::qpoases::Solver             solver (solver_parameters);
        // solution
        humoto::qpoases::Solution           solution;
        humoto::qpoases::Solution           old_solution;
/*
        // parameters of the solver
        humoto::kktsolver::SolverParameters   solver_parameters;
        // a solver which is giong to be used
        humoto::kktsolver::Solver             solver (solver_parameters);
        // solution
        humoto::Solution           solution;
        humoto::Solution           old_solution;
        humoto::Solution                    solution_guess;
*/


        // parameters of the control problem
        //humoto::pepper_ik::WBCParameters           wbc_parameters;
        humoto::pepper_ik::WBCParameters           wbc_parameters;
        wbc_parameters.readConfig<humoto::config::yaml::Reader>(config_path + "wbc_parameters.yaml");
        // control problem, which is used to construct an optimization problem
        humoto::pepper_ik::WholeBodyController<MODEL_FEATURES>     wbc;

        // model representing the controlled system
        humoto::pepper_ik::Model<MODEL_FEATURES>                   model;
        model.loadParameters(config_path + "pepper_fixedwheels.urdf");


        // options for walking
        humoto::pepper_ik::MotionParameters     motion_parameters;
        motion_parameters.readConfig<humoto::config::yaml::Reader>(config_path + "motion_parameters.yaml");


        humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>   generalized_coordinates;
        //humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>   generalized_coordinates(config_path + "initial_state_pepper_ik_default_root.yaml");
        model.updateState(generalized_coordinates);


        /*
        motion_parameters.writeConfig("motion_parameters.yaml");
        wbc_parameters.writeConfig("wbc_parameters.yaml");
        generalized_coordinates.writeConfig("initial_state_pepper_ik.yaml");
        */


        humoto::Timer       timer;

        humoto::Logger      logger(log_file_name);
        humoto::LogEntryName prefix;



        std::size_t counter = 0;
        for (std::size_t i = 0; i < 20; ++i)
        {
            motion_parameters.base_com_position_.x() += 0.05;
            motion_parameters.body_com_position_.x() += 0.05;
            motion_parameters.body_com_position_.y() = (i % 2 == 0 ? -0.01 : 0.01);


            HUMOTO_LOG_RAW("===================");
            HUMOTO_LOG("iter", i);
            for (std::size_t j = 0;; ++j, ++counter)
            {
                timer.start();
                // prepare control problem for new iteration
                if (wbc.update(model, motion_parameters) != humoto::ControlProblemStatus::OK)
                {
                    break;
                }


                // form an optimization problem
                opt_problem.form(solution, model, wbc);
                //opt_problem.form(solution, solution_guess, model, wbc, old_solution);

                // solve an optimization problem
                solver.solve(solution, opt_problem);
                //solver.solve(solution, opt_problem, solution_guess);
                timer.stop();
                HUMOTO_LOG_RAW(timer);

                //========================
                // logging
                prefix = humoto::LogEntryName("ik_loop").add(i).add(j);

                opt_problem.log(logger, prefix);
                solver.log(logger, prefix);
                solution.log(logger, prefix);
                //wbc.log(logger, prefix);
                //model.log(logger, prefix);
                //========================
                /*
                source test_000.m
                loop_n = 2;
                dq = [];
                for i = 1:numel(ik_loop{loop_n})
                    dq = [dq, ik_loop{loop_n}{i}.solution.x];
                end
                */

                // extract next model state from the solution and update model
                generalized_coordinates = wbc.getNextGeneralizedCoordinates(solution, model);
                model.updateState(generalized_coordinates);

                old_solution = solution;

                if (solution.x_.lpNorm<Eigen::Infinity>() < wbc_parameters.joint_angle_error_tolerance_)
                {
                    break;
                }
            }


            prefix = humoto::LogEntryName("control_loop").add(i);
            generalized_coordinates.log(logger, prefix);
            /*
            source test_000.m
            q = [];
            for i = 1:numel(control_loop)
                q_tmp = [control_loop{i}.generalized_coordinates.root_pose, control_loop{i}.joint_angles]
                q = [q, q_tmp];
            end
            */
        }
    }
    catch (const std::exception &e)
    {
        HUMOTO_LOG_RAW(e.what());
        exit(-1);
    }

    return (0);
}
