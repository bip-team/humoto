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
#include <fenv.h>

#define HUMOTO_GLOBAL_LOGGER_ENABLED

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
// common & abstract classes
#include "humoto/humoto.h"
// specific solver (many can be included simultaneously)
#include "humoto/kktsolver.h"
// specific control problem (many can be included simultaneously)
#include "humoto/pepper_ik.h"

#include "utilities.h"


#define MODEL_FEATURES humoto::pepper_ik::ModelFeatures::FIXED_WHEELS | humoto::pepper_ik::ModelFeatures::ROOT_PLANAR

#include "hierarchy.h"


//#define PERFORMANCE_TEST

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

//    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW | FE_UNDERFLOW);

    try
    {
        // optimization problem (a stack of tasks / hierarchy)
        humoto::OptimizationProblem     opt_problem;

        // parameters of the solver
        humoto::kktsolver::SolverParameters   solver_parameters;
//        solver_parameters.solution_method_ = humoto::kktsolver::SolverParameters::CONSTRAINT_ELIMINATION_LLT;
        // a solver which is giong to be used
        humoto::kktsolver::Solver             solver (solver_parameters);
        // solution
        humoto::Solution           solution;



        // parameters of the control problem
#ifdef PERFORMANCE_TEST
        humoto::pepper_ik::WBCParameters           wbc_parameters;
#else
        humoto::pepper_ik::WBCParameters           wbc_parameters(config_path + "wbc_parameters.yaml");
#endif // PERFORMANCE_TEST
        // control problem, which is used to construct an optimization problem
        humoto::pepper_ik::WholeBodyController<MODEL_FEATURES>     wbc;

        // model representing the controlled system
        humoto::pepper_ik::Model<MODEL_FEATURES>                   model;
        model.loadParameters(config_path + "pepper_fixedwheels_roottibia_planar.urdf");


        // options for walking
        humoto::pepper_ik::MotionParameters     motion_parameters;


#ifdef PERFORMANCE_TEST
        setupHierarchy_v0(opt_problem);
#else
        setupHierarchy_v0(opt_problem);
        //setupHierarchy_v2(opt_problem);
#endif // PERFORMANCE_TEST


        humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>   generalized_coordinates;
        model.updateState(generalized_coordinates);


        /*
        motion_parameters.writeConfig("motion_parameters.yaml");
        wbc_parameters.writeConfig("wbc_parameters.yaml");
        generalized_coordinates.writeConfig("initial_state_pepper_ik.yaml");
        */

        humoto::Timer       timer;

#ifndef PERFORMANCE_TEST
        std::string         commands_filename = std::string(argv[0]) + ".dat";
        humoto::Logger      logger(log_file_name);
        humoto::LogEntryName prefix;
        std::ofstream       commands(commands_filename.c_str());

        commands << std::setprecision(std::numeric_limits<double>::digits10);
#endif // PERFORMANCE_TEST

        std::size_t counter = 0;
        for (std::size_t i = 0; i < 100; ++i)
        {
            //motion_parameters.body_com_position_.z() = 0.75;
            /*
            motion_parameters.base_com_position_.x() += 0.05;
            motion_parameters.body_com_position_.x() += 0.05;
            motion_parameters.body_com_position_.y() = (i % 2 == 0 ? -0.01 : 0.01);
            */
            motion_parameters.base_com_position_.x() += 0.0005;
            motion_parameters.body_com_position_.x() += 0.0005;


            HUMOTO_LOG_RAW("===================");
            HUMOTO_LOG("iter = ", i);
            model.saveCurrentState();
            for (std::size_t j = 0; ; ++j, ++counter)
            {
                if (j == wbc_parameters.maximal_number_of_iterations_)
                {
                    HUMOTO_THROW_MSG("Maximal number of IK iterations reached.");
                }
                timer.start();
                // prepare control problem for new iteration
                if (wbc.update(model, motion_parameters) != humoto::ControlProblemStatus::OK)
                {
                    HUMOTO_THROW_MSG("Control problem could not be updated.");
                }


                // form an optimization problem
                opt_problem.form(solution, model, wbc);

                // solve an optimization problem
                solver.solve(solution, opt_problem);
                timer.stop();
                HUMOTO_LOG_RAW(timer);

#ifndef PERFORMANCE_TEST
                //========================
                // logging
                prefix = humoto::LogEntryName("ik_loop").add(i).add(j);

                opt_problem.log(logger, prefix);
                solver.log(logger, prefix);
                solution.log(logger, prefix);
                //wbc.log(logger, prefix);
                //model.log(logger, prefix);
                /*
                source test_000.m
                loop_n = 2;
                dq = [];
                for i = 1:numel(ik_loop{loop_n})
                    dq = [dq, ik_loop{loop_n}{i}.solution.x];
                end
                */
                //========================
#endif // PERFORMANCE_TEST

                // extract next model state from the solution and update model
                generalized_coordinates = wbc.getNextGeneralizedCoordinates(solution, model);
                model.updateState(generalized_coordinates);


                humoto::pepper_ik::MotionParameters motion_parameters_errors;
                model.getStateError(motion_parameters_errors, motion_parameters);

                HUMOTO_LOG_RAW("---");

                HUMOTO_LOG_RAW(motion_parameters_errors.base_com_position_);
                HUMOTO_LOG_RAW(motion_parameters_errors.base_orientation_rpy_);
                HUMOTO_LOG_RAW(motion_parameters_errors.body_com_position_);


                if (solution.x_.lpNorm<Eigen::Infinity>() < wbc_parameters.joint_angle_error_tolerance_)
                {
                    break;
                }
            }
            humoto::pepper_ik::RobotCommand command;
            model.getRobotCommand(command, wbc_parameters.control_interval_);

#ifndef PERFORMANCE_TEST
            commands
                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HeadPitch] << " "
                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HeadYaw] << " "

                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LElbowRoll] << " "
                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LElbowYaw] << " "
                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RElbowRoll] << " "
                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RElbowYaw] << " "

                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LWristYaw] << " "
                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RWristYaw] << " "

                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LShoulderPitch] << " "
                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LShoulderRoll] << " "
                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RShoulderPitch] << " "
                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RShoulderRoll] << " "

                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HipPitch] << " "
                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HipRoll] << " "
                    << command.joint_angles_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::KneePitch] << " "

                    << command.wheel_velocities_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::WHEEL_FRONT_RIGHT] << " "
                    << command.wheel_velocities_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::WHEEL_FRONT_LEFT] << " "
                    << command.wheel_velocities_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::WHEEL_BACK] << std::endl;


            prefix = humoto::LogEntryName("control_loop").add(i);
            generalized_coordinates.log(logger, prefix);
            command.log(logger, prefix);
            model.log(logger, prefix);
            /*
            source test_000.m
            q = [];
            for i = 1:numel(control_loop)
                q_tmp = [control_loop{i}.generalized_coordinates.root_pose, control_loop{i}.joint_angles]
                q = [q, q_tmp];
            end
            */
#endif // PERFORMANCE_TEST
        }
#ifndef PERFORMANCE_TEST
        commands.close();
#endif // PERFORMANCE_TEST

        HUMOTO_LOG_RAW("---");
        HUMOTO_LOG_RAW( humoto::rigidbody::convertMatrixToEulerAngles(
                                model.getTagOrientation(model.getLinkTag("Head")),
                                humoto::rigidbody::EulerAngles::RPY));
    }
    catch (const std::exception &e)
    {
        HUMOTO_LOG_RAW(e.what());
        exit(-1);
    }

    return (0);
}
