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
// specific control problem (many can be included simultaneously)
#include "humoto/pepper_ik.h"

#include "utilities.h"


#define MODEL_FEATURES humoto::pepper_ik::ModelFeatures::FIXED_WHEELS | humoto::pepper_ik::ModelFeatures::ROOT_PLANAR


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


    humoto::Logger      logger(log_file_name);

    try
    {
        double x_first = -0.15;
        double x_last = 0.15 + 1e-6;
        double x_step = 0.015;

        double y_first = -0.1;
        double y_last = 0.1 + 1e-6;
        double y_step = 0.01;

        double z_first = -0.1;
        double z_last = 0.0 + 1e-6;
        double z_step = 0.01;


        Eigen::MatrixXd     points;
        Eigen::VectorXd     feasibility_flags;
        humoto::EigenIndex  points_i = 0;

        points.resize(  static_cast<int>(1+round((x_last-x_first)/x_step))
                        *
                        static_cast<int>(1+round((y_last-y_first)/y_step))
                        *
                        static_cast<int>(1+round((z_last-z_first)/z_step)),
                        3);
        feasibility_flags.setZero(  static_cast<int>(1+round((x_last-x_first)/x_step))
                                    *
                                    static_cast<int>(1+round((y_last-y_first)/y_step))
                                    *
                                    static_cast<int>(1+round((z_last-z_first)/z_step)));

        humoto::LogEntryName prefix;

        for (double x = x_first; x <= x_last; x += x_step)
        {
            for (double y = y_first; y <= y_last; y += y_step)
            {
                for (double z = z_first; z <= z_last; z += z_step, ++points_i)
                {
                    // optimization problem (a stack of tasks / hierarchy)
                    humoto::pepper_ik::ConfigurableOptimizationProblem<MODEL_FEATURES>     opt_problem;
                    opt_problem.readConfig(config_path + "hierarchies_planar.yaml", true, "Hierarchy01");


                    // parameters of the solver
                    humoto::qpoases::SolverParameters   solver_parameters;
                    //solver_parameters.crash_on_any_failure_ = false;
                    // a solver which is giong to be used
                    humoto::qpoases::Solver             solver (solver_parameters);
                    // solution
                    humoto::qpoases::Solution           solution;



                    // parameters of the control problem
                    humoto::pepper_ik::WBCParameters           wbc_parameters(config_path + "wbc_parameters.yaml");
                    wbc_parameters.maximal_number_of_iterations_    = 500;

                    // control problem, which is used to construct an optimization problem
                    humoto::pepper_ik::WholeBodyController<MODEL_FEATURES>     wbc;

                    // model representing the controlled system
                    humoto::pepper_ik::Model<MODEL_FEATURES>                   model;
                    model.loadParameters(config_path + "pepper_fixedwheels_roottibia_planar.urdf");


                    // options for walking
                    humoto::pepper_ik::MotionParameters     motion_parameters(config_path + "motion_parameters.yaml");


                    humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>   generalized_coordinates;
                    //humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>   generalized_coordinates(config_path + "initial_state_pepper_ik_planar.yaml");
                    model.updateState(generalized_coordinates);


                    humoto::Timer       timer;


                    motion_parameters.body_com_position_.x() = -4.18093905757091e-03 + x;
                    motion_parameters.body_com_position_.y() = 0.0 + y;
                    motion_parameters.body_com_position_.z() = 0.763104597149514 + z;


                    model.saveCurrentState();
                    for (std::size_t j = 0; ; ++j)
                    {
                        if (j == wbc_parameters.maximal_number_of_iterations_)
                        {
                            feasibility_flags(points_i) = 1;
                            break;
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
                        try
                        {
                            solver.solve(solution, opt_problem);
                        }
                        catch(std::exception &e)
                        {
                            HUMOTO_LOG_RAW(e.what());
                            // infeasible or other error
                            feasibility_flags(points_i) = 2;
                            break;
                        }
                        timer.stop();
                        HUMOTO_LOG_RAW(points_i*10000 + j);


                        //========================
                        // logging
                        /*
                        prefix = humoto::LogEntryName("ik_loop").add(points_i).add(j);

                        opt_problem.log(logger, prefix);
                        solver.log(logger, prefix);
                        solution.log(logger, prefix);
                        */
                        //wbc.log(logger, prefix);
                        //model.log(logger, prefix);
                        //========================


                        // extract next model state from the solution and update model
                        generalized_coordinates = wbc.getNextGeneralizedCoordinates(solution, model);
                        model.updateState(generalized_coordinates);

                        if (solution.x_.lpNorm<Eigen::Infinity>() < wbc_parameters.joint_angle_error_tolerance_)
                        {
                            feasibility_flags(points_i) = 3;
                            break;
                        }
                    }

                    try
                    {
                        model.checkCurrentState();
                    }
                    catch(std::exception &e)
                    {
                        HUMOTO_LOG_RAW(e.what());
                        // joint bounds violated
                        feasibility_flags(points_i) = 4;
                    }


                    humoto::pepper_ik::MotionParameters         motion_parameters_errors;
                    model.getStateError(motion_parameters_errors, motion_parameters);

                    /*
                    HUMOTO_LOG_RAW("---");
                    HUMOTO_LOG_RAW(motion_parameters_errors.base_com_position_.norm());
                    HUMOTO_LOG_RAW(motion_parameters_errors.base_orientation_rpy_.norm());
                    HUMOTO_LOG_RAW(motion_parameters_errors.body_com_position_.norm());
                    */


                    points(points_i, 0) = motion_parameters.body_com_position_.x();
                    points(points_i, 1) = motion_parameters.body_com_position_.y();
                    points(points_i, 2) = motion_parameters.body_com_position_.z();

                    if (    motion_parameters_errors.base_com_position_.norm()
                            + motion_parameters_errors.base_orientation_rpy_.norm()
                            + motion_parameters_errors.body_com_position_.norm()  <  wbc_parameters.motion_parameters_tolerance_)
                    {
                        feasibility_flags(points_i) = -1;
                    }


                    prefix = humoto::LogEntryName("point").add(points_i);
                    /*
                    generalized_coordinates.log(logger, prefix);
                    model.log(logger, prefix);
                    */
                }
            }
        }

        std::string feasibility_logfile_name = argv[0];
        humoto::Logger      logger_feasibility(feasibility_logfile_name + "_feasibility.m");

        logger_feasibility.log(humoto::LogEntryName("points"), points);
        logger_feasibility.log(humoto::LogEntryName("feasibility_flags"), feasibility_flags);
    }
    catch (const std::exception &e)
    {
        HUMOTO_LOG_RAW(e.what());
        exit(-1);
    }

    return (0);
}
