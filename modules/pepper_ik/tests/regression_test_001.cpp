/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "gtest/gtest.h"


// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
// common & abstract classes
#include "humoto/humoto.h"
// walking-related classes
#include "humoto/walking.h"
// specific solver (many can be included simultaneously)
#include "humoto/kktsolver.h"
// specific control problem (many can be included simultaneously)
#include "humoto/pepper_ik.h"

#define MODEL_FEATURES humoto::pepper_ik::ModelFeatures::FIXED_WHEELS | humoto::pepper_ik::ModelFeatures::ROOT_PLANAR

#include "utilities_regression.h"

namespace humoto_tests
{
    namespace pepper_ik
    {
        class TestFixture : public ::testing::Test
        {
            protected:
                humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>   ref_generalized_coordinates_;
                std::size_t     number_of_iterations_;
                double          x_increment_;
                double          y_increment_;


                humoto::pepper_ik::ConfigurableOptimizationProblem<MODEL_FEATURES>             opt_problem;
                humoto::kktsolver::SolverParameters     solver_parameters;
                humoto::kktsolver::Solver               solver;
                humoto::Solution                        solution;


                humoto::pepper_ik::WBCParameters                            wbc_parameters;
                humoto::pepper_ik::WholeBodyController<MODEL_FEATURES>      wbc;
                humoto::pepper_ik::Model<MODEL_FEATURES>                    model;

                humoto::pepper_ik::MotionParameters     motion_parameters;

                humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>   generalized_coordinates_;


            protected:
                TestFixture()
                {
                    x_increment_ = 0.0005;
                    y_increment_ = -0.0006;
                    // this should be even
                    number_of_iterations_ = 500;

                    std::string config_name_features_part;
                    if ((MODEL_FEATURES) & humoto::pepper_ik::ModelFeatures::ROOT_PLANAR)
                    {
                        config_name_features_part = "planar";
                    }
                    else
                    {
                        HUMOTO_THROW_MSG("Unsupported model features, update the test.");
                    }

                    model.loadParameters(g_config_path + "pepper_fixedwheels_roottibia_" + config_name_features_part + ".urdf");

                    wbc_parameters.readConfig(g_config_path + "wbc_parameters.yaml");
                    motion_parameters.readConfig(g_config_path + "motion_parameters_default.yaml");
                    generalized_coordinates_.readConfig(g_config_path + "initial_state_pepper_ik_" + config_name_features_part + "_default.yaml");

                    opt_problem.readConfig(g_config_path + "hierarchies_" + config_name_features_part + ".yaml", true, "Hierarchy00");
                    model.updateState(generalized_coordinates_);

                    ref_generalized_coordinates_ = generalized_coordinates_;
                    ref_generalized_coordinates_.root_pose_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::ROOT_TRANSLATION_X]
                        += number_of_iterations_ * x_increment_;
                    ref_generalized_coordinates_.root_pose_[humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::ROOT_TRANSLATION_Y]
                        += number_of_iterations_ * y_increment_;
                }



                /**
                 * @brief Simple control loop
                 *
                 * @return status
                 */
                humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>
                    run(const humoto::kktsolver::SolverParameters::Method solution_method)
                {
                    solver_parameters.solution_method_ = solution_method;
                    solver.setParameters(solver_parameters);

                    for (std::size_t i = 0; i < number_of_iterations_; ++i)
                    {
                        motion_parameters.base_com_position_.x() += x_increment_;
                        motion_parameters.body_com_position_.x() += x_increment_;

                        motion_parameters.base_com_position_.y() += y_increment_;
                        if (i % 2 == 1)
                        {
                            motion_parameters.body_com_position_.y() += 2*y_increment_;
                        }


                        model.saveCurrentState();
                        for (std::size_t j = 0; ; ++j)
                        {
                            if (j == wbc_parameters.maximal_number_of_iterations_)
                            {
                                HUMOTO_THROW_MSG("Maximal number of IK iterations reached.");
                            }
                            // prepare control problem for new iteration
                            if (wbc.update(model, motion_parameters) != humoto::ControlProblemStatus::OK)
                            {
                                HUMOTO_THROW_MSG("Control problem could not be updated.");
                            }


                            // form an optimization problem
                            opt_problem.form(solution, model, wbc);

                            // solve an optimization problem
                            solver.solve(solution, opt_problem);


                            // extract next model state from the solution and update model
                            generalized_coordinates_ = wbc.getNextGeneralizedCoordinates(solution, model);
                            model.updateState(generalized_coordinates_);

                            if (solution.x_.lpNorm<Eigen::Infinity>() < wbc_parameters.joint_angle_error_tolerance_)
                            {
                                break;
                            }
                        }
                    }

                    /*
                    humoto::Logger logger("debug.m");
                    generalized_coordinates_.log(logger);
                    ref_generalized_coordinates_.log(logger);
                    */
                    return (generalized_coordinates_);
                }
        };

        // Tests that the last MPC solution matches ref
        TEST_F(TestFixture, StateMatchReference_LU)
        {
            humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES> generalized_coordinates
                = run(humoto::kktsolver::SolverParameters::LU);

            ASSERT_TRUE(ref_generalized_coordinates_.root_pose_.isApprox(
                        generalized_coordinates.root_pose_,
                        wbc_parameters.joint_angle_error_tolerance_));

            ASSERT_TRUE(ref_generalized_coordinates_.joint_angles_.isApprox(
                        generalized_coordinates.joint_angles_,
                        wbc_parameters.joint_angle_error_tolerance_));
        }


        TEST_F(TestFixture, StateMatchReference_CONSTRAINT_ELIMINATION_LLT)
        {
            humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES> generalized_coordinates
                = run(humoto::kktsolver::SolverParameters::CONSTRAINT_ELIMINATION_LLT);

            ASSERT_TRUE(ref_generalized_coordinates_.root_pose_.isApprox(
                        generalized_coordinates.root_pose_,
                        wbc_parameters.joint_angle_error_tolerance_));

            ASSERT_TRUE(ref_generalized_coordinates_.joint_angles_.isApprox(
                        generalized_coordinates.joint_angles_,
                        wbc_parameters.joint_angle_error_tolerance_));
        }
    }
}


HUMOTO_DEFINE_REGRESSION_TEST_MAIN()
