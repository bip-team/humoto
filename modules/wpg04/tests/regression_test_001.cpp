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
#include "humoto/wpg04.h"

//testing
#include "gtest/gtest.h"
#include "utilities_regression.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);

//#undef HUMOTO_GLOBAL_LOGGER_ENABLED

namespace humoto_tests
{
    namespace wpg04
    {
        class TestBase
        {
            protected:
                VerificationData        reference_;

                // optimization problem (a stack of tasks / hierarchy)
                humoto::wpg04::ConfigurableOptimizationProblem           opt_problem;


                // a solver which is giong to be used
                humoto::qpoases::Solver               solver;
                // parameters of the solver
                humoto::qpoases::SolverParameters     solver_parameters;
                // solution
                humoto::qpoases::Solution             solution;


                // options for walking
                humoto::wpg04::WalkParameters         walk_parameters;
                // FSM for walking
                humoto::walking::StanceFiniteStateMachine stance_fsm;
                // model representing the controlled system
                humoto::wpg04::Model                  model;
                humoto::walking::RobotFootParameters  robot_parameters;
                humoto::wpg04::ModelState             model_state;

                // parameters of the control problem
                humoto::wpg04::MPCParameters          wpg_parameters;
                // control problem, which is used to construct an optimization problem
                humoto::wpg04::MPCforWPG              wpg;


            protected:
                /**
                 * @brief Simple control loop
                 *
                 * @return status
                */
                VerificationData run()
                {
                    for (unsigned int i = 0;; ++i)
                    {
                        // prepare control problem for new iteration
                        if (wpg.update(model, stance_fsm, walk_parameters) != humoto::ControlProblemStatus::OK)
                        {
                            break;
                        }

                        // form an optimization problem
                        opt_problem.form(solution, model, wpg);

                        // solve an optimization problem
                        solver.solve(solution, opt_problem);

                        // extract next model state from the solution and update model
                        stance_fsm.shiftTime(wpg_parameters.sampling_time_ms_);
                        model_state = wpg.getNextModelState(solution, stance_fsm, model);
                        model.updateState(model_state);
                    }

                    // return last solution
                    return( VerificationData(solution, model.state_) );
                }
        };


        class Fixture_FixedFootPositions
            :   public ::testing::Test,
                public TestBase
        {
            protected:
                Fixture_FixedFootPositions()
                {
                    reference_.solution_.readConfig<humoto::config::yaml::Reader>(g_ref_filename, "Solution_FixedFootPositions");
                    reference_.state_.readConfig<humoto::config::yaml::Reader>(g_ref_filename, "ModelState_FixedFootPositions");

                    walk_parameters.com_velocity_ << 0.1, 0.;
                    walk_parameters.first_stance_com_velocity_ = walk_parameters.com_velocity_;
                    walk_parameters.num_steps_ = 7;

                    stance_fsm.setParameters(walk_parameters);
                    wpg.setParameters(wpg_parameters);

                    opt_problem.readConfig<humoto::config::yaml::Reader>(g_config_path + "/hierarchies.yaml", "Hierarchy01");
                }
        };


        class Fixture_RotatingFeet
            :   public ::testing::Test,
                public TestBase
        {
            protected:
                Fixture_RotatingFeet()
                {
                    reference_.solution_.readConfig<humoto::config::yaml::Reader>(g_ref_filename, "Solution_RotatingFeet");
                    reference_.state_.readConfig<humoto::config::yaml::Reader>(g_ref_filename, "ModelState_RotatingFeet");

                    walk_parameters.com_velocity_ << 0.1, 0.;
                    walk_parameters.first_stance_com_velocity_ = walk_parameters.com_velocity_;
                    walk_parameters.num_steps_ = 7;
                    walk_parameters.theta_increment_ = 0.1;

                    stance_fsm.setParameters(walk_parameters);
                    wpg.setParameters(wpg_parameters);

                    opt_problem.readConfig<humoto::config::yaml::Reader>(g_config_path + "/hierarchies.yaml", "Hierarchy00");
                }
        };


        class Fixture_HRP4Parameters
            :   public ::testing::Test,
                public TestBase
        {
            protected:
                Fixture_HRP4Parameters()
                {
                    reference_.solution_.readConfig<humoto::config::yaml::Reader>(g_ref_filename, "Solution_HRP4Parameters");
                    reference_.state_.readConfig<humoto::config::yaml::Reader>(g_ref_filename, "ModelState_HRP4Parameters");

                    walk_parameters.readConfig<humoto::config::yaml::Reader>(g_config_path + "walk_parameters__walk_in_place_7_steps.yaml");
                    robot_parameters.readConfig<humoto::config::yaml::Reader>(g_config_path + "robot_hrp4.yaml");
                    model_state.readConfig<humoto::config::yaml::Reader>(g_config_path + "initial_state_hrp4.yaml");

                    solver.setParameters    (solver_parameters);
                    stance_fsm.setParameters(walk_parameters);
                    wpg.setParameters       (wpg_parameters);

                    opt_problem.readConfig<humoto::config::yaml::Reader>(g_config_path + "/hierarchies.yaml", "Hierarchy00");

                    model.setFootParameters(robot_parameters);
                    model.updateState      (model_state);
                }
        };


        class Fixture_SingleConfigurationFile
            :   public ::testing::Test,
                public TestBase
        {
            protected:
                Fixture_SingleConfigurationFile()
                {
                    reference_.solution_.readConfig<humoto::config::yaml::Reader>(g_ref_filename, "Solution_SingleConfigurationFile");
                    reference_.state_.readConfig<humoto::config::yaml::Reader>(g_ref_filename, "ModelState_SingleConfigurationFile");

                    humoto::config::yaml::Reader config_reader("regression_test_001.yaml");

                    walk_parameters.readConfig(config_reader);
                    wpg_parameters.readConfig(config_reader);
                    solver_parameters.readConfig(config_reader);

                    solver.setParameters(solver_parameters);
                    stance_fsm.setParameters(walk_parameters);
                    wpg.setParameters(wpg_parameters);

                    opt_problem.readConfig(config_reader);
                }
        };



        // Tests that the last MPC solution matches ref
        TEST_F(Fixture_FixedFootPositions, SolutionMatchReference)
        {
            HUMOTO_TEST_COMPARE_WITH_REFERENCE(run(), reference_, 1e-08);
        }


        // Tests that the last MPC solution matches ref
        TEST_F(Fixture_RotatingFeet, SolutionMatchReference)
        {
            HUMOTO_TEST_COMPARE_WITH_REFERENCE(run(), reference_, 1e-08);
        }


        // Tests that the last MPC solution matches ref
        TEST_F(Fixture_HRP4Parameters, SolutionMatchReference)
        {
            HUMOTO_TEST_COMPARE_WITH_REFERENCE(run(), reference_, 1e-08);
        }


        // Tests that the last MPC solution matches ref
        TEST_F(Fixture_SingleConfigurationFile, SolutionMatchReference)
        {
            HUMOTO_TEST_COMPARE_WITH_REFERENCE(run(), reference_, 1e-08);
        }
    }
}


HUMOTO_DEFINE_REGRESSION_TEST_MAIN()
