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
#include "humoto/lexls.h"
// specific control problem (many can be included simultaneously)
#include "humoto/wpg04.h"

//testing
#include "gtest/gtest.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);

#include "utilities_regression.h"

namespace humoto_tests
{
    namespace wpg04
    {
        class TestFixture_2levels : public ::testing::Test
        {
            protected:
                VerificationData        reference_;


            protected:
                TestFixture_2levels()
                {
                    reference_.solution_.readConfig<humoto::config::yaml::Reader>(g_ref_filename);
                    reference_.state_.readConfig<humoto::config::yaml::Reader>(g_ref_filename);

                    walk_parameters.com_velocity_ << 0.1, 0.;
                    walk_parameters.first_stance_com_velocity_ = walk_parameters.com_velocity_;
                    walk_parameters.num_steps_ = 7;

                    stance_fsm.setParameters(walk_parameters);
                    wpg.setParameters(wpg_parameters);

                    opt_problem.readConfig<humoto::config::yaml::Reader>(g_config_path + "/hierarchies.yaml", "Hierarchy02");
                }


                // optimization problem (a stack of tasks / hierarchy)
                humoto::wpg04::ConfigurableOptimizationProblem               opt_problem;
                // a solver which is giong to be used
                humoto::lexls::Solver                     solver;
                // solution
                humoto::lexls::Solution                   solution;
                // options for walking
                humoto::wpg04::WalkParameters             walk_parameters;
                //FSM for walking
                humoto::walking::StanceFiniteStateMachine stance_fsm;
                // model representing the controlled system
                humoto::wpg04::Model                      model;
                // parameters of the control problem
                humoto::wpg04::MPCParameters              wpg_parameters;
                // control problem, which is used to construct an optimization problem
                humoto::wpg04::MPCforWPG                  wpg;
                humoto::wpg04::ModelState                 model_state;


                /**
                 * @brief Simple control loop
                 *
                 * @return Last solution vector
                */
                humoto_tests::wpg04::VerificationData run()
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
                    return(VerificationData(solution, model.state_) );
                }
        };


        class TestFixture_3levels : public ::testing::Test
        {
            protected:
                VerificationData        reference_;


            protected:
                TestFixture_3levels()
                {
                    reference_.solution_.readConfig<humoto::config::yaml::Reader>(g_ref_filename);
                    reference_.state_.readConfig<humoto::config::yaml::Reader>(g_ref_filename);

                    walk_parameters.com_velocity_ << 0.1, 0.;
                    walk_parameters.first_stance_com_velocity_ = walk_parameters.com_velocity_;
                    walk_parameters.num_steps_ = 7;

                    stance_fsm.setParameters(walk_parameters);
                    wpg.setParameters(wpg_parameters);

                    opt_problem.readConfig<humoto::config::yaml::Reader>(g_config_path + "/hierarchies.yaml", "Hierarchy03");
                }


                // optimization problem (a stack of tasks / hierarchy)
                humoto::wpg04::ConfigurableOptimizationProblem               opt_problem;
                // a solver which is giong to be used
                humoto::lexls::Solver                     solver;
                // solution
                humoto::lexls::Solution                   solution;
                // options for walking
                humoto::wpg04::WalkParameters             walk_parameters;
                //FSM for walking
                humoto::walking::StanceFiniteStateMachine stance_fsm;
                // model representing the controlled system
                humoto::wpg04::Model                      model;
                // parameters of the control problem
                humoto::wpg04::MPCParameters              wpg_parameters;
                // control problem, which is used to construct an optimization problem
                humoto::wpg04::MPCforWPG                  wpg;
                humoto::wpg04::ModelState                 model_state;


                /**
                 * @brief Simple control loop
                 *
                 * @return Last solution vector
                */
                humoto_tests::wpg04::VerificationData run()
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
                    return(VerificationData(solution, model.state_) );
                }
        };


        // Tests that the last MPC solution matches ref
        TEST_F(TestFixture_2levels, SolutionMatchReferenceNoSolutionNoASGuess)
        {
            HUMOTO_TEST_COMPARE_WITH_REFERENCE(run(), reference_, 1e-08);
        }


        // Tests that the last MPC solution matches ref
        TEST_F(TestFixture_3levels, SolutionMatchReferenceNoSolutionNoASGuess)
        {
            HUMOTO_TEST_COMPARE_WITH_REFERENCE(run(), reference_, 1e-08);
        }
    }
}


HUMOTO_DEFINE_REGRESSION_TEST_MAIN()
