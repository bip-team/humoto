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
#ifdef HUMOTO_BRIDGE_lexls
#include "humoto/lexls.h"
#endif
#include "humoto/qpoases.h"
// specific control problem (many can be included simultaneously)
#include "humoto/pepper_mpc.h"

//testing
#include "gtest/gtest.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);

#include "utilities_regression.h"


namespace humoto_tests
{
    namespace pepper_mpc
    {
        class TestFixture : public ::testing::Test
        {
            protected:
                TestFixture()
                {
                    robot_parameters.readConfig<humoto::config::yaml::Reader>(g_config_path + "robot_parameters.yaml");
                    mg_parameters.readConfig<humoto::config::yaml::Reader>(g_config_path + "mpc_parameters.yaml");
                    motion_parameters.readConfig<humoto::config::yaml::Reader>(g_config_path + "motion_parameters_circle_fast.yaml");
                    model_state.readConfig<humoto::config::yaml::Reader>(g_config_path + "initial_state_pepper.yaml");
                    model.updateState(model_state);


                    switch (motion_parameters.motion_mode_)
                    {
                        case humoto::pepper_mpc::MotionMode::MAINTAIN_POSITION:
                            opt_problem.readConfig<humoto::config::yaml::Reader>(g_config_path + "hierarchies.yaml", "Hierarchy01");
                            break;
                        case humoto::pepper_mpc::MotionMode::MAINTAIN_VELOCITY:
                            opt_problem.readConfig<humoto::config::yaml::Reader>(g_config_path + "hierarchies.yaml", "Hierarchy00");
                            break;
                        default:
                            HUMOTO_THROW_MSG("Unsupported motion mode.");
                    }
                }


                // optimization problem (a stack of tasks / hierarchy)
                humoto::pepper_mpc::ConfigurableOptimizationProblem     opt_problem;
                // a solver which is giong to be used
                humoto::qpoases::Solver                   qpoases_solver;
#ifdef HUMOTO_BRIDGE_lexls
                humoto::lexls::Solver                   lexls_solver;
#endif

                // solution
                humoto::qpoases::Solution               qpoases_solution;
#ifdef HUMOTO_BRIDGE_lexls
                humoto::lexls::Solution                 lexls_solution;
#endif


                // parameters of the control problem
                humoto::pepper_mpc::MPCParameters           mg_parameters;
                // control problem, which is used to construct an optimization problem
                humoto::pepper_mpc::MPCforMG                mg;
                // model representing the controlled system
                humoto::pepper_mpc::RobotParameters         robot_parameters;
                humoto::pepper_mpc::Model                   model;

                // options for walking
                humoto::pepper_mpc::MotionParameters        motion_parameters;

                humoto::pepper_mpc::ModelState              model_state;


                humoto::ActiveSet qpoases_active_set_actual;
                humoto::ActiveSet qpoases_active_set_determined;
                humoto::Violations  qpoases_violations;

#ifdef HUMOTO_BRIDGE_lexls
                humoto::ActiveSet lexls_active_set_actual;
                humoto::ActiveSet lexls_active_set_determined;
                humoto::Violations  lexls_violations;
#endif


                /**
                 * @brief Simple control loop
                 *
                 * @return Last solution vector
                */
                std::size_t run()
                {
                    std::size_t mismatch_count = 0;

                    for (unsigned int i = 0;; ++i)
                    {
                        // prepare control problem for new iteration
                        if (mg.updateAndShift(  motion_parameters,
                                                model,
                                                mg_parameters.sampling_time_ms_) != humoto::ControlProblemStatus::OK)
                        {
                            break;
                        }

                        // form an optimization problem
                        opt_problem.form(qpoases_solution, model, mg);
#ifdef HUMOTO_BRIDGE_lexls
                        mg.initSolutionStructure(lexls_solution);
#endif

                        // solve an optimization problem
                        qpoases_solver.solve(qpoases_solution, qpoases_active_set_actual, opt_problem);
#ifdef HUMOTO_BRIDGE_lexls
                        lexls_solver.solve(lexls_solution, lexls_active_set_actual, opt_problem);
#endif

                        opt_problem.determineActiveSet(qpoases_active_set_determined, qpoases_solution);
                        opt_problem.computeViolations(qpoases_violations, qpoases_solution);

#ifdef HUMOTO_BRIDGE_lexls
                        opt_problem.determineActiveSet(lexls_active_set_determined, lexls_solution);
                        opt_problem.computeViolations(lexls_violations, lexls_solution);
#endif


                        //========================
                        for (std::size_t l = 0; l < qpoases_active_set_actual.size(); ++l)
                        {
                            for (std::size_t k = 0; k < qpoases_active_set_actual[l].size(); ++k)
                            {
                                if (qpoases_active_set_determined[l][k] != qpoases_active_set_actual[l][k])
                                {
                                    if (
                                        ! ( (qpoases_active_set_determined[l][k] == humoto::ConstraintActivationType::EQUALITY)
                                        && (qpoases_active_set_actual[l][k] ==  humoto::ConstraintActivationType::LOWER_BOUND) )
                                        )
                                    {
                                        switch (qpoases_active_set_actual[l][k])
                                        {
                                            case humoto::ConstraintActivationType::LOWER_BOUND:
                                                if (std::abs(qpoases_violations[l].getLower(k)) < 1e-10)
                                                {
                                                    break;
                                                }
                                            case humoto::ConstraintActivationType::UPPER_BOUND:
                                                if (std::abs(qpoases_violations[l].getUpper(k)) < 1e-10)
                                                {
                                                    break;
                                                }
                                            default:
                                                ++mismatch_count;
                                                break;
                                        }
                                    }
                                }

#ifdef HUMOTO_BRIDGE_lexls
                                if (lexls_active_set_determined[l][k] != lexls_active_set_actual[l][k])
                                {
                                    ++mismatch_count;
                                }
#endif
                            }
                        }
                        //========================


                        // extract next model state from the solution and update model
                        model_state = mg.getNextModelState(qpoases_solution, model);
                        model.updateState(model_state);
                    }

                    // return last solution
                    return (mismatch_count);
                }
        };


        // Tests that the last MPC solution matches ref
        TEST_F(TestFixture, CountActiveSetMismatch)
        {
            ASSERT_TRUE(run() == 0);
        }
    }
}


HUMOTO_DEFINE_REGRESSION_TEST_MAIN()
