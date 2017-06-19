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

// common & abstract classes (must be first)
#include "humoto/humoto.h"
// specific solver (many can be included simultaneously)
#include "humoto/lexls.h"
#include "humoto/qpoases.h"
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
        class TestFixture : public ::testing::Test
        {
            protected:
                TestFixture()
                {
                    setupHierarchy_v5(opt_problem);
                }


                // optimization problem (a stack of tasks / hierarchy)
                humoto::OptimizationProblem               opt_problem;
                // a solver which is giong to be used
                humoto::qpoases::Solver                   qpoases_solver;
                humoto::lexls::Solver                   lexls_solver;


                // solution
                humoto::qpoases::Solution               qpoases_solution;
                humoto::lexls::Solution                 lexls_solution;


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


                humoto::ActiveSet qpoases_active_set_actual;
                humoto::ActiveSet qpoases_active_set_determined;
                humoto::Violations  qpoases_violations;

                humoto::ActiveSet lexls_active_set_actual;
                humoto::ActiveSet lexls_active_set_determined;
                humoto::Violations  lexls_violations;


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
                        if (wpg.update(model, stance_fsm, walk_parameters) != humoto::ControlProblemStatus::OK)
                        {
                            break;
                        }

                        // form an optimization problem
                        opt_problem.form(qpoases_solution, model, wpg);
                        wpg.initSolutionStructure(lexls_solution);

                        // solve an optimization problem
                        qpoases_solver.solve(qpoases_solution, qpoases_active_set_actual, opt_problem);
                        lexls_solver.solve(lexls_solution, lexls_active_set_actual, opt_problem);

                        opt_problem.determineActiveSet(qpoases_active_set_determined, qpoases_solution);
                        opt_problem.computeViolations(qpoases_violations, qpoases_solution);

                        opt_problem.determineActiveSet(lexls_active_set_determined, lexls_solution);
                        opt_problem.computeViolations(lexls_violations, lexls_solution);


                        //========================
                        for (std::size_t l = 0; l < qpoases_active_set_actual.size(); ++l)
                        {
                            for (std::size_t k = 0; k < qpoases_active_set_actual[l].size(); ++k)
                            {
                                if (qpoases_active_set_determined[l][k] != qpoases_active_set_actual[l][k])
                                {
                                    if (!(qpoases_active_set_determined[l][k] == humoto::ConstraintActivationType::EQUALITY)
                                        && (qpoases_active_set_actual[l][k] ==  humoto::ConstraintActivationType::LOWER_BOUND))
                                    {
                                        if ((qpoases_active_set_actual[l][k] == humoto::ConstraintActivationType::LOWER_BOUND)
                                            && (std::abs(qpoases_violations[l].getLower(k)) < 1e-12))
                                        {
                                            if ((qpoases_active_set_actual[l][k] == humoto::ConstraintActivationType::UPPER_BOUND)
                                                && (std::abs(qpoases_violations[l].getUpper(k)) < 1e-12))
                                            {
                                                ++mismatch_count;
                                            }
                                        }
                                    }
                                }

                                if (lexls_active_set_determined[l][k] != lexls_active_set_actual[l][k])
                                {
                                    ++mismatch_count;
                                }
                            }
                        }
                        //========================


                        // extract next model state from the solution and update model
                        stance_fsm.shiftTime(wpg_parameters.sampling_time_ms_);
                        model_state = wpg.getNextModelState(qpoases_solution, stance_fsm, model);
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


/**
 * @brief main
 *
 * @param[in] argc number of args
 * @param[in] argv args
 *
 * @return status
 */
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
