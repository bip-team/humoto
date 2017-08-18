/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#define HUMOTO_TEST_FIXTURE_NAME    HUMOTO_NAME_GENERATOR(TestFixture_, HUMOTO_TEST_SOLVER_NAMESPACE)

namespace humoto_tests
{
    namespace wpg04
    {
        class HUMOTO_TEST_FIXTURE_NAME : public ::testing::Test
        {
            protected:
                VerificationData        reference_;


            protected:
                // optimization problem (a stack of tasks / hierarchy)
                humoto::wpg04::ConfigurableOptimizationProblem  opt_problem;
                // a solver which is giong to be used
                humoto::HUMOTO_TEST_SOLVER_NAMESPACE::Solver    solver;
                // solution
                humoto::HUMOTO_TEST_SOLVER_NAMESPACE::Solution  solution;
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


            protected:
                HUMOTO_TEST_FIXTURE_NAME()
                {
                    reference_.solution_.readConfig<humoto::config::yaml::Reader>(g_ref_filename);
                    reference_.state_.readConfig<humoto::config::yaml::Reader>(g_ref_filename);

                    walk_parameters.com_velocity_ << 0.1, 0.;
                    walk_parameters.first_stance_com_velocity_ = walk_parameters.com_velocity_;
                    walk_parameters.num_steps_ = 7;

                    stance_fsm.setParameters(walk_parameters);
                    wpg.setParameters(wpg_parameters);

                    opt_problem.readConfig<humoto::config::yaml::Reader>(g_config_path + "/hierarchies.yaml", HUMOTO_TEST_HIERARCHY_ID);
                }


                /**
                 * @brief Simple control loop
                 *
                 * @return Last solution vector
                */
                VerificationData run1()
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


#ifndef HUMOTO_TEST_DISABLE_HOT_STARTING
                /**
                 * @brief Simple control loop
                 *
                 * @return Last solution vector
                */
                VerificationData run2()
                {
                    humoto::ActiveSet active_set_guess;
                    humoto::ActiveSet active_set_actual;

                    for (unsigned int i = 0;; ++i)
                    {
                        // prepare control problem for new iteration
                        if (wpg.update(model, stance_fsm, walk_parameters) != humoto::ControlProblemStatus::OK)
                        {
                            break;
                        }

                        // form an optimization problem
                        opt_problem.form(solution, active_set_guess, model, wpg);

                        // solve an optimization problem
                        solver.solve(solution, active_set_actual, opt_problem, active_set_guess);
                        opt_problem.processActiveSet(active_set_actual);

                        // extract next model state from the solution and update model
                        stance_fsm.shiftTime(wpg_parameters.sampling_time_ms_);
                        model_state = wpg.getNextModelState(solution, stance_fsm, model);
                        model.updateState(model_state);
                    }


                    // return last solution
                    return(VerificationData(solution, model.state_) );
                }


                /**
                 * @brief Simple control loop
                 *
                 * @return Last solution vector
                */
                VerificationData run3()
                {
                    humoto::ActiveSet active_set_guess;

                    for (unsigned int i = 0;; ++i)
                    {
                        // prepare control problem for new iteration
                        if (wpg.update(model, stance_fsm, walk_parameters) != humoto::ControlProblemStatus::OK)
                        {
                            break;
                        }

                        // form an optimization problem
                        opt_problem.form(solution, active_set_guess, model, wpg);

                        // solve an optimization problem
                        solver.solve(solution, opt_problem, active_set_guess);

                        // extract next model state from the solution and update model
                        stance_fsm.shiftTime(wpg_parameters.sampling_time_ms_);
                        model_state = wpg.getNextModelState(solution, stance_fsm, model);
                        model.updateState(model_state);
                    }


                    // return last solution
                    return(VerificationData(solution, model.state_) );
                }


                /**
                 * @brief Simple control loop
                 *
                 * @return Last solution vector
                */
                VerificationData run4()
                {
                    humoto::ActiveSet active_set_guess;
                    humoto::ActiveSet active_set_actual;

                    humoto::HUMOTO_TEST_SOLVER_NAMESPACE::Solution  old_solution;
                    humoto::Solution                                solution_guess;

                    for (unsigned int i = 0;; ++i)
                    {
                        // prepare control problem for new iteration
                        if (wpg.update(model, stance_fsm, walk_parameters) != humoto::ControlProblemStatus::OK)
                        {
                            break;
                        }

                        // form an optimization problem
                        opt_problem.form(solution, solution_guess, active_set_guess, model, wpg, old_solution);

                        // solve an optimization problem
                        solver.solve(solution, active_set_actual, opt_problem, solution_guess, active_set_guess);
                        opt_problem.processActiveSet(active_set_actual);

                        old_solution = solution;

                        // extract next model state from the solution and update model
                        stance_fsm.shiftTime(wpg_parameters.sampling_time_ms_);
                        model_state = wpg.getNextModelState(solution, stance_fsm, model);
                        model.updateState(model_state);
                    }


                    // return last solution
                    return(VerificationData(solution, model.state_) );
                }


                /**
                 * @brief Simple control loop
                 *
                 * @return Last solution vector
                */
                VerificationData run5()
                {
                    humoto::ActiveSet       active_set_guess;

                    humoto::HUMOTO_TEST_SOLVER_NAMESPACE::Solution  old_solution;
                    humoto::Solution                                solution_guess;

                    for (unsigned int i = 0;; ++i)
                    {
                        // prepare control problem for new iteration
                        if (wpg.update(model, stance_fsm, walk_parameters) != humoto::ControlProblemStatus::OK)
                        {
                            break;
                        }

                        // form an optimization problem
                        opt_problem.form(solution, solution_guess, active_set_guess, model, wpg, old_solution);

                        // solve an optimization problem
                        solver.solve(solution, opt_problem, solution_guess, active_set_guess);

                        old_solution = solution;

                        // extract next model state from the solution and update model
                        stance_fsm.shiftTime(wpg_parameters.sampling_time_ms_);
                        model_state = wpg.getNextModelState(solution, stance_fsm, model);
                        model.updateState(model_state);
                    }


                    // return last solution
                    return(VerificationData(solution, model.state_) );
                }


                /**
                 * @brief Simple control loop
                 *
                 * @return Last solution vector
                */
                VerificationData run6()
                {
                    humoto::HUMOTO_TEST_SOLVER_NAMESPACE::Solution  old_solution;
                    humoto::Solution                                solution_guess;

                    for (unsigned int i = 0;; ++i)
                    {
                        // prepare control problem for new iteration
                        if (wpg.update(model, stance_fsm, walk_parameters) != humoto::ControlProblemStatus::OK)
                        {
                            break;
                        }

                        // form an optimization problem
                        opt_problem.form(solution, solution_guess, model, wpg, old_solution);

                        // solve an optimization problem
                        solver.solve(solution, opt_problem, solution_guess);

                        old_solution = solution;

                        // extract next model state from the solution and update model
                        stance_fsm.shiftTime(wpg_parameters.sampling_time_ms_);
                        model_state = wpg.getNextModelState(solution, stance_fsm, model);
                        model.updateState(model_state);
                    }


                    // return last solution
                    return(VerificationData(solution, model.state_) );
                }
#endif
        };


        // Tests that the last MPC solution matches ref
        // Don't guess active set, don't hotstart the solution
        TEST_F(HUMOTO_TEST_FIXTURE_NAME, SolutionMatchReferenceNoSolutionNoASGuess)
        {
            HUMOTO_TEST_COMPARE_WITH_REFERENCE(run1(), reference_, 1e-08);
        }


#ifndef HUMOTO_TEST_DISABLE_HOT_STARTING
        // Tests that the last MPC solution matches ref
        // Guess active set based on the active set from the previous iteration, don't hotstart
        TEST_F(HUMOTO_TEST_FIXTURE_NAME, SolutionMatchReferenceNoSolutionASGuessPrevIter)
        {
            HUMOTO_TEST_COMPARE_WITH_REFERENCE(run2(), reference_, 1e-08);
        }

        // Tests that the last MPC solution matches ref
        // Guess active set based on heuristic (without information from previous iteration), don't hotstart
        TEST_F(HUMOTO_TEST_FIXTURE_NAME, SolutionMatchReferenceNoSolutionASGuessHeuristic)
        {
            HUMOTO_TEST_COMPARE_WITH_REFERENCE(run3(), reference_, 1e-08);
        }

        // Tests that the last MPC solution matches ref
        // Use solution from previous iteration to hotstart + guess active set based on previous iteration
        TEST_F(HUMOTO_TEST_FIXTURE_NAME, SolutionMatchReferenceSolutionGuessASGuessPrevIter)
        {
            HUMOTO_TEST_COMPARE_WITH_REFERENCE(run4(), reference_, 1e-08);
        }

        // Tests that the last MPC solution matches ref
        // Use solution from previous iteration to hotstart + guess active set based on heuristic
        TEST_F(HUMOTO_TEST_FIXTURE_NAME, SolutionMatchReferenceSolutionGuessASGuessHeuristic)
        {
            HUMOTO_TEST_COMPARE_WITH_REFERENCE(run5(), reference_, 1e-08);
        }

        // Tests that the last MPC solution matches ref
        // Use solution from previous iteration to hotstart and don't guess active set
        TEST_F(HUMOTO_TEST_FIXTURE_NAME, SolutionMatchReferenceSolutionGuessNoASGuess)
        {
            HUMOTO_TEST_COMPARE_WITH_REFERENCE(run6(), reference_, 1e-08);
        }
#endif
    }
}

#undef HUMOTO_TEST_FIXTURE_NAME
#undef HUMOTO_TEST_SOLVER_NAMESPACE
#undef HUMOTO_TEST_DISABLE_HOT_STARTING
