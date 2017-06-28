/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "solver_performance_test.h"

namespace humoto_tests
{
    namespace pepper_mpc
    {
        namespace HUMOTO_TEST_SOLVER_NAMESPACE
        {
            class ControllerSingleRunBase : public humoto_tests::SolverPerformanceSingleRunBase
            {
                protected:
                    // optimization problem (a stack of tasks / hierarchy)
                    humoto::OptimizationProblem                    opt_problem_;

                    // a solver which is giong to be used
                    humoto::HUMOTO_TEST_SOLVER_NAMESPACE::Solver   solver_;
                    // solution
                    humoto::HUMOTO_TEST_SOLVER_NAMESPACE::Solution solution_;

                    humoto::pepper_mpc::RobotParameters      robot_parameters_;
                    // parameters of the control problem
                    humoto::pepper_mpc::MPCParameters        mg_parameters_;
                    // control problem, which is used to construct an optimization problem
                    humoto::pepper_mpc::MPCforMG             mg_;
                    // model representing the controlled system
                    humoto::pepper_mpc::Model                model_;
                    // options for walking
                    humoto::pepper_mpc::MotionParameters     motion_parameters_;
                    humoto::pepper_mpc::ModelState           model_state_;


                public:
                    void initialize (const std::string & config_path)
                    {
                        robot_parameters_.readConfig (config_path + "robot_parameters.yaml");
                        mg_parameters_.readConfig    (config_path + "mpc_parameters.yaml");
                        motion_parameters_.readConfig(config_path + "motion_parameters_circle_fast.yaml");
                        model_state_.readConfig      (config_path + "initial_state_pepper.yaml");

                        model_.updateState(model_state_);
                        mg_.setParameters    (mg_parameters_);
                        HUMOTO_TEST_HIERARCHY_SETUP_FUNCTION(opt_problem_, motion_parameters_);
                    }
            };


            class SolverPerformanceNoHotstarting : public ControllerSingleRunBase
            {
                public:
                    void run(std::vector<Eigen::Vector3d> & measures)
                    {
                        Eigen::Vector3d     iter_measures;
                        std::size_t i = 0;
                        for(;; ++i)
                        {
                            timer_global_.start();
                            timer_form_problem_.start();
                            // prepare control problem for new iteration
                            if (mg_.updateAndShift( motion_parameters_,
                                                    model_,
                                                    mg_parameters_.subsampling_time_ms_) != humoto::ControlProblemStatus::OK)
                            {
                                break;
                            }

                            // form an optimization problem
                            opt_problem_.form(solution_, model_, mg_);
                            iter_measures(0) = timer_form_problem_.stop();

                            timer_solve_problem_.start();
                            // solve an optimization problem
                            solver_.solve(solution_, opt_problem_);
                            iter_measures(1) = timer_solve_problem_.stop();

                            // extract next model state from the solution and update model
                            //model_state_ = mg_.getNextModelState(solution_, model_);
                            mg_.parseSolution(solution_);
                            model_state_ = mg_.getModelState(model_, mg_parameters_.subsampling_time_ms_);
                            model_.updateState(model_state_);
                            iter_measures(2) = timer_global_.stop();

                            measures.push_back(iter_measures);
                        }
                    }
            };


#ifndef HUMOTO_TEST_DISABLE_HOT_STARTING
            class SolverPerformanceActiveSetGuess : public ControllerSingleRunBase
            {
                public:
                    void run(std::vector<Eigen::Vector3d> & measures)
                    {
                        humoto::ActiveSet active_set_guess;
                        humoto::ActiveSet active_set_actual;

                        Eigen::Vector3d iter_measures;
                        std::size_t i = 0;
                        for(;; ++i)
                        {
                            timer_global_.start();
                            timer_form_problem_.start();
                            // prepare control problem for new iteration
                            if (mg_.updateAndShift( motion_parameters_,
                                                    model_,
                                                    mg_parameters_.subsampling_time_ms_) != humoto::ControlProblemStatus::OK)
                            {
                                break;
                            }

                            // form an optimization problem
                            opt_problem_.form(solution_, active_set_guess, model_, mg_);
                            iter_measures(0) = timer_form_problem_.stop();

                            timer_solve_problem_.start();
                            // solve an optimization problem
                            solver_.solve(solution_, active_set_actual, opt_problem_, active_set_guess);
                            iter_measures(1) = timer_solve_problem_.stop();

                            opt_problem_.processActiveSet(active_set_actual);

                            // extract next model state from the solution and update model
                            //model_state_ = mg_.getNextModelState(solution_, model_);
                            mg_.parseSolution(solution_);
                            model_state_ = mg_.getModelState(model_, mg_parameters_.subsampling_time_ms_);
                            model_.updateState(model_state_);
                            iter_measures(2) = timer_global_.stop();

                            measures.push_back(iter_measures);
                        }
                    }
            };


            class SolverPerformanceActiveSetAndSolutionGuess : public ControllerSingleRunBase
            {
                public:
                    void run(std::vector<Eigen::Vector3d> & measures)
                    {
                        humoto::ActiveSet active_set_guess;
                        humoto::ActiveSet active_set_actual;

                        humoto::HUMOTO_TEST_SOLVER_NAMESPACE::Solution    old_solution;
                        humoto::Solution            solution_guess;

                        Eigen::Vector3d iter_measures;
                        std::size_t i = 0;
                        for(;; ++i)
                        {
                            timer_global_.start();
                            // prepare control problem for new iteration
                            timer_form_problem_.start();
                            if (mg_.updateAndShift( motion_parameters_,
                                                    model_,
                                                    mg_parameters_.subsampling_time_ms_) != humoto::ControlProblemStatus::OK)
                            {
                                break;
                            }

                            // form an optimization problem
                            opt_problem_.form(solution_, solution_guess, active_set_guess, model_, mg_, old_solution);
                            iter_measures(0) = timer_form_problem_.stop();

                            timer_solve_problem_.start();
                            // solve an optimization problem
                            solver_.solve(solution_, active_set_actual, opt_problem_, solution_guess, active_set_guess);
                            iter_measures(1) = timer_solve_problem_.stop();

                            opt_problem_.processActiveSet(active_set_actual);
                            old_solution = solution_;

                            // extract next model state from the solution and update model
                            //model_state_ = mg_.getNextModelState(solution_, model_);
                            mg_.parseSolution(solution_);
                            model_state_ = mg_.getModelState(model_, mg_parameters_.subsampling_time_ms_);
                            model_.updateState(model_state_);
                            iter_measures(2) = timer_global_.stop();

                            measures.push_back(iter_measures);
                        }
                    }
            };


            class SolverPerformanceSolutionGuess : public ControllerSingleRunBase
            {
                public:
                    void run(std::vector<Eigen::Vector3d> & measures)
                    {
                        humoto::HUMOTO_TEST_SOLVER_NAMESPACE::Solution    old_solution;
                        humoto::Solution            solution_guess;

                        Eigen::Vector3d iter_measures;
                        std::size_t i = 0;
                        for(;; ++i)
                        {
                            timer_global_.start();
                            // prepare control problem for new iteration
                            timer_form_problem_.start();
                            if (mg_.updateAndShift( motion_parameters_,
                                                    model_,
                                                    mg_parameters_.subsampling_time_ms_) != humoto::ControlProblemStatus::OK)
                            {
                                break;
                            }

                            // form an optimization problem
                            opt_problem_.form(solution_, solution_guess, model_, mg_, old_solution);
                            iter_measures(0) = timer_form_problem_.stop();

                            timer_solve_problem_.start();
                            // solve an optimization problem
                            solver_.solve(solution_, opt_problem_, solution_guess);
                            iter_measures(1) = timer_solve_problem_.stop();

                            old_solution = solution_;

                            // extract next model state from the solution and update model
                            //model_state_ = mg_.getNextModelState(solution_, model_);
                            mg_.parseSolution(solution_);
                            model_state_ = mg_.getModelState(model_, mg_parameters_.subsampling_time_ms_);
                            model_.updateState(model_state_);
                            iter_measures(2) = timer_global_.stop();

                            measures.push_back(iter_measures);
                        }
                    }
            };
#endif


            void run(   const std::size_t n_of_simulations,
                        const std::string &config_path)
            {
                humoto::Logger          logger(HUMOTO_STRING_NAME_GENERATOR("time_perf_", HUMOTO_TEST_SOLVER_NAMESPACE, ".m"));
                humoto::LogEntryName    prefix(HUMOTO_STRING_NAME_GENERATOR("", HUMOTO_TEST_SOLVER_NAMESPACE, "_simulation"));


                SolverPerformanceMultiRun<SolverPerformanceNoHotstarting>::run(
                        logger, n_of_simulations, config_path, prefix, "no_hotstart");

#ifndef HUMOTO_TEST_DISABLE_HOT_STARTING
                SolverPerformanceMultiRun<SolverPerformanceActiveSetGuess>::run(
                        logger, n_of_simulations, config_path, prefix, "as_guess_prev_iter");

                SolverPerformanceMultiRun<SolverPerformanceActiveSetAndSolutionGuess>::run(
                        logger, n_of_simulations, config_path, prefix, "sol_guess_as_guess_prev_iter");

                SolverPerformanceMultiRun<SolverPerformanceSolutionGuess>::run(
                        logger, n_of_simulations, config_path, prefix, "sol_guess_no_as_guess");
#endif
            }
        }
    }
}

#undef HUMOTO_TEST_SOLVER_NAMESPACE
#undef HUMOTO_TEST_DISABLE_HOT_STARTING
