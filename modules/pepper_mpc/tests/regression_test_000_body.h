/**
    @file
    @author  Jan Michalczyk
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#define HUMOTO_TEST_FIXTURE_NAME    HUMOTO_NAME_GENERATOR(TestFixture_, HUMOTO_TEST_SOLVER_NAMESPACE)

namespace humoto_tests
{
    namespace pepper_mpc
    {
        class HUMOTO_TEST_FIXTURE_NAME : public ::testing::Test
        {
            protected:
                humoto::Solution ref_solution;


            protected:
                HUMOTO_TEST_FIXTURE_NAME()
                {
                    ref_solution.readConfig<humoto::config::yaml::Reader>(g_ref_filename);

                    robot_parameters.readConfig<humoto::config::yaml::Reader>(g_config_path + "robot_parameters.yaml");
                    mg_parameters.readConfig<humoto::config::yaml::Reader>(g_config_path + "mpc_parameters.yaml");
                    motion_parameters.readConfig<humoto::config::yaml::Reader>(g_config_path + "motion_parameters_circle.yaml");
                    model_state.readConfig<humoto::config::yaml::Reader>(g_config_path + "initial_state_pepper.yaml");
                    model.updateState(model_state);


                    switch (motion_parameters.motion_mode_)
                    {
                        case humoto::pepper_mpc::MotionMode::MAINTAIN_POSITION:
                            opt_problem.readConfig<humoto::config::yaml::Reader>(g_config_path + "hierarchies.yaml", true, "Hierarchy01");
                            break;
                        case humoto::pepper_mpc::MotionMode::MAINTAIN_VELOCITY:
                            opt_problem.readConfig<humoto::config::yaml::Reader>(g_config_path + "hierarchies.yaml", true, "Hierarchy00");
                            break;
                        default:
                            HUMOTO_THROW_MSG("Unsupported motion mode.");
                    }
                }


                // optimization problem (a stack of tasks / hierarchy)
                humoto::pepper_mpc::ConfigurableOptimizationProblem     opt_problem;

                // parameters of the solver
                humoto::HUMOTO_TEST_SOLVER_NAMESPACE::SolverParameters   solver_parameters;
                // a solver which is giong to be used
                humoto::HUMOTO_TEST_SOLVER_NAMESPACE::Solver             solver;
                // solution
                humoto::HUMOTO_TEST_SOLVER_NAMESPACE::Solution           solution;

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


                /**
                 * @brief Simple control loop
                 *
                 * @return status
                */
                Eigen::VectorXd run()
                {
                    mg.setParameters(mg_parameters);
                    model.setParameters(robot_parameters);


                    for (std::size_t i = 0;; ++i)
                    {
                        // prepare control problem for new iteration
                        if (mg.updateAndShift(  motion_parameters,
                                                model,
                                                mg_parameters.sampling_time_ms_) != humoto::ControlProblemStatus::OK)
                        {
                            break;
                        }

                        // form an optimization problem
                        opt_problem.form(solution, model, mg);
                        // solve an optimization problem
                        solver.solve(solution, opt_problem);

                        // extract next model state from the solution and update model
                        model_state = mg.getNextModelState(solution, model);
                        model.updateState(model_state);
                    }

                    // return last solution
                    return solution.get_x();
                }
        };


        // Tests that the last MPC solution matches ref
        TEST_F(HUMOTO_TEST_FIXTURE_NAME, SolutionMatchReference)
        {
            ASSERT_TRUE(ref_solution.x_.isApprox(run(), 1e-8));
        }
    }
}


#undef HUMOTO_TEST_FIXTURE_NAME
#undef HUMOTO_TEST_SOLVER_NAMESPACE
