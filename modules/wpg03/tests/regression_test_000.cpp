/**
    @file
    @author  Jan Michalczyk
    @author  Don Joven Agravante
    @copyright 2014-2017 INRIA, 2014-2015 CNRS. Licensed under the Apache
    License, Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

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
#include "humoto/wpg03.h"

//testing
#include "gtest/gtest.h"

#include "utilities_regression.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);


namespace humoto_tests
{
    namespace wpg03
    {
        class TestFixture : public ::testing::Test
        {
            protected:
                humoto::Solution ref_solution;


            protected:
                TestFixture()
                {
                    ref_solution.readConfig<humoto::config::yaml::Reader>(g_ref_filename);
                }


                /**
                 * @brief Setup hierarchy.
                 *
                 * @param[in] opt_problem       hierarchy
                 * @param[in] mpc_type
                 */
                void setupHierarchy(humoto::OptimizationProblem &opt_problem,
                                    const humoto::wpg03::MPCInteractionType mpc_type)
                {
                    // generic objectives
                    humoto::TaskSharedPointer    task_cop_position    (new humoto::wpg03::TaskCoPPosition   );
                    humoto::TaskSharedPointer    task_com_jerk        (new humoto::wpg03::TaskCoMJerk       );

                    // generic constraints
                    humoto::TaskSharedPointer    task_footstep_bounds (new humoto::wpg03::TaskFootstepBounds);
                    humoto::TaskSharedPointer    task_cop_bounds      (new humoto::wpg03::TaskCoPBounds     );

                    // reset the optimization problem
                    opt_problem.reset(2);

                    // push tasks into the stack/hierarchy
                    opt_problem.pushTask(task_footstep_bounds, 0);
                    opt_problem.pushTask(task_cop_bounds, 0);
                    opt_problem.pushTask(task_com_jerk, 1);
                    opt_problem.pushTask(task_cop_position, 1);

                    // interaction specific tasks
                    switch(mpc_type)
                    {
                        case humoto::wpg03::MPC_NORMAL:
                        {
                            humoto::TaskSharedPointer  task_com_velocity(new humoto::wpg03::TaskCoMVelocity);
                            opt_problem.pushTask(task_com_velocity, 1);
                            break;
                        }
                        case humoto::wpg03::MPC_FOLLOWER:
                        {
                            humoto::TaskSharedPointer  task_com_impedance(new humoto::wpg03::TaskCoMImpedance);
                            opt_problem.pushTask(task_com_impedance, 1);
                            break;
                        }
                        case humoto::wpg03::MPC_LEADER:
                        {
                            humoto::TaskSharedPointer  task_com_trajectory(new humoto::wpg03::TaskCoMTrajectory);
                            opt_problem.pushTask(task_com_trajectory, 1);

                            humoto::TaskSharedPointer  task_ext_wrench(new humoto::wpg03::TaskExtWrench);
                            opt_problem.pushTask(task_ext_wrench, 1);

                            humoto::TaskSharedPointer  task_ext_wrench_bounds(new humoto::wpg03::TaskExtWrenchBounds);
                            opt_problem.pushTask(task_ext_wrench_bounds, 0);

                            break;
                        }
                        default:
                        {
                            HUMOTO_THROW_MSG("unexpected MPC type");
                            break;
                        }
                    }
                }

                Eigen::VectorXd run()
                {
                    // optimization problem (a stack of tasks / hierarchy)
                    humoto::OptimizationProblem     opt_problem;

                    // parameters of the solver
                    humoto::qpoases::SolverParameters   solver_parameters;
                    // a solver which is giong to be used
                    humoto::qpoases::Solver             solver (solver_parameters);
                    // solution
                    humoto::qpoases::Solution           solution;

                    // parameters of the control problem
                    humoto::wpg03::MPCParameters            wpg_parameters;
                    // control problem, which is used to construct an optimization problem
                    humoto::wpg03::MPCforWPG                wpg(wpg_parameters);
                    // model representing the controlled system
                    humoto::wpg03::RobotParameters          robot_parameters;
                    humoto::wpg03::Model                    model(robot_parameters);

                    // options for walking
                    humoto::wpg03::WalkOptions              walk_options;
                    walk_options.com_velocity_ << 0.1, 0.;
                    walk_options.first_ds_com_velocity_ = walk_options.com_velocity_;
                    walk_options.num_steps_ = 10;

                    // Finite state machine for walking (determines sequence and durations of
                    // steps, as well as orientations of footsteps)
                    humoto::wpg03::WalkFiniteStateMachine   walk_fsm(model, wpg_parameters, walk_options);

                    // Selector for interaction type
                    humoto::wpg03::MPCInteractionType mpc_type;
                    //    mpc_type = humoto::wpg03::MPC_NORMAL;
                    mpc_type = humoto::wpg03::MPC_FOLLOWER;
                    //    mpc_type = humoto::wpg03::MPC_LEADER;

                    switch(mpc_type)
                    {
                        case humoto::wpg03::MPC_NORMAL:
                        {
                            std::cout << "using MPC_NORMAL" << std::endl;
                            break;
                        }
                        case humoto::wpg03::MPC_FOLLOWER:
                        {
                            std::cout << "using MPC_FOLLOWER" << std::endl;
                            break;
                        }
                        case humoto::wpg03::MPC_LEADER:
                        {
                            std::cout << "using MPC_LEADER" << std::endl;
                            break;
                        }
                        default:
                        {
                            HUMOTO_THROW_MSG("unexpected MPC type");
                            break;
                        }
                    }

                    setupHierarchy(opt_problem, mpc_type);

                    humoto::wpg03::ModelState   model_state;

                    // The current measured external forces
                    Eigen::VectorXd f_0;
                    f_0.resize(6);
                    f_0.setZero();

                    // Simple selector for prediction models
                    //bool isForceConstant = true;
                    bool isForceConstant = false; // for linear approximation

                    // Vector of the predicted external forces
                    Eigen::VectorXd f_pred;

                    humoto::Timer       timer;

                    for (unsigned int i = 0; /*i < 4*/; ++i)
                    {
                        timer.start();

                        // predict the forces
                        if(isForceConstant)
                        {
                            f_pred = humoto::wpg03::predictWrenchConstant(f_0, wpg.mpc_parameters_.preview_horizon_length_);
                        }
                        else
                        {
                            Eigen::VectorXd slope(6);
                            slope << 0., 0., 0.,  // Newton/sec
                                     0., 0., 0. ; // Newton-meter/sec
                            f_pred = humoto::wpg03::predictWrenchLinear(f_0, wpg.mpc_parameters_.preview_horizon_length_,
                                                                        slope, wpg.mpc_parameters_.getSamplingTime());
                        }

                        // prepare control problem for new iteration
                        // TODO: make sure f_pred only has z for mpc_type leader
                        switch(mpc_type)
                        {
                            case humoto::wpg03::MPC_FOLLOWER:
                            {
                                //                    wpg.setTrackingGains(20.);
                                wpg.setTrackingGains(1.,2.,3.,4.,5.,6.); // Dummy impedance gains
                                break;
                            }
                            case humoto::wpg03::MPC_LEADER:
                            {
                                Eigen::VectorXd com_traj(wpg.mpc_parameters_.preview_horizon_length_*6);
                                com_traj.setZero();
                                wpg.setTrajectory(com_traj);
                                wpg.setWrenchBounds(2.,2.,2.,2.);
                            }
                            default:
                            {
                                break;
                            }
                        }

                        if (wpg.update(model, walk_fsm, f_pred, mpc_type) != humoto::ControlProblemStatus::OK)
                        {
                            break;
                        }

                        // form an optimization problem
                        opt_problem.form(solution, model, wpg);

                        // solve an optimization problem
                        solver.solve(solution, opt_problem);
                        timer.stop();

                        // update FSM
                        walk_fsm.update(wpg_parameters);

                        // extract next model state from the solution and update model
                        model_state = wpg.getNextModelState(solution, walk_fsm, model, mpc_type);
                        model.updateState(model_state);
                    }

                    return solution.get_x();
                }
        };

        // Tests that the last MPC solution matches ref
        TEST_F(TestFixture, SolutionMatchReference)
        {
            ASSERT_TRUE(ref_solution.x_.isApprox(run(), 1e-8));
        }
    }
}


HUMOTO_DEFINE_REGRESSION_TEST_MAIN()
