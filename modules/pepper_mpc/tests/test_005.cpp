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
#ifdef HUMOTO_BRIDGE_lexls
#include "humoto/lexls.h"
#endif
// specific control problem (many can be included simultaneously)
#include "humoto/pepper_mpc.h"

#include "utilities.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);



/**
 * @brief Simple control loop
 *
 * @param[in] argc number of arguments
 * @param[in] argv arguments
 *
 * @return status
 */
int main(int argc, char **argv)
{
    std::string config_path = humoto_tests::getConfigPath(argc, argv, 1);
    std::string log_file_name = humoto_tests::getLogFileName(argc, argv, 2);

    try
    {
        // optimization problem (a stack of tasks / hierarchy)
        humoto::pepper_mpc::ConfigurableOptimizationProblem     opt_problem;

        // a solver which is giong to be used
        humoto::qpoases::SolverParameters         qpoases_solver_parameters;
        //solver_parameters.crash_on_any_failure_ = false;
        humoto::qpoases::Solver                   qpoases_solver(qpoases_solver_parameters);

#ifdef HUMOTO_BRIDGE_lexls
        humoto::lexls::SolverParameters         lexls_solver_parameters;
        //solver_parameters.crash_on_any_failure_ = false;
        humoto::lexls::Solver                   lexls_solver(lexls_solver_parameters);
#endif


        // solution
        humoto::qpoases::Solution               qpoases_solution;
#ifdef HUMOTO_BRIDGE_lexls
        humoto::lexls::Solution                 lexls_solution;
#endif


        humoto::pepper_mpc::RobotParameters         robot_parameters(config_path + "robot_parameters.yaml");
        // model representing the controlled system
        humoto::pepper_mpc::Model                   model(robot_parameters);

        // parameters of the control problem
        humoto::pepper_mpc::MPCParameters           mg_parameters(config_path + "mpc_parameters.yaml");
        // control problem, which is used to construct an optimization problem
        humoto::pepper_mpc::MPCforMG                mg(mg_parameters);


        // options for walking
        humoto::pepper_mpc::MotionParameters        motion_parameters(config_path + "motion_parameters_circle_fast.yaml");

        switch (motion_parameters.motion_mode_)
        {
            case humoto::pepper_mpc::MotionMode::MAINTAIN_POSITION:
                opt_problem.readConfig(config_path + "hierarchies.yaml", true, "Hierarchy01");
                break;
            case humoto::pepper_mpc::MotionMode::MAINTAIN_VELOCITY:
                opt_problem.readConfig(config_path + "hierarchies.yaml", true, "Hierarchy00");
                break;
            default:
                HUMOTO_THROW_MSG("Unsupported motion mode.");
        }

        humoto::pepper_mpc::ModelState   model_state(config_path + "initial_state_pepper.yaml");
        model.updateState(model_state);


        humoto::ActiveSet qpoases_active_set_actual;
        humoto::ActiveSet qpoases_active_set_determined;
        humoto::Violations  qpoases_violations;

#ifdef HUMOTO_BRIDGE_lexls
        humoto::ActiveSet lexls_active_set_actual;
        humoto::ActiveSet lexls_active_set_determined;
        humoto::Violations  lexls_violations;
#endif

        humoto::Logger      logger(log_file_name);
        humoto::LogEntryName    prefix;

        for (unsigned int i = 0;; ++i)
        {
            prefix = humoto::LogEntryName("humoto").add(i);

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
            // logging
            opt_problem.log(logger, prefix);

            qpoases_violations.log(logger, prefix);
            qpoases_active_set_actual.log(logger, prefix, "qpoases_active_set_qpoases");
            qpoases_active_set_determined.log(logger, prefix, "qpoases_active_set_determined");
            qpoases_solver.log(logger, prefix);
            qpoases_solution.log(logger, prefix);

#ifdef HUMOTO_BRIDGE_lexls
            lexls_violations.log(logger, prefix);
            lexls_active_set_actual.log(logger, prefix, "lexls_active_set_lexls");
            lexls_active_set_determined.log(logger, prefix, "lexls_active_set_determined");
            lexls_solver.log(logger, prefix);
            lexls_solution.log(logger, prefix);
#endif

            mg.log(logger, prefix);
            model.log(logger, prefix);
            //========================

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
                                    std::cout   << "QPOASES:  "
                                                << "Iteration = " << std::setw(3) << i
                                                << " | "
                                                << "Level = " << std::setw(3) << l
                                                << " | "
                                                << "Constaint = " << std::setw(3) << k
                                                << " // "
                                                << "solver = " << qpoases_active_set_actual[l][k]
                                                << " | "
                                                << "determined = " << qpoases_active_set_determined[l][k]
                                                << " // "
                                                << "violation = ["
                                                    << qpoases_violations[l][k] << ", "
                                                    << qpoases_violations[l].getLower(k) << ", "
                                                    << qpoases_violations[l].getUpper(k) << "]"
                                                << std::endl;
                                    break;
                            }
                        }
                    }

#ifdef HUMOTO_BRIDGE_lexls
                    if (lexls_active_set_determined[l][k] != lexls_active_set_actual[l][k])
                    {
                        std::cout   << "LEXLS:    "
                                    << "Iteration = " << std::setw(3) << i
                                    << " | "
                                    << "Level = " << std::setw(3) << l
                                    << " | "
                                    << "Constaint = " << std::setw(3) << k
                                    << " // "
                                    << "solver = " << lexls_active_set_actual[l][k]
                                    << " | "
                                    << "determined = " << lexls_active_set_determined[l][k]
                                    << " // "
                                    << "violation = ["
                                        << lexls_violations[l][k] << ", "
                                        << lexls_violations[l].getLower(k) << ", "
                                        << lexls_violations[l].getUpper(k) << "]"
                                    << std::endl;
                    }
#endif
                }
            }
            //========================

            // extract next model state from the solution and update model
            model_state = mg.getNextModelState(qpoases_solution, model);
            model.updateState(model_state);
        }
    }
    catch (const std::exception &e)
    {
        HUMOTO_LOG_RAW(e.what());
        exit(-1);
    }
    return (0);
}

