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
#include "humoto/qpoases.h"
#include "humoto/lexls.h"
// specific control problem (many can be included simultaneously)
#include "humoto/wpg04.h"

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
    std::string     log_file_name = humoto_tests::getLogFileName(argc, argv);

    try
    {
        // optimization problem (a stack of tasks / hierarchy)
        humoto::OptimizationProblem               opt_problem;

        // a solver which is giong to be used
        humoto::qpoases::SolverParameters         qpoases_solver_parameters;
        //solver_parameters.crash_on_any_failure_ = false;
        humoto::qpoases::Solver                   qpoases_solver(qpoases_solver_parameters);

        humoto::lexls::SolverParameters         lexls_solver_parameters;
        //solver_parameters.crash_on_any_failure_ = false;
        humoto::lexls::Solver                   lexls_solver(lexls_solver_parameters);


        // solution
        humoto::qpoases::Solution               qpoases_solution;
        humoto::lexls::Solution                 lexls_solution;


        // options for walking
        humoto::wpg04::WalkParameters                walk_parameters;
        walk_parameters.com_velocity_ << 0.1, 0.;
        walk_parameters.first_stance_com_velocity_ = walk_parameters.com_velocity_;
        walk_parameters.num_steps_ = 7;
        //FSM for walking
        humoto::walking::StanceFiniteStateMachine stance_fsm(walk_parameters);
        // model representing the controlled system
        humoto::wpg04::Model                      model;
        // parameters of the control problem
        humoto::wpg04::MPCParameters              wpg_parameters;
        // control problem, which is used to construct an optimization problem
        humoto::wpg04::MPCforWPG                  wpg(wpg_parameters);


        setupHierarchy_v5(opt_problem);

        humoto::wpg04::ModelState model_state;

        humoto::ActiveSet qpoases_active_set_actual;
        humoto::ActiveSet qpoases_active_set_determined;
        humoto::Violations  qpoases_violations;

        humoto::ActiveSet lexls_active_set_actual;
        humoto::ActiveSet lexls_active_set_determined;
        humoto::Violations  lexls_violations;

        humoto::Timer       timer;
        humoto::Logger      logger(log_file_name);
        humoto::LogEntryName    prefix;

        for (unsigned int i = 0;; ++i)
        {
            prefix = humoto::LogEntryName("humoto").add(i);

            timer.start();
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

            timer.stop();

            opt_problem.determineActiveSet(qpoases_active_set_determined, qpoases_solution);
            opt_problem.computeViolations(qpoases_violations, qpoases_solution);

            opt_problem.determineActiveSet(lexls_active_set_determined, lexls_solution);
            opt_problem.computeViolations(lexls_violations, lexls_solution);


            //========================
            // logging
            opt_problem.log(logger, prefix);

            qpoases_violations.log(logger, prefix);
            qpoases_active_set_actual.log(logger, prefix, "qpoases_active_set_qpoases");
            qpoases_active_set_determined.log(logger, prefix, "qpoases_active_set_determined");
            qpoases_solver.log(logger, prefix);
            qpoases_solution.log(logger, prefix);

            lexls_violations.log(logger, prefix);
            lexls_active_set_actual.log(logger, prefix, "lexls_active_set_lexls");
            lexls_active_set_determined.log(logger, prefix, "lexls_active_set_determined");
            lexls_solver.log(logger, prefix);
            lexls_solution.log(logger, prefix);

            stance_fsm.log(logger, prefix);
            wpg.log(logger, prefix);
            model.log(logger, prefix);
            //========================

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
                                }
                            }
                        }
                    }

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
                }
            }
            //========================

            // extract next model state from the solution and update model
            stance_fsm.shiftTime(wpg_parameters.sampling_time_ms_);
            model_state = wpg.getNextModelState(qpoases_solution, stance_fsm, model);
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

