/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace humoto
{
    namespace lexls
    {
        /**
         * @brief Parameters of the solver
         */
        class HUMOTO_LOCAL SolverParameters : public LexLS::ParametersLexLSI, public humoto::SolverParametersBase
        {
            #define HUMOTO_CONFIG_SECTION_ID    "SolverParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR   SolverParameters
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(humoto::SolverParametersBase)\
                \
                HUMOTO_CONFIG_SCALAR(max_number_of_factorizations)\
                \
                HUMOTO_CONFIG_SCALAR(tol_linear_dependence  )\
                HUMOTO_CONFIG_SCALAR(tol_wrong_sign_lambda  )\
                HUMOTO_CONFIG_SCALAR(tol_correct_sign_lambda)\
                HUMOTO_CONFIG_SCALAR(tol_feasibility        )\
                \
                HUMOTO_CONFIG_SCALAR(cycling_handling_enabled)\
                HUMOTO_CONFIG_SCALAR(cycling_max_counter     )\
                HUMOTO_CONFIG_SCALAR(cycling_relax_step      )\
                \
                HUMOTO_CONFIG_ENUM(regularization_type)\
                HUMOTO_CONFIG_SCALAR(max_number_of_CG_iterations    )\
                HUMOTO_CONFIG_SCALAR(variable_regularization_factor )\
                \
                HUMOTO_CONFIG_SCALAR(modify_x_guess_enabled      )\
                HUMOTO_CONFIG_SCALAR(modify_type_active_enabled  )\
                HUMOTO_CONFIG_SCALAR(modify_type_inactive_enabled)\
                HUMOTO_CONFIG_SCALAR(set_min_init_ctr_violation  )\
                \
                HUMOTO_CONFIG_SCALAR(use_phase1_v0              )\
                HUMOTO_CONFIG_SCALAR(log_working_set_enabled    )\
                HUMOTO_CONFIG_SCALAR(deactivate_first_wrong_sign)
            #include "humoto/config/define_accessors.h"


            public:
                /**
                 * @brief Default constructor
                 */
                SolverParameters()
                {
                    setDefaults();
                }

                /**
                 * @brief Set default parameters.
                 */
                void setDefaults()
                {
                    LexLS::ParametersLexLSI::setDefaults();
                }
        };



        /**
         * @brief LexLS solver.
         */
        class HUMOTO_LOCAL Solver : public humoto::SolverGuessSolutionActiveSet<SolverParameters>
        {
            private:
                std::vector<LexLS::ObjectiveType>   obj_type_;
                std::vector<LexLS::Index>           num_ctr_;
                std::vector<LexLS::Index>           simple_bounds_indicies_;

                std::vector<Eigen::MatrixXd>        ctr_data_;

                std::size_t                         first_general_level_;

                LexLS::internal::LexLSI *           lexlsi_;


            private:
                /// @copydoc humoto::Solver::initialize
                void initialize(  const humoto::OptimizationProblem   &hierarchy,
                                  const humoto::SolutionStructure     &sol_structure)
                {
                    reset();

                    initializeSolver(hierarchy, sol_structure);
                    formConstraints(hierarchy, sol_structure);
                }


                /**
                 * @brief Check hierarchy, set flags, count variables.
                 *
                 * @param[in] hierarchy hierarchy
                 * @param[in] sol_structure solution structure
                 */
                void initializeSolver(const humoto::OptimizationProblem   &hierarchy,
                                      const humoto::SolutionStructure     &sol_structure)
                {
                    std::size_t num_obj = hierarchy.getNumberOfLevels();
                    first_general_level_ = 0;


                    obj_type_.resize(num_obj);
                    num_ctr_.resize(num_obj);
                    ctr_data_.resize(num_obj);


                    for (std::size_t i = 0; i < num_obj; ++i)
                    {
                        num_ctr_[i] = hierarchy[i].getNumberOfConstraints();

                        if ( (i == 0) && hierarchy[i].isSimple() )
                        {
                            obj_type_[i] = LexLS::SIMPLE_BOUNDS_OBJECTIVE;
                            first_general_level_ = 1;
                        }
                        else
                        {
                            obj_type_[i] = LexLS::GENERAL_OBJECTIVE;
                        }
                    }


                    try
                    {
                        lexlsi_ = new LexLS::internal::LexLSI ( sol_structure.getNumberOfVariables(),
                                                                num_obj,
                                                                num_ctr_.data(),
                                                                obj_type_.data());

                        lexlsi_->setParameters(parameters_);
                    }
                    catch (const std::exception &e)
                    {
                        HUMOTO_THROW_MSG(e.what());
                    }
                }



                /**
                 * @brief Form a QP based on a hierarchy.
                 *
                 * @param[in] hierarchy hierarchy.
                 * @param[in] sol_structure solution structure
                 */
                void formConstraints ( const humoto::OptimizationProblem   &hierarchy,
                                       const humoto::SolutionStructure     &sol_structure)
                {
                    if (hierarchy[0].isSimple())
                    {
                        humoto::constraints::ContainerILU  simple_constraints_;
                        hierarchy[0].getSimpleConstraints(simple_constraints_, sol_structure);


                        simple_bounds_indicies_.resize(simple_constraints_.getNumberOfConstraints());
                        for (std::size_t i = 0; i < simple_constraints_.getNumberOfConstraints(); ++i)
                        {
                            simple_bounds_indicies_[i] = simple_constraints_.getIndices()[i];
                        }


                        etools::concatenateMatricesHorizontally(ctr_data_[0],
                                                                simple_constraints_.getLowerBounds(),
                                                                simple_constraints_.getUpperBounds());

                        try
                        {
                            lexlsi_->setData(0, simple_bounds_indicies_.data(), ctr_data_[0]);
                        }
                        catch (const std::exception &e)
                        {
                            HUMOTO_THROW_MSG(e.what());
                        }
                    }


                    for (std::size_t i = first_general_level_; i < hierarchy.getNumberOfLevels(); ++i)
                    {
                        humoto::constraints::ContainerALU     general_constraints_;
                        hierarchy[i].getAllConstraints(general_constraints_, sol_structure);


                        etools::concatenateMatricesHorizontally(ctr_data_[i],
                                                                general_constraints_.getA(),
                                                                general_constraints_.getLowerBounds(),
                                                                general_constraints_.getUpperBounds());

                        try
                        {
                            lexlsi_->setData(i, ctr_data_[i]);
                        }
                        catch (const std::exception &e)
                        {
                            HUMOTO_THROW_MSG(e.what());
                        }
                    }
                }



                /// @copydoc humoto::Solver::solveHierarchy
                void solveHierarchy(humoto::Solution &solution,
                                    const humoto::OptimizationProblem &hierarchy)
                {
                    try
                    {
                        LexLS::TerminationStatus status = lexlsi_->solve();


                        solution.x_ = lexlsi_->get_x();


                        switch (status)
                        {
                            case LexLS::PROBLEM_SOLVED:
                                solution.return_status_ = SolverStatus::OK;
                                break;
                            case LexLS::MAX_NUMBER_OF_FACTORIZATIONS_EXCEEDED:
                                solution.return_status_ = SolverStatus::MAX_ITER;
                                break;
                            default:
                                solution.return_status_ = SolverStatus::OTHER;
                                break;
                        }


                        try
                        {
                            humoto::lexls::Solution &lexls_solution = dynamic_cast<humoto::lexls::Solution &> (solution);

                            lexls_solution.lexls_termination_status_ = status;
                            lexls_solution.number_of_activations_    = lexlsi_->getActivationsCount();
                            lexls_solution.number_of_deactivations_  = lexlsi_->getDeactivationsCount();
                            lexls_solution.number_of_factorizations_ = lexlsi_->getFactorizationsCount();
                            lexls_solution.cycling_counter_          = lexlsi_->getCyclingCounter();
                        }
                        catch(...)
                        {
                            // not critical
                        }
                    }
                    catch (const std::exception &e)
                    {
                        HUMOTO_THROW_MSG(e.what());
                    }
                }



                /// @copydoc humoto::SolverGuessActiveSetMixin::getActiveSet
                void getActiveSet(  humoto::ActiveSet                   &active_set,
                                    const humoto::OptimizationProblem   &hierarchy)
                {
                    hierarchy.initializeActiveSet(active_set);
                    std::vector< LexLS::ConstraintActivationType >  lexlsi_active_constraints;


                    for (std::size_t i = 0; i < hierarchy.getNumberOfLevels(); ++i)
                    {
                        try
                        {
                            lexlsi_active_constraints.clear();
                            lexlsi_->getActiveCtr(i, lexlsi_active_constraints);
                        }
                        catch (const std::exception &e)
                        {
                            HUMOTO_THROW_MSG(e.what());
                        }


                        for (std::size_t j = 0; j < lexlsi_active_constraints.size(); ++j)
                        {
                            switch (lexlsi_active_constraints[j])
                            {
                                case LexLS::CTR_ACTIVE_LB:
                                    active_set[i][j] = humoto::ConstraintActivationType::LOWER_BOUND;
                                    break;
                                case LexLS::CTR_ACTIVE_UB:
                                    active_set[i][j] = humoto::ConstraintActivationType::UPPER_BOUND;
                                    break;
                                case LexLS::CTR_ACTIVE_EQ:
                                    active_set[i][j] = humoto::ConstraintActivationType::EQUALITY;
                                    break;
                                case LexLS::CTR_INACTIVE:
                                    active_set[i][j] = humoto::ConstraintActivationType::INACTIVE;
                                    break;
                                default:
                                    HUMOTO_THROW_MSG("Incorrectly initialized active set guess.");
                                    break;
                            }
                        }
                    }
                }


                /// @copydoc humoto::SolverGuessActiveSetMixin::setActiveSet
                void setActiveSet(  const humoto::ActiveSet             &active_set,
                                    const humoto::OptimizationProblem   &hierarchy)
                {
                    for (std::size_t i = 0; i < hierarchy.getNumberOfLevels(); ++i)
                    {
                        for (LexLS::Index j = 0; j < num_ctr_[i]; ++j)
                        {
                            switch (active_set[i][j])
                            {
                                case humoto::ConstraintActivationType::INACTIVE:
                                    // nothing to activate
                                    break;
                                case humoto::ConstraintActivationType::LOWER_BOUND:
                                    lexlsi_->api_activate(i, j, LexLS::CTR_ACTIVE_LB);
                                    break;
                                case humoto::ConstraintActivationType::UPPER_BOUND:
                                    lexlsi_->api_activate(i, j, LexLS::CTR_ACTIVE_UB);
                                    break;
                                case humoto::ConstraintActivationType::EQUALITY:
                                    //lexlsi_->api_activate(i, j, LexLS::CTR_ACTIVE_EQ);
                                    break;
                                default:
                                    HUMOTO_THROW_MSG("Incorrectly initialized active set guess.");
                                    break;
                            }
                        }
                    }
                }


                /// @copydoc humoto::SolverGuessSolutionMixin::setSolutionGuess
                void setSolutionGuess(const humoto::Solution & solution_guess)
                {
                    lexlsi_->set_x0(solution_guess.x_);
                }


                /// @copydoc humoto::Solver::reset
                void reset()
                {
                    if (lexlsi_ != NULL)
                    {
                        delete lexlsi_;
                        lexlsi_ = NULL;
                    }
                }


            public:
                /**
                 * @brief Default constructor (with default parameters)
                 */
                Solver()
                {
                    lexlsi_ = NULL;
                    reset();
                }


                /**
                 * @brief Destructor
                 */
                ~Solver()
                {
                    reset();
                }


                /**
                 * @brief Construct solver with specified parameters.
                 *
                 * @param[in] parameters parameters
                 */
                Solver(const SolverParameters &parameters)
                {
                    lexlsi_ = NULL;
                    setParameters(parameters);
                }



                /**
                 * @brief Log a QP problem
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string &name = "lexls") const
                {
                    LogEntryName subname = parent;
                    subname.add(name).add("level");

                    for (std::size_t i = 0; i < ctr_data_.size(); ++i)
                    {
                        LogEntryName subname_i = subname;
                        subname_i.add(i);

                        logger.log(LogEntryName(subname_i).add("type"), obj_type_[i]);
                        logger.log(LogEntryName(subname_i).add("num_ctr"), num_ctr_[i]);

                        logger.log(LogEntryName(subname_i).add("data"), ctr_data_[i]);

                        if (obj_type_[i] == LexLS::SIMPLE_BOUNDS_OBJECTIVE)
                        {
                            logger.log(LogEntryName(subname_i).add("simple_bounds_indicies"), simple_bounds_indicies_);
                        }
                    }
                }
        };
    }
}
