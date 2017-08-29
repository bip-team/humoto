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
    namespace qpoases
    {
        /**
         * @brief Parameters of the solver
         */
        class HUMOTO_LOCAL SolverParameters : public humoto::SolverParametersBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "SolverParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR SolverParameters
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(humoto::SolverParametersBase) \
                \
                HUMOTO_CONFIG_SCALAR_(max_number_of_iterations) \
                HUMOTO_CONFIG_SCALAR_(max_cpu_time) \
                \
                HUMOTO_CONFIG_MEMBER_CLASS(options_, "QPoasesParameters")
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            private:
                class HUMOTO_LOCAL QPoasesParameters : public qpOASES::Options, public humoto::config::ConfigurableBase
                {
                    #define HUMOTO_CONFIG_SECTION_ID "QPoasesParameters"
                    #define HUMOTO_CONFIG_CONSTRUCTOR QPoasesParameters
                    #define HUMOTO_CONFIG_ENTRIES \
                        HUMOTO_CONFIG_ENUM(enableRamping)\
                        HUMOTO_CONFIG_ENUM(enableFarBounds)\
                        HUMOTO_CONFIG_ENUM(enableFlippingBounds)\
                        HUMOTO_CONFIG_ENUM(enableRegularisation)\
                        HUMOTO_CONFIG_ENUM(enableFullLITests)\
                        HUMOTO_CONFIG_ENUM(enableNZCTests)\
                        HUMOTO_CONFIG_SCALAR(enableDriftCorrection)\
                        HUMOTO_CONFIG_SCALAR(enableCholeskyRefactorisation)\
                        HUMOTO_CONFIG_ENUM(enableEqualities)\
                        HUMOTO_CONFIG_SCALAR(terminationTolerance)\
                        HUMOTO_CONFIG_SCALAR(boundTolerance)\
                        HUMOTO_CONFIG_SCALAR(boundRelaxation)\
                        HUMOTO_CONFIG_SCALAR(epsNum)\
                        HUMOTO_CONFIG_SCALAR(epsDen)\
                        HUMOTO_CONFIG_SCALAR(maxPrimalJump)\
                        HUMOTO_CONFIG_SCALAR(maxDualJump)\
                        HUMOTO_CONFIG_SCALAR(initialRamping)\
                        HUMOTO_CONFIG_SCALAR(finalRamping)\
                        HUMOTO_CONFIG_SCALAR(initialFarBounds)\
                        HUMOTO_CONFIG_SCALAR(growFarBounds)\
                        HUMOTO_CONFIG_ENUM(initialStatusBounds)\
                        HUMOTO_CONFIG_SCALAR(epsFlipping)\
                        HUMOTO_CONFIG_SCALAR(numRegularisationSteps)\
                        HUMOTO_CONFIG_SCALAR(epsRegularisation)\
                        HUMOTO_CONFIG_SCALAR(numRefinementSteps)\
                        HUMOTO_CONFIG_SCALAR(epsIterRef)\
                        HUMOTO_CONFIG_SCALAR(epsLITests)\
                        HUMOTO_CONFIG_SCALAR(epsNZCTests)\
                        HUMOTO_CONFIG_SCALAR(rcondSMin)\
                        HUMOTO_CONFIG_ENUM(enableInertiaCorrection)\
                        HUMOTO_CONFIG_ENUM(enableDropInfeasibles)\
                        HUMOTO_CONFIG_SCALAR(dropBoundPriority)\
                        HUMOTO_CONFIG_SCALAR(dropEqConPriority)\
                        HUMOTO_CONFIG_SCALAR(dropIneqConPriority)
                    #include HUMOTO_CONFIG_DEFINE_ACCESSORS

                    public:
                        QPoasesParameters()
                        {
                            setDefaults();
                        }

                        /**
                         * @brief Initialize to default parameters
                         */
                        void setDefaults()
                        {
                            setToMPC ();
                        }
                };


            public:
                /// Parameters for qpOASES
                QPoasesParameters   options_;

                /// Limit on the number of iterations
                int                 max_number_of_iterations_;

                /// Limit on the computation time
                double              max_cpu_time_;


            public:
                void setDefaults()
                {
                    max_number_of_iterations_ = 1000;
                    max_cpu_time_ = 0.0;
                }


                /**
                 * @brief Default constructor
                 */
                SolverParameters()
                {
                    setDefaults();
                }
        };



        /**
         * @brief QP solver.
         *
         * Solves hierarchy as an optimization problem of the following form:
         *                 minimize     1/2* x' H x + x' g
         *                 subject to   lb  <=  x <= ub
         *                              lbA <= A x <= ubA
         * where 'x' is solution, 'H' -- Hessian matrix, 'g' -- gradient
         * vector, 'lb' and 'ub' vectors of simple lower and upper bounds, 'Ax'
         * -- matrix of general constraints with bounds 'lbA' and 'ubA'.
         */
        class HUMOTO_LOCAL Solver : public humoto::QPSolverMixin< humoto::SolverGuessSolutionActiveSet<SolverParameters> >
        {
            private:
                /// If set the active set is returned
                bool                solution_guess_provided_;
                bool                active_set_guess_provided_;


                humoto::QPObjectiveSharedPointer    objective_;


                /// Constraints (lbA <= A x <= ubA)
                humoto::constraints::ContainerALU     general_constraints_;
                humoto::constraints::ContainerILU     bounds_;


                //@{
                /// Simple bounds on variables
                Eigen::VectorXd     lb_;
                Eigen::VectorXd     ub_;
                //@}

                /// Guess of the solution
                const double * solution_guess_;


                ///@{
                /// The active set guess for qpOASES
                qpOASES::Bounds         qpoases_active_set_guess_bounds_;
                qpOASES::Constraints    qpoases_active_set_guess_constraints_;
                ///@}

                Eigen::VectorXd     qpoases_active_set_bounds_;
                Eigen::VectorXd     qpoases_active_set_constraints_;


                /// Nonzero if constraints are specified
                std::size_t    number_of_constraints_;

                std::size_t    objective_level_;

                qpOASES::QProblem *  qp_;


            private:
                /// @copydoc humoto::Solver::initialize
                void initialize(  humoto::OptimizationProblem   &hierarchy,
                                  const humoto::SolutionStructure     &sol_structure)
                {
                    reset();

                    std::size_t     number_of_levels = hierarchy.getNumberOfLevels();
                    objective_level_ = number_of_levels - 1;

                    objective_ = hierarchy[objective_level_].getObjective();


                    if (number_of_levels > 1)
                    {
                        if (hierarchy[0].getNumberOfSimpleConstraints() > 0)
                        {
                            hierarchy[0].getSimpleConstraints(bounds_, lb_, ub_, sol_structure);
                        }


                        number_of_constraints_  = hierarchy[0].getNumberOfGeneralConstraints();
                        if (number_of_constraints_ > 0)
                        {
                            hierarchy[0].getGeneralConstraints(general_constraints_, sol_structure);
                        }
                    }

                    qp_ = new qpOASES::QProblem(sol_structure.getNumberOfVariables(),
                                                number_of_constraints_,
                                                qpOASES::HST_SEMIDEF);
                }


                /// @copydoc humoto::Solver::solveHierarchy
                void solveHierarchy(humoto::Solution &solution,
                                    const humoto::OptimizationProblem &hierarchy)
                {
                    qpOASES::real_t     max_time = parameters_.max_cpu_time_;

                    qpOASES::real_t     *max_time_ptr = NULL;

                    qpOASES::real_t     *lb_ptr = NULL;
                    qpOASES::real_t     *ub_ptr = NULL;

                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_rm;
                    qpOASES::real_t     *A_ptr = NULL;
                    qpOASES::real_t     *lbA_ptr = NULL;
                    qpOASES::real_t     *ubA_ptr = NULL;

                    const qpOASES::real_t     *solution_guess_ptr = NULL;

                    qpOASES::Bounds         *active_set_bounds_ptr = NULL;
                    qpOASES::Constraints    *active_set_constraints_ptr = NULL;

                    qpOASES::returnValue    qpoases_return_value;


                    if (max_time != 0.0)
                    {
                        max_time_ptr = &max_time;
                    }

                    if (bounds_.getNumberOfConstraints() > 0)
                    {
                        lb_ptr = lb_.data();
                        ub_ptr = ub_.data();
                    }

                    if (number_of_constraints_ > 0)
                    {
                        // Note: qpOASES expects rowMajor data, Eigen is colMajor by default
                        //       only A needs to be handled since H is symmetric and the rest
                        //       are vectors
                        A_rm = general_constraints_.getA();
                        A_ptr = A_rm.data();

                        lbA_ptr = general_constraints_.getLowerBounds().data();
                        ubA_ptr = general_constraints_.getUpperBounds().data();
                    }

                    if (solution_guess_provided_)
                    {
                        solution_guess_ptr = solution_guess_;
                    }


                    if (active_set_guess_provided_)
                    {
                        active_set_bounds_ptr       = &qpoases_active_set_guess_bounds_;
                        active_set_constraints_ptr  = &qpoases_active_set_guess_constraints_;
                    }


                    int number_of_iterations = parameters_.max_number_of_iterations_;

                    qp_->setOptions (parameters_.options_);
                    qpoases_return_value =
                        qp_->init(  objective_->getHessian().data(),
                                    objective_->getGradient().data(),
                                    A_ptr,
                                    lb_ptr,
                                    ub_ptr,
                                    lbA_ptr,
                                    ubA_ptr,
                                    number_of_iterations,
                                    max_time_ptr,
                                    solution_guess_ptr,
                                    NULL, // Optimal dual solution vector
                                    active_set_bounds_ptr,
                                    active_set_constraints_ptr);


                    qp_->getPrimalSolution( solution.x_.data() );



                    switch (qpoases_return_value)
                    {
                        case qpOASES::SUCCESSFUL_RETURN:
                            solution.return_status_ = SolverStatus::OK;
                            break;
                        case qpOASES::RET_MAX_NWSR_REACHED:
                            solution.return_status_ = SolverStatus::MAX_ITER;
                            break;
                        default:
                            solution.return_status_ = SolverStatus::OTHER;
                            break;
                    }

                    try
                    {
                        humoto::qpoases::Solution &qpoases_solution = dynamic_cast<humoto::qpoases::Solution &> (solution);

                        qpoases_solution.qpoases_return_value_ = qpoases_return_value;
                        qpoases_solution.number_of_iterations_ = number_of_iterations;
                    }
                    catch(...)
                    {
                        // not critical
                    }
                }



                /// @copydoc humoto::SolverGuessActiveSetMixin::getActiveSet
                void getActiveSet(  humoto::ActiveSet                   &active_set,
                                    const humoto::OptimizationProblem   &hierarchy)
                {
                    if (bounds_.getNumberOfConstraints() > 0)
                    {
                        qpoases_active_set_bounds_.resize(bounds_.getNumberOfVariables());
                        qp_->getWorkingSetBounds(qpoases_active_set_bounds_.data());
                    }

                    if (number_of_constraints_ > 0)
                    {
                        qpoases_active_set_constraints_.resize(number_of_constraints_);
                        qp_->getWorkingSetConstraints(qpoases_active_set_constraints_.data());
                    }


                    hierarchy.initializeActiveSet(active_set);

                    // Objective level includes only equalities by design.
                    active_set.set(objective_level_, ConstraintActivationType::EQUALITY);


                    for (std::size_t i = 0; i < number_of_constraints_; ++i)
                    {
                        if (qpoases_active_set_constraints_[i] == -1.0)
                        {
                            active_set[0][i] = ConstraintActivationType::LOWER_BOUND;
                        }
                        else
                        {
                            if (qpoases_active_set_constraints_[i] == 0.0)
                            {
                                active_set[0][i] = ConstraintActivationType::INACTIVE;
                            }
                            else
                            {
                                if (qpoases_active_set_constraints_[i] == 1.0)
                                {
                                    active_set[0][i] = ConstraintActivationType::UPPER_BOUND;
                                }
                                else
                                {
                                    HUMOTO_THROW_MSG("Active set returned by qpOASES is incorrectly initialized.");
                                }
                            }
                        }
                    }


                    for (std::size_t i = 0; i < bounds_.getNumberOfConstraints(); ++i)
                    {
                        std::size_t var_index = bounds_.getIndices()[i];

                        if (qpoases_active_set_bounds_[var_index] == -1.0)
                        {
                            if (bounds_.getLowerBounds()[i] < lb_[var_index])
                            {
                                active_set[0][number_of_constraints_ + i] = ConstraintActivationType::INACTIVE;
                            }
                            else
                            {
                                active_set[0][number_of_constraints_ + i] = ConstraintActivationType::LOWER_BOUND;
                            }
                        }
                        else
                        {
                            if (qpoases_active_set_bounds_[var_index] == 0.0)
                            {
                                active_set[0][number_of_constraints_ + i] = ConstraintActivationType::INACTIVE;
                            }
                            else
                            {
                                if (qpoases_active_set_bounds_[var_index] == 1.0)
                                {
                                    if (bounds_.getUpperBounds()[i] > ub_[var_index])
                                    {
                                        active_set[0][number_of_constraints_ + i] = ConstraintActivationType::INACTIVE;
                                    }
                                    else
                                    {
                                        active_set[0][number_of_constraints_ + i] = ConstraintActivationType::UPPER_BOUND;
                                    }
                                }
                                else
                                {
                                    HUMOTO_THROW_MSG("Active set returned by qpOASES is incorrectly initialized.");
                                }
                            }
                        }
                    }
                }


                /**
                 * @copydoc humoto::SolverGuessActiveSetMixin::setActiveSet
                 *
                 * @attention One variable may be simply bounded multiple
                 * times. In this case, the guess is set to the guess for the
                 * last bound on the level.
                 */
                void setActiveSet(  const humoto::ActiveSet             &active_set,
                                    const humoto::OptimizationProblem   &hierarchy)
                {
                    active_set_guess_provided_ = true;

                    qpoases_active_set_guess_constraints_.init(number_of_constraints_);

                    for (std::size_t i = 0; i < number_of_constraints_; ++i)
                    {
                        ConstraintActivationType::Type type = active_set[0][i];

                        switch (type)
                        {
                            case ConstraintActivationType::LOWER_BOUND:
                                qpoases_active_set_guess_constraints_.setupConstraint(i, qpOASES::ST_LOWER);
                                break;
                            case ConstraintActivationType::INACTIVE:
                                qpoases_active_set_guess_constraints_.setupConstraint(i, qpOASES::ST_INACTIVE);
                                break;
                            case ConstraintActivationType::UPPER_BOUND:
                                qpoases_active_set_guess_constraints_.setupConstraint(i, qpOASES::ST_UPPER);
                                break;
                            case ConstraintActivationType::EQUALITY:
                                qpoases_active_set_guess_constraints_.setupConstraint(i, qpOASES::ST_LOWER);
                                break;
                            default:
                                HUMOTO_THROW_MSG("Incorrectly initialized active set guess.");
                                break;
                        }
                    }


                    qpoases_active_set_guess_bounds_.init(bounds_.getNumberOfVariables());
                    qpoases_active_set_guess_bounds_.setupAllFree();
                    for (std::size_t i = 0; i < bounds_.getNumberOfConstraints() ; ++i)
                    {
                        std::size_t var_index = bounds_.getIndices()[i];
                        ConstraintActivationType::Type type = active_set[0][number_of_constraints_+i];

                        switch (type)
                        {
                            case ConstraintActivationType::LOWER_BOUND:
                                qpoases_active_set_guess_bounds_.setupBound(var_index, qpOASES::ST_LOWER);
                                break;
                            case ConstraintActivationType::INACTIVE:
                                // all bounds are already inactive due to setupAllFree().
                                //qpoases_active_set_guess_bounds_.setupBound(var_index, qpOASES::ST_INACTIVE);
                                break;
                            case ConstraintActivationType::UPPER_BOUND:
                                qpoases_active_set_guess_bounds_.setupBound(var_index, qpOASES::ST_UPPER);
                                break;
                            case ConstraintActivationType::EQUALITY:
                                qpoases_active_set_guess_bounds_.setupBound(var_index, qpOASES::ST_LOWER);
                                break;
                            default:
                                HUMOTO_THROW_MSG("Incorrectly initialized active set guess.");
                                break;
                        }
                    }
                }


                /// @copydoc humoto::SolverGuessSolutionMixin::setSolutionGuess
                void setSolutionGuess(const humoto::Solution & solution_guess)
                {
                    solution_guess_provided_ = true;
                    solution_guess_ = solution_guess.x_.data();
                }


                /// @copydoc humoto::Solver::reset
                void reset()
                {
                    number_of_constraints_ = 0;
                    objective_level_ = 0;

                    solution_guess_provided_ = false;
                    active_set_guess_provided_ = false;

                    if (qp_ != NULL)
                    {
                        delete qp_;
                        qp_ = NULL;
                    }
                }


            public:
                /**
                 * @brief Default constructor (with default parameters)
                 */
                Solver()
                {
                    qp_ = NULL;
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
                 * @brief Set parameters.
                 *
                 * @param[in] parameters parameters
                 */
                Solver(const SolverParameters &parameters)
                {
                    qp_ = NULL;
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
                            const std::string &name = "qpoases") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    general_constraints_.log(logger, subname, "general_constraints");

                    logger.log(LogEntryName(subname).add("lb"), lb_);
                    logger.log(LogEntryName(subname).add("ub"), ub_);
                }
        };
    }
}
