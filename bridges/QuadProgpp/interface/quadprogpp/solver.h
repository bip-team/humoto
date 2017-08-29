/**
    @file
    @author  Alexander Sherikov

    @brief
*/

#pragma once


namespace humoto
{
    namespace quadprogpp
    {
        /**
         * @brief Parameters of the solver
         */
        class HUMOTO_LOCAL SolverParameters : public humoto::SolverParametersBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "SolverParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR SolverParameters
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(SolverParametersBase) \
                HUMOTO_CONFIG_SCALAR_(regularization_factor)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            public:
                /// Regularization factor (disabled if = 0)
                double              regularization_factor_;


            public:
                void setDefaults()
                {
                    regularization_factor_ = 0;
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
         */
        class HUMOTO_LOCAL Solver : public humoto::QPSolverMixin< humoto::Solver<SolverParameters> >
        {
            private:
                QuadProgpp::Solver solver;


                humoto::QPObjectiveSharedPointer    objective_;


                /// Constraints (lbA <= A x)
                humoto::constraints::ContainerAL     inequality_constraints_;
                humoto::constraints::ContainerAB     equality_constraints_;


            private:
                /// @copydoc humoto::Solver::initialize
                void initialize(  humoto::OptimizationProblem   &hierarchy,
                                  const humoto::SolutionStructure     &sol_structure)
                {
                    reset();

                    std::size_t     number_of_levels = hierarchy.getNumberOfLevels();
                    std::size_t     objective_level_ = number_of_levels - 1;

                    objective_ = hierarchy[objective_level_].getObjective();


                    if (number_of_levels > 1)
                    {
                        hierarchy[0].getInEqualityConstraints(  inequality_constraints_,
                                                                equality_constraints_,
                                                                sol_structure);
                    }
                }



                /// @copydoc humoto::Solver::solveHierarchy
                void solveHierarchy(humoto::Solution &solution,
                                    const humoto::OptimizationProblem &hierarchy)
                {
                    HUMOTO_ASSERT ( parameters_.regularization_factor_ >= 0,
                                    "Regularization factor must be nonnegative.");

                    if (parameters_.regularization_factor_ > 0)
                    {
                        objective_->regularize(parameters_.regularization_factor_);
                    }

                    double qp_status = solver.solve(objective_->getHessian(),
                                                    objective_->getGradient(),
                                                    equality_constraints_.getA().transpose(),
                                                    -equality_constraints_.getB(),
                                                    inequality_constraints_.getA().transpose(),
                                                    -inequality_constraints_.getLowerBounds(),
                                                    solution.x_);


                    if (qp_status == QuadProgpp::Status::OK)
                    {
                        solution.return_status_ = SolverStatus::OK;
                    }
                    else
                    {
                        solution.return_status_ = SolverStatus::OTHER;
                    }


                    try
                    {
                        humoto::quadprogpp::Solution &quadprogpp_solution = dynamic_cast<humoto::quadprogpp::Solution &> (solution);

                        quadprogpp_solution.quadprogpp_return_value_ = qp_status;
                    }
                    catch(...)
                    {
                        // not critical
                    }
                }


            public:
                /**
                 * @brief Default constructor (with default parameters)
                 */
                Solver()
                {
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
                            const std::string &name = "quadprogpp") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    equality_constraints_.log(logger, subname, "equality_constraints");
                    inequality_constraints_.log(logger, subname, "inequality_constraints");
                }
        };
    }
}
