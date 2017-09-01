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

                humoto::QPProblem_AB_AL     qp_problem_;


            private:
                /// @copydoc humoto::Solver::initialize
                void initialize(  humoto::OptimizationProblem   &hierarchy,
                                  const humoto::SolutionStructure     &sol_structure)
                {
                    hierarchy.getQPProblem(qp_problem_, sol_structure);
                }



                /// @copydoc humoto::Solver::solveHierarchy
                void solveHierarchy(humoto::Solution &solution,
                                    const humoto::OptimizationProblem &hierarchy)
                {
                    HUMOTO_ASSERT ( parameters_.regularization_factor_ >= 0,
                                    "Regularization factor must be nonnegative.");

                    if (parameters_.regularization_factor_ > 0)
                    {
                        qp_problem_.regularize(parameters_.regularization_factor_);
                    }

                    double qp_status = solver.solve(qp_problem_.getHessian(),
                                                    qp_problem_.getGradient(),
                                                    qp_problem_.getEqualities().getA().transpose(),
                                                    -qp_problem_.getEqualities().getB(),
                                                    qp_problem_.getInequalities().getA().transpose(),
                                                    -qp_problem_.getInequalities().getLowerBounds(),
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
                }


                /**
                 * @brief Destructor
                 */
                ~Solver()
                {
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
                    qp_problem_.log(logger, subname);
                }
        };
    }
}
