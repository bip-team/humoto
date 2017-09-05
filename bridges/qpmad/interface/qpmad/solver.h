/**
    @file
    @author  Alexander Sherikov

    @brief
*/

#pragma once


namespace humoto
{
    namespace qpmad
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
                HUMOTO_CONFIG_SCALAR_(regularization_factor) \
                HUMOTO_CONFIG_MEMBER_CLASS(options_, "QPmadParameters")
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            public:
                class QPmadParameters : public ::qpmad::SolverParameters,
                                        public humoto::config::ConfigurableBase
                {
                    #define HUMOTO_CONFIG_SECTION_ID "QPmadParameters"
                    #define HUMOTO_CONFIG_CONSTRUCTOR QPmadParameters
                    #define HUMOTO_CONFIG_ENTRIES \
                        HUMOTO_CONFIG_ENUM_(hessian_type)\
                        HUMOTO_CONFIG_SCALAR_(tolerance)\
                        HUMOTO_CONFIG_SCALAR_(max_iter)
                    #include HUMOTO_CONFIG_DEFINE_ACCESSORS

                    protected:
                        void setDefaults()
                        {
                        }

                    public:
                        QPmadParameters()
                        {
                        }
                };


                /// Regularization factor (disabled if = 0)
                double              regularization_factor_;

                QPmadParameters     options_;


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
                ::qpmad::Solver       solver;

                humoto::QPProblem_ILU_ALU       qp_problem_;


            private:
                /// @copydoc humoto::Solver::initialize
                void initialize(  humoto::OptimizationProblem   &hierarchy,
                                  const humoto::SolutionStructure     &sol_structure)
                {
                    hierarchy.getQPProblem(qp_problem_, sol_structure, false);
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

                    ::qpmad::Solver::ReturnStatus qp_status =
                        solver.solve(   solution.x_,
                                        qp_problem_.getHessian(),
                                        qp_problem_.getGradient(),
                                        qp_problem_.getSolutionLowerBounds(),
                                        qp_problem_.getSolutionUpperBounds(),
                                        qp_problem_.getGeneralConstraints().getA(),
                                        qp_problem_.getGeneralConstraints().getLowerBounds(),
                                        qp_problem_.getGeneralConstraints().getUpperBounds());


                    switch (qp_status)
                    {
                        case ::qpmad::Solver::OK:
                            solution.return_status_ = SolverStatus::OK;
                            break;
                        case ::qpmad::Solver::MAXIMAL_NUMBER_OF_ITERATIONS:
                            solution.return_status_ = SolverStatus::MAX_ITER;
                            break;
                        default:
                            solution.return_status_ = SolverStatus::OTHER;
                            break;
                    }


                    try
                    {
                        humoto::qpmad::Solution &qpmad_solution = dynamic_cast<humoto::qpmad::Solution &> (solution);

                        qpmad_solution.qpmad_return_value_ = qp_status;
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
                            const std::string &name = "qpmad") const
                {
                    LogEntryName subname = parent; subname.add(name);
                    qp_problem_.log(logger, subname);
                }
        };
    }
}
