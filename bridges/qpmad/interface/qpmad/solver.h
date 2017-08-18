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
                HUMOTO_CONFIG_PARENT_CLASS(SolverParametersBase); \
                HUMOTO_CONFIG_SCALAR_(regularization_factor); \
                HUMOTO_CONFIG_MEMBER_CLASS(options_, "QPmadParameters");
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            public:
                class QPmadParameters : public ::qpmad::SolverParameters,
                                        public humoto::config::ConfigurableBase
                {
                    #define HUMOTO_CONFIG_SECTION_ID "QPmadParameters"
                    #define HUMOTO_CONFIG_CONSTRUCTOR QPmadParameters
                    #define HUMOTO_CONFIG_ENTRIES \
                        HUMOTO_CONFIG_ENUM_(hessian_type);\
                        HUMOTO_CONFIG_SCALAR_(tolerance);\
                        HUMOTO_CONFIG_SCALAR_(max_iter);
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

                /// Hessian
                Eigen::MatrixXd     H_;

                /// Gradient vector
                Eigen::VectorXd     g_;


                /// Constraints
                humoto::constraints::ContainerALU     general_constraints_;

                humoto::constraints::ContainerILU     bounds_;
                Eigen::VectorXd     lb_;
                Eigen::VectorXd     ub_;


            private:
                /// @copydoc humoto::Solver::initialize
                void initialize(  const humoto::OptimizationProblem   &hierarchy,
                                  const humoto::SolutionStructure     &sol_structure)
                {
                    reset();

                    std::size_t     number_of_levels = hierarchy.getNumberOfLevels();
                    std::size_t     objective_level_ = number_of_levels - 1;

                    hierarchy[objective_level_].getObjective(H_, g_);


                    if (number_of_levels > 1)
                    {
                        if (hierarchy[0].getNumberOfSimpleConstraints() > 0)
                        {
                            hierarchy[0].getSimpleConstraints(bounds_, lb_, ub_, sol_structure);
                        }

                        if (hierarchy[0].getNumberOfGeneralConstraints() > 0)
                        {
                            hierarchy[0].getGeneralConstraints(general_constraints_, sol_structure);
                        }
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
                        for (EigenIndex i = 0; i < H_.rows(); ++i)
                        {
                            H_(i,i) += parameters_.regularization_factor_;
                        }
                    }

                    ::qpmad::Solver::ReturnStatus qp_status =
                        solver.solve(   solution.x_,
                                        H_,
                                        g_,
                                        lb_,
                                        ub_,
                                        general_constraints_.getA(),
                                        general_constraints_.getLowerBounds(),
                                        general_constraints_.getUpperBounds());


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
                            const std::string &name = "qpmad") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    logger.log(LogEntryName(subname).add("H"), H_);
                    logger.log(LogEntryName(subname).add("g"), g_);

                    general_constraints_.log(logger, subname, "general_constraints");

                    logger.log(LogEntryName(subname).add("lb"), lb_);
                    logger.log(LogEntryName(subname).add("ub"), ub_);
                }
        };
    }
}
