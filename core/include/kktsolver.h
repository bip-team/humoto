/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    /**
     * @brief A trivial solver for problems with equality constraints.
     *
     * @ingroup Solvers
     * @{
     * @defgroup kktsolver kktsolver
     * @}
     *
     * @ingroup kktsolver
     */
    namespace kktsolver
    {
    }
}


namespace humoto
{
    namespace kktsolver
    {
        /**
         * @brief Parameters of the solver
         *
         * @attention @ref humoto::kktsolver::SolverParameters::CONSTRAINT_ELIMINATION_LLT
         * works only with diagonal Hessians, i.e., the second level must
         * contain simple objectives only.
         */
        class HUMOTO_LOCAL SolverParameters : public humoto::SolverParametersBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "SolverParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR SolverParameters
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(SolverParametersBase)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            public:
                enum Method
                {
                    UNDEFINED   = 0,
                    LU          = 1,
                    QR          = 2,
                    CONSTRAINT_ELIMINATION_LLT = 3
                };


            public:
                Method      solution_method_;
                double      elimination_regularization_factor_;



            public:
                void setDefaults()
                {
                    solution_method_ = LU;
                    elimination_regularization_factor_ = 1e-08;
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
        class HUMOTO_LOCAL Solver : public humoto::Solver<SolverParameters>
        {
            private:
                humoto::QPProblem_AB        qp_problem_;



            private:
                /// @copydoc humoto::Solver::initialize
                void initialize(  const humoto::OptimizationProblem   &hierarchy,
                                  const humoto::SolutionStructure     &sol_structure)
                {
                    hierarchy.getQPProblem(qp_problem_, sol_structure);
                }



                /// @copydoc humoto::Solver::solveHierarchy
                void solveHierarchy(humoto::Solution &solution,
                                    const humoto::OptimizationProblem &hierarchy)
                {
                    switch(parameters_.solution_method_)
                    {
                        case SolverParameters::CONSTRAINT_ELIMINATION_LLT:
                            if (QPObjective::HESSIAN_DIAGONAL == qp_problem_.getHessianType())
                            {
                                solveWithElimination(solution, hierarchy);
                            }
                            else
                            {
                                HUMOTO_THROW_MSG("This solution method requires diagonal Hessian.");
                            }
                            break;
                        case SolverParameters::LU:
                        case SolverParameters::QR:
                            solveDirect(solution, hierarchy);
                            break;
                        default:
                            HUMOTO_THROW_MSG("Unknown solution method.");
                    }
                }


                /**
                 * @brief Eliminate constraints and solve (only diagonal Hessians are supported).
                 *
                 * @param[out] solution
                 * @param[in] hierarchy
                 */
                void solveWithElimination(  humoto::Solution &solution,
                                            const humoto::OptimizationProblem &hierarchy)
                {
                    std::ptrdiff_t  num_var = solution.getNumberOfVariables();

                    Eigen::VectorXd     inverted_H;
                    inverted_H.resize(num_var);
                    for (EigenIndex i = 0; i < inverted_H.size(); ++i)
                    {
                        inverted_H(i) = 1./(qp_problem_.getHessian()(i,i) + parameters_.elimination_regularization_factor_);
                    }

                    Eigen::VectorXd     iH_g = inverted_H.asDiagonal() * qp_problem_.getGradient();
                    Eigen::MatrixXd     iH_At = inverted_H.asDiagonal() * qp_problem_.getEqualities().getA().transpose();

                    solution.x_.noalias() = iH_At
                                            *
                                            // lambda
                                            (qp_problem_.getEqualities().getA() * iH_At).llt().solve(
                                                    qp_problem_.getEqualities().getA()*iH_g + qp_problem_.getEqualities().getB())
                                            -
                                            iH_g;

                    solution.return_status_ = SolverStatus::OK;
                }


                /**
                 * @brief Solve KKT system directly
                 *
                 * @param[out] solution
                 * @param[in] hierarchy
                 */
                void solveDirect(   humoto::Solution &solution,
                                    const humoto::OptimizationProblem &hierarchy)
                {
                    Eigen::MatrixXd   kkt_matrix;
                    Eigen::VectorXd   kkt_vector;

                    std::ptrdiff_t  num_var = solution.getNumberOfVariables();
                    std::ptrdiff_t  num_ctr = qp_problem_.getEqualities().getNumberOfConstraints();

                    kkt_matrix.resize(num_var+num_ctr, num_var+num_ctr);
                    kkt_vector.resize(num_var+num_ctr);

                    kkt_matrix <<   qp_problem_.getHessian(), qp_problem_.getEqualities().getA().transpose(),
                                    qp_problem_.getEqualities().getA(), Eigen::MatrixXd::Zero(num_ctr, num_ctr);

                    kkt_vector <<   -qp_problem_.getGradient(),
                                    qp_problem_.getEqualities().getB();


                    Eigen::VectorXd x;
                    switch(parameters_.solution_method_)
                    {
                        case SolverParameters::LU:
                            x = kkt_matrix.lu().solve(kkt_vector);
                            break;
                        case SolverParameters::QR:
                            x = kkt_matrix.colPivHouseholderQr().solve(kkt_vector);
                            break;
                        default:
                            HUMOTO_THROW_MSG("Unknown solution method.");
                    }

                    solution.return_status_ = SolverStatus::OK;

                    solution.x_ = x.segment(0, num_var);
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
                explicit Solver(const SolverParameters &parameters)
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
                            const std::string &name = "kktsolver") const
                {
                    LogEntryName subname = parent; subname.add(name);
                    qp_problem_.log(logger, subname);
                }
        };
    }
}
