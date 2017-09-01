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
     * @brief A container for a QP Objective.
     */
    class HUMOTO_LOCAL QPObjective
    {
        public:
            enum Status
            {
                UNDEFINED = 0,
                INITIALIZED = 1,
                MODIFIED = 2
            };

            enum HessianType
            {
                HESSIAN_UNDEFINED = 0,
                HESSIAN_GENERIC = 1,
                HESSIAN_DIAGONAL = 2,
                HESSIAN_LOWER_TRIANGULAR = 3
            };


        protected:
            /// Hessian
            Eigen::MatrixXd     H_;

            /// Gradient vector
            Eigen::VectorXd     g_;

            HessianType         hessian_type_;
            Status              status_;


        public:
            /**
             * @brief Generates (general) objective containing all constraints on this level
             *
             * @param[in] hlevel            hierarchy level
             * @param[in] initialize_upper_triangular_part
             */
            void initializeObjective(   const humoto::HierarchyLevel    &hlevel,
                                        const bool initialize_upper_triangular_part = true)
            {
                HUMOTO_ASSERT(hlevel.isEquality(), "Objective contains inequality constraints.");
                HUMOTO_ASSERT(hlevel.getNumberOfConstraints() > 0, "Empty objective.");


                hlevel.getObjective(H_, g_);
                g_ = -g_;


                if (hlevel.isSimple())
                {
                    if (initialize_upper_triangular_part)
                    {
                        etools::convertLLTtoSymmetric(H_);
                    }

                    hessian_type_ = HESSIAN_DIAGONAL;
                }
                else
                {
                    if (initialize_upper_triangular_part)
                    {
                        etools::convertLLTtoSymmetric(H_);
                        hessian_type_ = HESSIAN_GENERIC;
                    }
                    else
                    {
                        hessian_type_ = HESSIAN_LOWER_TRIANGULAR;
                    }
                }

                status_ = INITIALIZED;
            }


            /**
             * @brief Const accessor
             *
             * @return objective component
             */
            const Eigen::MatrixXd   & getHessian() const
            {
                return (H_);
            }

            /**
             * @brief Const accessor
             *
             * @return objective component
             */
            const Eigen::VectorXd   & getGradient() const
            {
                return (g_);
            }



            /**
             * @brief Non-const accessor
             *
             * @return objective component
             */
            Eigen::MatrixXd   & getHessian()
            {
                hessian_type_ = HESSIAN_GENERIC;
                status_ = MODIFIED;
                return (H_);
            }

            /**
             * @brief Non-const accessor
             *
             * @return objective component
             */
            Eigen::VectorXd    & getGradient()
            {
                status_ = MODIFIED;
                return (g_);
            }



            /**
             * @brief Status of the objective.
             *
             * @return status
             */
            Status getStatus() const
            {
                return (status_);
            }



            /**
             * @brief Hessian type
             *
             * @return Hessian type
             */
            HessianType getHessianType() const
            {
                return (hessian_type_);
            }


            /**
             * @brief Adds regularization to the Hessian.
             *
             * @param[in] regularization_factor
             */
            void regularize(const double regularization_factor)
            {
                status_ = MODIFIED;

                for (EigenIndex i = 0; i < H_.rows(); ++i)
                {
                    H_(i,i) += regularization_factor;
                }
            }


            /**
             * @brief Log objective
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                        const LogEntryName &parent = LogEntryName(),
                        const std::string &name = "objective") const
            {
                LogEntryName subname = parent; subname.add(name);

                logger.log(LogEntryName(subname).add("H"), H_);
                logger.log(LogEntryName(subname).add("g"), g_);

                logger.log(LogEntryName(subname).add("hessian_type"), hessian_type_);
                logger.log(LogEntryName(subname).add("status"), status_);
            }
    };


    /**
     * @brief QP constraints container
     */
    class QPConstraints_ILU_ALU
    {
        public:
            humoto::constraints::ContainerALU     general_constraints_;
            humoto::constraints::ContainerILU     bounds_;

            //@{
            /// Simple bounds on variables
            Eigen::VectorXd     lb_;
            Eigen::VectorXd     ub_;
            //@}



        public:
            /**
             * @brief Generates (general) objective containing all constraints on this level
             *
             * @param[in] hlevel            hierarchy level
             * @param[in] sol_structure     structure of the solution
             */
            void initializeConstraints( const HierarchyLevel  &hlevel,
                                        const humoto::SolutionStructure & sol_structure)
            {
                hlevel.getGeneralConstraints(general_constraints_, sol_structure);

                hlevel.getSimpleConstraints(bounds_, sol_structure);
                bounds_.initializeSolutionBounds(lb_, ub_);
            }


            /**
             * @brief get container with simple bounds
             *
             * @return constraints container
             */
            const humoto::constraints::ContainerILU & getSimpleBounds() const
            {
                return(bounds_);
            }

            /**
             * @brief get container with general constraints
             *
             * @return constraints container
             */
            const humoto::constraints::ContainerALU & getGeneralConstraints() const
            {
                return(general_constraints_);
            }

            /**
             * @brief Get LB vector: 'LB <= X'
             *
             * @return  LB vector
             */
            const Eigen::VectorXd & getSolutionLowerBounds() const
            {
                return(lb_);
            }

            /**
             * @brief Get UB vector: 'X <= UB'
             *
             * @return  UB vector
             */
            const Eigen::VectorXd & getSolutionUpperBounds() const
            {
                return(ub_);
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
                        const std::string &name = "constraints") const
            {
                LogEntryName subname = parent; subname.add(name);

                bounds_.log(logger, subname, "bounds");
                general_constraints_.log(logger, subname, "general_constraints");

                logger.log(LogEntryName(subname).add("lb"), lb_);
                logger.log(LogEntryName(subname).add("ub"), ub_);
            }
    };



    /**
     * @brief QP constraints container
     */
    class QPConstraints_AB_AL
    {
        protected:
            humoto::constraints::ContainerAL     inequality_constraints_;
            humoto::constraints::ContainerAB     equality_constraints_;


        public:
            /**
             * @brief Generates (general) objective containing all constraints on this level
             *
             * @param[in] hlevel            hierarchy level
             * @param[in] sol_structure     structure of the solution
             */
            void initializeConstraints( const HierarchyLevel  &hlevel,
                                        const humoto::SolutionStructure & sol_structure)
            {
                hlevel.getEqualityConstraints(equality_constraints_, sol_structure);
                hlevel.getInequalityConstraints(inequality_constraints_, sol_structure);
            }


            /**
             * @brief get container with inequalities
             *
             * @return constraints container
             */
            const humoto::constraints::ContainerAL & getInequalities() const
            {
                return(inequality_constraints_);
            }

            /**
             * @brief get container with equalities
             *
             * @return constraints container
             */
            const humoto::constraints::ContainerAB & getEqualities() const
            {
                return(equality_constraints_);
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
                        const std::string &name = "constraints") const
            {
                LogEntryName subname = parent; subname.add(name);

                equality_constraints_.log(logger, subname, "equality_constraints");
                inequality_constraints_.log(logger, subname, "inequality_constraints");
            }
    };


    /**
     * @brief QP constraints container
     */
    class QPConstraints_AB
    {
        protected:
            humoto::constraints::ContainerAB     equality_constraints_;


        public:
            /**
             * @brief Generates (general) objective containing all constraints on this level
             *
             * @param[in] hlevel            hierarchy level
             * @param[in] sol_structure     structure of the solution
             */
            void initializeConstraints( const HierarchyLevel  &hlevel,
                                        const humoto::SolutionStructure & sol_structure)
            {
                HUMOTO_ASSERT(hlevel.isEquality(), "Inequality constraints are not supported.");

                hlevel.getEqualityConstraints(equality_constraints_, sol_structure);
            }


            /**
             * @brief get container with equalities
             *
             * @return constraints container
             */
            const humoto::constraints::ContainerAB & getEqualities() const
            {
                return(equality_constraints_);
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
                        const std::string &name = "constraints") const
            {
                LogEntryName subname = parent; subname.add(name);

                equality_constraints_.log(logger, subname, "equality_constraints");
            }
    };


    /**
     * @brief QP problem base class
     *
     * @tparam t_Constraints    Constraints container type.
     */
    template<class t_Constraints>
        class QPProblemBase : public QPObjective, public t_Constraints
    {
        public:
            /**
             * @brief Log a QP problem
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                        const LogEntryName &parent = LogEntryName(),
                        const std::string &name = "qp_problem") const
            {
                LogEntryName subname = parent; subname.add(name);

                QPObjective::log(logger, subname);
                t_Constraints::log(logger, subname);
            }
    };



    /**
     * @brief A QP problem with constraints of a specific type
     */
    class QPProblem_ILU_ALU : public QPProblemBase<QPConstraints_ILU_ALU>
    {
    };

    /**
     * @brief A QP problem with constraints of a specific type
     */
    class QPProblem_AB_AL : public QPProblemBase<QPConstraints_AB_AL>
    {
    };

    /**
     * @brief A QP problem with constraints of a specific type
     */
    class QPProblem_AB : public QPProblemBase<QPConstraints_AB>
    {
    };
}
