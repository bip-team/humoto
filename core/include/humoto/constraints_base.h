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
    namespace constraints
    {
        /**
         * @brief Constraint type
         */
        class ConstraintType
        {
            public:
                enum Type
                {
                    UNDEFINED           = 0,
                    ILU                 = 1,    // L <= x[I] <= U
                    IL                  = 2,    // L <= x[I]
                    IU                  = 3,    //      x[I] <= U
                    IB                  = 4,    //      x[I] = B
                    IB0                 = 5,    //      x[I] = 0
                    GILU                = 6,    // L <= diag(G) * x[I] <= U
                    GIL                 = 7,    // L <= diag(G) * x[I]
                    GIU                 = 8,    //      diag(G) * x[I] <= U
                    GIB                 = 9,    //      diag(G) * x[I] = B
                    GIB0                = 10,   //      diag(G) * x[I] = 0
                    ALU                 = 11,   // L <= A * x <= U
                    AL                  = 12,   // L <= A * x
                    AU                  = 13,   //      A * x <= U
                    AB                  = 14,   //      A * x = B
                    AB0                 = 15,   //      A * x = 0
                    ASLU                = 16,   // L <= A * S * x <= U
                    ASL                 = 17,   // L <= A * S * x
                    ASU                 = 18,   //      A * S * x <= U
                    ASB                 = 19,   //      A * S * x = B
                    ASB0                = 20    //      A * S * x = 0
                };


                static bool isEquality(const Type ctr_type)
                {
                    switch(ctr_type)
                    {
                        case IB:
                        case IB0:
                        case GIB:
                        case GIB0:
                        case AB:
                        case AB0:
                        case ASB:
                        case ASB0:
                            return(true);
                        case UNDEFINED:
                            HUMOTO_THROW_MSG("Uninitialized constraint type.");
                        default:
                            return(false);
                    }
                }

                static bool isSimple(const Type ctr_type)
                {
                    switch(ctr_type)
                    {
                        case ILU:
                        case IL:
                        case IU:
                        case IB:
                        case IB0:
                        case GILU:
                        case GIL:
                        case GIU:
                        case GIB:
                        case GIB0:
                            return(true);
                        case UNDEFINED:
                            HUMOTO_THROW_MSG("Uninitialized constraint type.");
                        default:
                            return(false);
                    }
                }


                static bool isTwoSidedInequality(const Type ctr_type)
                {
                    switch(ctr_type)
                    {
                        case ILU:
                        case GILU:
                        case ALU:
                        case ASLU:
                            return(true);
                        case UNDEFINED:
                            HUMOTO_THROW_MSG("Uninitialized constraint type.");
                        default:
                            return(false);
                    }
                }
        };


        // ======================================================================================
        // ======================================================================================
        // ======================================================================================


        /**
         * @brief Constraints abstract interface class
         */
        class HUMOTO_LOCAL ContainerBase
        {
            protected:
                std::size_t  number_of_variables_;


            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ContainerBase() {}


                ContainerBase()
                {
                    number_of_variables_ = 0;
                }


                /**
                 * @brief Set number of variables
                 *
                 * @param[in] number_of_variables number of variables
                 */
                void setNumberOfVariables(const std::size_t number_of_variables)
                {
                    number_of_variables_ = number_of_variables;
                }


                /**
                 * @brief Reset body of constraints (matrix A or I)
                 *
                 * @param[in] number_of_constraints     number of constraints
                 * @param[in] initialize_matrices       initialize matrices using defaults
                 */
                virtual void resetBody( const std::size_t  number_of_constraints = 0,
                                        const bool initialize_matrices = false) = 0;


                /**
                 * @brief Reset bounds of constraints (b, lb, ub)
                 *
                 * @param[in] number_of_constraints     number of constraints
                 * @param[in] initialize_matrices       initialize matrices using defaults
                 */
                virtual void resetBounds(   const std::size_t  number_of_constraints = 0,
                                            const bool initialize_matrices = false) = 0;


                /**
                 * @brief Initialize constraints
                 *
                 * @param[in] number_of_constraints     number of constraints
                 * @param[in] number_of_variables       number of variables
                 * @param[in] initialize_matrices       initialize matrices using defaults
                 */
                void reset (const std::size_t  number_of_constraints = 0,
                            const std::size_t  number_of_variables = 0,
                            const bool initialize_matrices = false)
                {
                    setNumberOfVariables(number_of_variables);

                    resetBody(number_of_constraints, initialize_matrices);
                    resetBounds(number_of_constraints, initialize_matrices);
                }


                /**
                 * @brief Log body.
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                virtual void    logBody(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                        const LogEntryName &parent = LogEntryName(),
                                        const std::string &name = "constraints") const = 0;

                /**
                 * @brief Log bounds.
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                virtual void    logBounds(  humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                            const LogEntryName &parent = LogEntryName(),
                                            const std::string  &name = "constraints") const = 0;


            public:
                /**
                 * @brief Returns number of variables in the task.
                 *
                 * @return number of variables
                 */
                std::size_t getNumberOfVariables() const
                {
                    return (number_of_variables_);
                }



                /**
                 * @brief Returns number of constraints in the task.
                 *
                 * @return number of constraints
                 */
                virtual std::size_t getNumberOfConstraints() const = 0;



                /**
                 * @brief Log data.
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void    log(  humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                              const LogEntryName &parent = LogEntryName(),
                              const std::string &name = "constraints") const
                {
                    LogEntryName subname = parent;
                    subname.add(name);

                    logger.log(LogEntryName(subname).add("number_of_variables"), number_of_variables_);

                    logBody(logger, subname, "");
                    logBounds(logger, subname, "");
                }


                /**
                 * @brief Computes location of the copied constraints in a
                 * container and verifies their consistency.
                 *
                 * @param[in] number_of_constraints number of added constraints
                 * @param[in] constraints_offset    offset in the container
                 * @param[in] number_of_variables   number of variables in the container
                 *
                 * @return Location
                 */
                Location    getCopyLocation(const std::size_t number_of_constraints,
                                            const std::size_t constraints_offset,
                                            const std::size_t number_of_variables) const
                {
                    Location ctr_location(constraints_offset, number_of_constraints);

                    HUMOTO_ASSERT(  ctr_location.checkLength(getNumberOfConstraints()),
                                    "Added constraints exceed preallocated storage space.");
                    HUMOTO_ASSERT(  getNumberOfVariables() == number_of_variables,
                                    "Constraints have mismatching number of variables.")

                    return (ctr_location);
                }
        };



        /**
         * @brief Container for general inequality constraints.
         */
        class HUMOTO_LOCAL ContainerALU : public BodyAMixin< BoundsLUMixin <ContainerBase> >
        {
            public:
                using  ContainerBase::reset;
        };


        /**
         * @brief Container for general onesided inequality constraints.
         */
        class HUMOTO_LOCAL ContainerAL : public BodyAMixin< BoundsLMixin <ContainerBase> >
        {
            public:
                using  ContainerBase::reset;
        };


        /**
         * @brief Container for general equality constraints.
         */
        class HUMOTO_LOCAL ContainerAB : public BodyAMixin< BoundsBMixin <ContainerBase> >
        {
            public:
                using  ContainerBase::reset;
        };


        /**
         * @brief Container for simple inequality constraints.
         */
        class HUMOTO_LOCAL ContainerILU : public BodyIMixin< BoundsLUMixin <ContainerBase> >
        {
            public:
                using  ContainerBase::reset;


                /**
                 * @brief Generates LB and UB, such that 'LB <= X <= UB'
                 *
                 * @param[out] lb
                 * @param[out] ub
                 */
                void initializeSolutionBounds(Eigen::VectorXd &lb, Eigen::VectorXd &ub) const
                {
                    if (getNumberOfConstraints() == 0)
                    {
                        lb.resize(0);
                        ub.resize(0);
                    }
                    else
                    {
                        lb.setConstant(number_of_variables_, -humoto::g_infinity);
                        ub.setConstant(number_of_variables_,  humoto::g_infinity);

                        for (std::size_t i = 0; i < getNumberOfConstraints(); ++i)
                        {
                            unsigned int    variable_index = getIndices()[i];

                            if (lb[variable_index] < getLowerBounds()[i])
                            {
                                lb[variable_index] = getLowerBounds()[i];
                            }

                            if (ub[variable_index] > getUpperBounds()[i])
                            {
                                ub[variable_index] = getUpperBounds()[i];
                            }

#ifndef DNDEBUG
                            if (lb[variable_index] > ub[variable_index])
                            {
                                std::stringstream err_msg;
                                err_msg << std::setprecision(std::numeric_limits<double>::digits10);
                                err_msg << "Inconsistent bounds (lb > ub): variable index = '" << variable_index
                                        << "', lb = '" << lb[variable_index]
                                        << "', ub = '" << ub[variable_index]
                                        << "'.";
                                HUMOTO_THROW_MSG(err_msg.str());
                            }
#endif
                        }
                    }
                }
        };


        // ======================================================================================
        // ======================================================================================
        // ======================================================================================


        /**
         * @brief Constraints abstract interface class
         */
        class HUMOTO_LOCAL ConstraintsBase : public ContainerBase
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsBase() {}


                ConstraintsBase()
                {
                    number_of_variables_ = 0;
                }


            public:
                /**
                 * @brief Copy bounds to given vectors
                 *
                 * @param[in] lb
                 * @param[in] ub
                 * @param[in] location offset and length
                 */
                virtual void copyBoundsTo(  Eigen::VectorXd & lb,
                                            Eigen::VectorXd & ub,
                                            const Location & location) const = 0;

                /**
                 * @brief Copy body to the given matrix
                 *
                 * @param[in] A
                 * @param[in] location offset and length
                 */
                virtual void copyBodyTo(Eigen::MatrixXd & A,
                                        const Location & location) const = 0;


                /**
                 * @brief Copy negated body to the given matrix
                 *
                 * @param[in] A
                 * @param[in] location offset and length
                 */
                virtual void copyNegativeBodyTo(Eigen::MatrixXd & A,
                                                const Location & location) const = 0;


                /**
                 * @brief Compute 'A^T * A' for general equality constaints and
                 * save or add the result to H. Compute 'A^T * b' for general
                 * equality constaints and save or add the result to g.
                 *
                 * @param[in,out] H  left lower triangular part of the result.
                 * @param[in,out] g  result
                 *
                 * @attention Only the left lower triangular part of H is formed.
                 * Apply appropriate conversion with
                 * etools::convertLLTtoSymmetric() before using H.
                 */
                virtual void getATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_THROW_MSG("Operation not supported.");
                }


                /// @copydoc getATAandATb
                virtual void addATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_THROW_MSG("Operation not supported.");
                }


                /**
                 * @brief Set constraints.
                 *
                 * @param[in,out] container         new constraints
                 * @param[in] constraints_offset    first row offset
                 *
                 * @return offset of the end of added constraints
                 */
                virtual std::size_t copyTo( ContainerILU & container,
                                            const std::size_t constraints_offset) const
                {
                    HUMOTO_THROW_MSG("Operation not supported.");
                }


                /**
                 * @brief Copy constraints to a container
                 *
                 * @param[in,out] container
                 * @param[in] constraints_offset    first row offset
                 *
                 * @return offset of the end of added constraints
                 */
                virtual std::size_t copyTo( ContainerAL & container,
                                            const std::size_t constraints_offset) const
                {
                    HUMOTO_THROW_MSG("Operation not supported.");
                }


                /**
                 * @brief Copy constraints to a container
                 *
                 * @param[in,out] container
                 * @param[in] constraints_offset    first row offset
                 *
                 * @return offset of the end of added constraints
                 */
                virtual std::size_t copyTo( ContainerAB & container,
                                            const std::size_t constraints_offset) const
                {
                    HUMOTO_THROW_MSG("Operation not supported.");
                }


                /**
                 * @brief Copy constraints to a container
                 *
                 * @param[in,out] container
                 * @param[in] constraints_offset    first row offset
                 *
                 * @return offset of the end of added constraints
                 */
                virtual std::size_t copyTo( ContainerALU & container,
                                            const std::size_t constraints_offset) const
                {
                    HUMOTO_THROW_MSG("Operation not supported.");
                }


                /**
                 * @brief Returns type of the constraints
                 *
                 * @return type of the constraints
                 */
                virtual ConstraintType::Type getType() const = 0;



                /**
                 * @brief Check properties of the constriaints
                 *
                 * @return true/false
                 */
                bool    isEquality() const
                {
                    return (ConstraintType::isEquality(getType()));
                }

                /// @copydoc isEquality
                bool    isSimple() const
                {
                    return (ConstraintType::isSimple(getType()));
                }

                /// @copydoc isEquality
                bool    isTwoSidedInequality() const
                {
                    return (ConstraintType::isTwoSidedInequality(getType()));
                }


                /**
                 * @brief Check consistency of the constraints
                 */
                virtual void checkConsistency() const = 0;



                /**
                 * @brief Determine active set given a solution vector
                 *
                 * @param[in,out] active_set    active set
                 * @param[in] location          location of the constraints in the active set
                 * @param[in] solution          solution vector
                 */
                virtual void determineActiveSet(ActiveSetConstraints & active_set,
                                                const Location & location,
                                                const Solution & solution) const = 0;

                /**
                 * @brief Compute violations given a solution vector
                 *
                 * @param[in,out] violations    violations
                 * @param[in] location          location of the constraints in the active set
                 * @param[in] solution          solution vector
                 */
                virtual void computeViolations( ViolationsConstraints & violations,
                                                const Location & location,
                                                const Solution & solution) const = 0;
        };


        // ======================================================================================
        // ======================================================================================
        // ======================================================================================


        /**
         * @defgroup ConstraintsToContainerCopy Copying of constraints to containers
         *
         * @ingroup ConstraintsImplementation
         *
         * A set of mixins which implement functionality for copying of
         * constraints to containers.
         */

        /**
         * @addtogroup ConstraintsToContainerCopy
         * @{
         */

        /**
         * @brief This mixin provides functionality for conversion of specific type
         * of constraints to a certain container.
         *
         * @tparam t_Base Base class: @ref humoto::constraints::ConstraintsBase or
         * @ref humoto::TaskBase wrapped with other mixins.
         *
         */
        template<class t_Base>
            class HUMOTO_LOCAL CopyAnyToALUMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~CopyAnyToALUMixin() {}
                CopyAnyToALUMixin() {}

            public:
                using   t_Base::copyTo;


                /// @copydoc humoto::constraints::ConstraintsBase::copyTo
                std::size_t copyTo( ContainerALU & container,
                                    const std::size_t constraints_offset) const
                {
                    Location ctr_location = container.getCopyLocation(  t_Base::getNumberOfConstraints(),
                                                                        constraints_offset,
                                                                        container.getNumberOfVariables());

                    t_Base::copyBoundsTo(container.getLowerBounds(), container.getUpperBounds(), ctr_location);
                    t_Base::copyBodyTo(container.getA(), ctr_location);

                    return (ctr_location.offset_ + ctr_location.length_);
                }
        };



        /// @copydoc CopyAnyToALUMixin
        template<class t_Base>
            class HUMOTO_LOCAL CopyLowerInequalityToALMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~CopyLowerInequalityToALMixin() {}
                CopyLowerInequalityToALMixin() {}

            public:
                using   t_Base::copyTo;


                /// @copydoc humoto::constraints::ConstraintsBase::copyTo
                virtual std::size_t copyTo( ContainerAL & container,
                                            const std::size_t constraints_offset) const
                {
                    Location ctr_location = container.getCopyLocation(  t_Base::getNumberOfConstraints(),
                                                                        constraints_offset,
                                                                        container.getNumberOfVariables());

                    container.getLowerBounds().segment(ctr_location.offset_, ctr_location.length_) = t_Base::getLowerBounds();
                    t_Base::copyBodyTo(container.getA(), ctr_location);

                    return (ctr_location.offset_ + ctr_location.length_);
                }
        };


        /// @copydoc CopyAnyToALUMixin
        template<class t_Base>
            class HUMOTO_LOCAL CopyUpperInequalityToALMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~CopyUpperInequalityToALMixin() {}
                CopyUpperInequalityToALMixin() {}

            public:
                using   t_Base::copyTo;


                /// @copydoc humoto::constraints::ConstraintsBase::copyTo
                virtual std::size_t copyTo( ContainerAL & container,
                                            const std::size_t constraints_offset) const
                {
                    Location ctr_location = container.getCopyLocation(  t_Base::getNumberOfConstraints(),
                                                                        constraints_offset,
                                                                        container.getNumberOfVariables());

                    container.getLowerBounds().segment(ctr_location.offset_, ctr_location.length_) = -t_Base::getUpperBounds();
                    t_Base::copyNegativeBodyTo(container.getA(), ctr_location);

                    return (ctr_location.offset_ + ctr_location.length_);
                }
        };


        /// @copydoc CopyAnyToALUMixin
        template<class t_Base>
            class HUMOTO_LOCAL CopyTwoSidedInequalityToALMixin
            : public CopyUpperInequalityToALMixin< CopyLowerInequalityToALMixin<t_Base> >
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~CopyTwoSidedInequalityToALMixin() {}
                CopyTwoSidedInequalityToALMixin() {}

            public:
                using   t_Base::copyTo;


                /// @copydoc humoto::constraints::ConstraintsBase::copyTo
                std::size_t copyTo( ContainerAL & container,
                                    const std::size_t constraints_offset) const
                {
                    return(
                            CopyUpperInequalityToALMixin< CopyLowerInequalityToALMixin<t_Base> >::copyTo(
                                container,
                                CopyLowerInequalityToALMixin<t_Base>::copyTo(  container,
                                                                                    constraints_offset))
                          );
                }
        };


        /// @copydoc CopyAnyToALUMixin
        template<class t_Base>
            class HUMOTO_LOCAL CopySimpleToILUMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~CopySimpleToILUMixin() {}
                CopySimpleToILUMixin() {}

            public:
                using   t_Base::copyTo;


                /// @copydoc humoto::constraints::ConstraintsBase::copyTo
                std::size_t copyTo( ContainerILU & container,
                                    const std::size_t constraints_offset) const
                {
                    Location ctr_location = container.getCopyLocation(  t_Base::getNumberOfConstraints(),
                                                                        constraints_offset,
                                                                        container.getNumberOfVariables());

                    t_Base::copyBoundsTo(container.getLowerBounds(), container.getUpperBounds(), ctr_location);

                    // copy indices or A
                    container.getIndices().segment(ctr_location.offset_, ctr_location.length_) = t_Base::getIndices();

                    return (ctr_location.offset_ + ctr_location.length_);
                }
        };


        /// @copydoc CopyAnyToALUMixin
        template<class t_Base>
            class HUMOTO_LOCAL CopyEqualityToABMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~CopyEqualityToABMixin() {}
                CopyEqualityToABMixin() {}

            public:
                using   t_Base::copyTo;


                /// @copydoc humoto::constraints::ConstraintsBase::copyTo
                std::size_t copyTo( ContainerAB & container,
                                    const std::size_t constraints_offset) const
                {
                    Location ctr_location = container.getCopyLocation(  t_Base::getNumberOfConstraints(),
                                                                        constraints_offset,
                                                                        container.getNumberOfVariables());

                    t_Base::copyEqualityBoundsTo(container.getB(), ctr_location);
                    t_Base::copyBodyTo(container.getA(), ctr_location);

                    return (ctr_location.offset_ + ctr_location.length_);
                }
        };
        /// @}
    }
}


