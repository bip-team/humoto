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
         * @defgroup ConstraintsBounds Implementation of bounds
         *
         * @ingroup ConstraintsImplementation
         *
         * A set of mixins which implement bounds of constraints.
         */

        /**
         * @addtogroup ConstraintsBounds
         * @{
         */

        /**
         * @brief Mixin representing vector 'b = 0' in an equality constraint.
         *
         * @tparam t_Base Base class: @ref humoto::constraints::ConstraintsBase or
         * @ref humoto::TaskBase wrapped with other mixins.
         */
        template <class t_Base>
            class HUMOTO_LOCAL BoundsB0Mixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~BoundsB0Mixin() {}
                BoundsB0Mixin() {}


                /// @copydoc humoto::constraints::ContainerBase::resetBounds
                void resetBounds (  const std::size_t  number_of_constraints = 0,
                                    const bool initialize_matrices = false)
                {
                    // nothing to do
                }


                /// @copydoc humoto::constraints::ContainerBase::logBounds
                void    logBounds(  humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                    const LogEntryName &parent = LogEntryName(),
                                    const std::string &name = "constraints") const
                {
                    // nothing to do
                }


                /**
                 * @brief Compute 'A^T * b' for general equality constaints and
                 * save or add the result to g.
                 *
                 * @param[in,out] g  result
                 */
                void getATb(Eigen::VectorXd &g) const
                {
                    g.setZero(t_Base::getNumberOfVariables());
                }


                /// @copydoc getATb
                void addATb(Eigen::VectorXd &g) const
                {
                    // nothing to do
                }


                /**
                 * @brief Copy b
                 *
                 * @param[in,out] b
                 * @param[in] location
                 */
                void copyEqualityBoundsTo(  Eigen::VectorXd & b,
                                            const Location & location) const
                {
                    b.segment(location.offset_, location.length_).setZero();
                }


            public:
                /// @copydoc humoto::constraints::ConstraintsBase::copyBoundsTo
                void copyBoundsTo(  Eigen::VectorXd & lb,
                                    Eigen::VectorXd & ub,
                                    const Location & location) const
                {
                    lb.segment(location.offset_, location.length_).setZero();
                    ub.segment(location.offset_, location.length_).setZero();
                }
        };


        /**
         * @brief Mixin representing vector 'b' in an equality constraint.
         *
         * @tparam t_Base Base class: @ref humoto::constraints::ConstraintsBase or
         * @ref humoto::TaskBase wrapped with other mixins.
         */
        template <class t_Base>
            class HUMOTO_LOCAL BoundsBMixin : public t_Base
        {
            protected:
                Eigen::VectorXd     b_;


            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~BoundsBMixin() {}
                BoundsBMixin() {}


                /**
                 * @copydoc humoto::constraints::ContainerBase::resetBounds
                 * @note Initialization: b is filled with zeros
                 */
                void resetBounds (  const std::size_t  number_of_constraints = 0,
                                    const bool initialize_matrices = false)
                {
                    if (initialize_matrices)
                    {
                        b_.setZero(number_of_constraints);
                    }
                    else
                    {
                        b_.resize(number_of_constraints);
                    }
                }


                /// @copydoc humoto::constraints::ContainerBase::logBounds
                void    logBounds(  humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                    const LogEntryName &parent = LogEntryName(),
                                    const std::string &name = "constraints") const
                {
                    logger.log(LogEntryName(parent).add(name).add("b"), b_);
                }


                /**
                 * @brief Copy b
                 *
                 * @param[in,out] b
                 * @param[in] location
                 */
                void copyEqualityBoundsTo(  Eigen::VectorXd & b,
                                            const Location & location) const
                {
                    b.segment(location.offset_, location.length_) = b_;
                }


            public:
                /**
                 * @brief Get vector b from equalities: 'A*x = b' or 'x = b'.
                 *
                 * @return Vector b.
                 */
                Eigen::VectorXd &  getB()
                {
                    return (b_);
                }


                /// @copydoc getB
                const Eigen::VectorXd &  getB() const
                {
                    return (b_);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::copyBoundsTo
                void copyBoundsTo(  Eigen::VectorXd & lb,
                                    Eigen::VectorXd & ub,
                                    const Location & location) const
                {
                    lb.segment(location.offset_, location.length_) = b_;
                    ub.segment(location.offset_, location.length_) = b_;
                }
        };


        /**
         * @brief Mixin representing lower bound in an inequality constraint.
         *
         * @tparam t_Base Base class: @ref humoto::constraints::ConstraintsBase or
         * @ref humoto::TaskBase wrapped with other mixins.
         */
        template <class t_Base>
            class HUMOTO_LOCAL BoundsLMixin : public t_Base
        {
            protected:
                Eigen::VectorXd     lb_;


            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~BoundsLMixin() {}
                BoundsLMixin() {}


                /**
                 * @copydoc humoto::constraints::ContainerBase::resetBounds
                 * @note Initialization: lb is set to -humoto::g_infinity
                 */
                void resetBounds (const std::size_t number_of_constraints, const bool initialize_matrices = false)
                {
                    if (initialize_matrices)
                    {
                        lb_.setConstant(number_of_constraints, -humoto::g_infinity);
                    }
                    else
                    {
                        lb_.resize(number_of_constraints);
                    }
                }


                /// @copydoc humoto::constraints::ContainerBase::logBounds
                void    logBounds(  humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                    const LogEntryName &parent = LogEntryName(),
                                    const std::string &name = "constraints") const
                {
                    logger.log(LogEntryName(parent).add(name).add("lb"), lb_);
                }


            public:
                /**
                 * @brief Get lower bounds (lb/ub vectors from 'lb <= A*x <= ub').
                 *
                 * @return lower bounds.
                 */
                Eigen::VectorXd &  getLowerBounds()
                {
                    return (lb_);
                }

                /// @copydoc getLowerBounds
                const Eigen::VectorXd &  getLowerBounds() const
                {
                    return (lb_);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::copyBoundsTo
                void copyBoundsTo(  Eigen::VectorXd & lb,
                                    Eigen::VectorXd & ub,
                                    const Location & location) const
                {
                    lb.segment(location.offset_, location.length_) = lb_;
                    ub.segment(location.offset_, location.length_).setConstant(humoto::g_infinity);
                }
        };



        /**
         * @brief Mixin representing upper bound in an inequality constraint.
         *
         * @tparam t_Base Base class: @ref humoto::constraints::ConstraintsBase or
         * @ref humoto::TaskBase wrapped with other mixins.
         */
        template <class t_Base>
            class HUMOTO_LOCAL BoundsUMixin : public t_Base
        {
            protected:
                Eigen::VectorXd     ub_;


            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~BoundsUMixin() {}
                BoundsUMixin() {}


                /**
                 * @copydoc humoto::constraints::ContainerBase::resetBounds
                 * @note Initialization: ub is set to humoto::g_infinity
                 */
                void resetBounds (const std::size_t number_of_constraints, const bool initialize_matrices = false)
                {
                    if (initialize_matrices)
                    {
                        ub_.setConstant(number_of_constraints,  humoto::g_infinity);
                    }
                    else
                    {
                        ub_.resize(number_of_constraints);
                    }
                }


                /// @copydoc humoto::constraints::ContainerBase::logBounds
                void    logBounds(  humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                    const LogEntryName &parent = LogEntryName(),
                                    const std::string &name = "constraints") const
                {
                    logger.log(LogEntryName(parent).add(name).add("ub"), ub_);
                }



            public:
                /**
                 * @brief Get upper bounds (ub vectors from 'A*x <= ub').
                 *
                 * @return upper bounds.
                 */
                Eigen::VectorXd &  getUpperBounds()
                {
                    return (ub_);
                }


                /// @copydoc getUpperBounds
                const Eigen::VectorXd &  getUpperBounds() const
                {
                    return (ub_);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::copyBoundsTo
                void copyBoundsTo(  Eigen::VectorXd & lb,
                                    Eigen::VectorXd & ub,
                                    const Location & location) const
                {
                    lb.segment(location.offset_, location.length_).setConstant(-humoto::g_infinity);
                    ub.segment(location.offset_, location.length_) = ub_;
                }
        };



        /**
         * @brief Mixin representing upper bound in an inequality constraint.
         *
         * @tparam t_Base Base class: @ref humoto::constraints::ConstraintsBase or
         * @ref humoto::TaskBase wrapped with other mixins.
         */
        template <class t_Base>
            class HUMOTO_LOCAL BoundsLUMixin : public BoundsLMixin< BoundsUMixin <t_Base> >
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~BoundsLUMixin() {}
                BoundsLUMixin() {}


                /**
                 * @copydoc humoto::constraints::ContainerBase::resetBounds
                 * @note Initialization: lb is set to -humoto::g_infinity, ub is set to humoto::g_infinity
                 */
                void resetBounds (const std::size_t number_of_constraints, const bool initialize_matrices = false)
                {
                    BoundsLMixin< BoundsUMixin <t_Base> >::resetBounds(number_of_constraints, initialize_matrices);
                    BoundsUMixin <t_Base> ::resetBounds(number_of_constraints, initialize_matrices);
                }


                /// @copydoc humoto::constraints::ContainerBase::logBounds
                void    logBounds(  humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                    const LogEntryName &parent = LogEntryName(),
                                    const std::string &name = "constraints") const
                {
                    LogEntryName subname = parent;
                    subname.add(name);
                    BoundsLMixin< BoundsUMixin <t_Base> >::logBounds(logger, subname, "");
                    BoundsUMixin <t_Base> ::logBounds(logger, subname, "");
                }


            public:
                /// @copydoc humoto::constraints::ConstraintsBase::copyBoundsTo
                void copyBoundsTo(  Eigen::VectorXd & lb,
                                    Eigen::VectorXd & ub,
                                    const Location & location) const
                {
                    lb.segment(location.offset_, location.length_) =
                        BoundsLMixin< BoundsUMixin <t_Base> >::getLowerBounds();
                    ub.segment(location.offset_, location.length_) =
                        BoundsUMixin<t_Base>::getUpperBounds();
                }
        };
        /// @}


        // ======================================================================================
        // ======================================================================================
        // ======================================================================================

        /**
         * @defgroup ConstraintsBodies Implementation of bodies
         *
         * @ingroup ConstraintsImplementation
         *
         * A set of mixins which implement bodies of constraints.
         */

        /**
         * @addtogroup ConstraintsBodies
         * @{
         */

        /**
         * @brief Mixin representing matrix 'A' in a general constraint.
         *
         * @tparam t_Base Base class: @ref humoto::constraints::ConstraintsBase or
         * @ref humoto::TaskBase wrapped with other mixins.
         */
        template <class t_Base>
            class HUMOTO_LOCAL BodyAMixin : public t_Base
        {
            protected:
                Eigen::MatrixXd     A_;


            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~BodyAMixin() {}
                BodyAMixin() {}


                /**
                 * @brief Initialize A
                 *
                 * @param[in] number_of_constraints     number of constraints
                 * @param[in] initialize_matrices       initialize matrices using defaults
                 *
                 * Initialization:
                 *      - A is filled with zeros
                 */
                void resetBody (const std::size_t number_of_constraints, const bool initialize_matrices = false)
                {
                    if (initialize_matrices)
                    {
                        A_.setZero(number_of_constraints, t_Base::getNumberOfVariables());
                    }
                    else
                    {
                        A_.resize(number_of_constraints, t_Base::getNumberOfVariables());
                    }
                }


                /// @copydoc humoto::constraints::ContainerBase::logBody
                void    logBody(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                const LogEntryName &parent = LogEntryName(),
                                const std::string &name = "constraints") const
                {
                    logger.log(LogEntryName(parent).add(name).add("A"), A_);
                }



                /**
                 * @brief Compute 'A^T * A' for general equality constaints and
                 * save or add the result to H.
                 *
                 * @param[in,out] H  left lower triangular part of the result.
                 *
                 * @attention Only the left lower triangular part of H is formed.
                 * Apply appropriate conversion with
                 * etools::convertLLTtoSymmetric() before using H.
                 */
                void getATA(Eigen::MatrixXd &H) const
                {
                    //H.noalias() = A_.transpose()*A_;
                    etools::getATA(H, A_);
                }


                /// @copydoc humoto::constraints::BodyAMixin::getATA
                void addATA(Eigen::MatrixXd &H) const
                {
                    //H.noalias() += A_.transpose()*A_;
                    etools::addATA(H, A_);
                }


            public:
                // Access matrices
                /**
                 * @brief Get matrix A from general constraints: 'A*x = b', 'lb <= A*x <= ub'.
                 *
                 * @return matrix A
                 */
                Eigen::MatrixXd &  getA()
                {
                    return (A_);
                }

                /// @copydoc getA
                const Eigen::MatrixXd &  getA() const
                {
                    return (A_);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::copyBodyTo
                void copyBodyTo(Eigen::MatrixXd & A, const Location & location) const
                {
                    HUMOTO_ASSERT(  A_.cols() == A.cols(),
                                    "Numbers of columns in constraint matrices do not match.");

                    A.block(location.offset_, 0, location.length_, A_.cols()) = A_;
                }


                /// @copydoc humoto::constraints::ConstraintsBase::copyBodyTo
                void copyNegativeBodyTo(Eigen::MatrixXd & A, const Location & location) const
                {
                    HUMOTO_ASSERT(  A_.cols() == A.cols(),
                                    "Numbers of columns in constraint matrices do not match.");

                    A.block(location.offset_, 0, location.length_, A_.cols()) = -A_;
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getNumberOfConstraints
                std::size_t getNumberOfConstraints() const
                {
                    return (A_.rows());
                }


                /**
                 * @brief Compute A(i,:) * vector
                 *
                 * @param[in] i         row index
                 * @param[in] vector    vector
                 *
                 * @return result of multiplication
                 */
                double getProduct(  const std::size_t i,
                                    const Eigen::VectorXd & vector) const
                {
                    return(A_.row(i) * vector);
                }
        };



        /**
         * @brief Mixin representing matrix 'A*S' in a general constraint.
         *
         * @tparam t_Base Base class: @ref humoto::constraints::ConstraintsBase or
         * @ref humoto::TaskBase wrapped with other mixins.
         */
        template <class t_Base>
            class HUMOTO_LOCAL BodyASMixin : public t_Base
        {
            protected:
                Eigen::MatrixXd     A_;
                std::ptrdiff_t      offset_;


            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~BodyASMixin() {}
                BodyASMixin()
                {
                    offset_ = 0;
                }


                /**
                 * @brief Initialize A
                 *
                 * @param[in] number_of_constraints     number of constraints
                 * @param[in] initialize_matrices       initialize matrices using defaults
                 *
                 * Initialization:
                 *      - A is filled with zeros
                 */
                void resetBody (const std::size_t number_of_constraints, const bool initialize_matrices = false)
                {
                    offset_ = 0;
                    if (initialize_matrices)
                    {
                        A_.setZero(number_of_constraints, t_Base::getNumberOfVariables());
                    }
                    else
                    {
                        A_.resize(number_of_constraints, t_Base::getNumberOfVariables());
                    }
                }


                /// @copydoc humoto::constraints::ContainerBase::logBody
                void    logBody(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                const LogEntryName &parent = LogEntryName(),
                                const std::string &name = "constraints") const
                {
                    logger.log(LogEntryName(parent).add(name).add("A"), A_);
                    logger.log(LogEntryName(parent).add(name).add("offset"), offset_);
                }



                /// @copydoc humoto::constraints::BodyAMixin::getATA
                void getATA(Eigen::MatrixXd &H) const
                {
                    etools::getATA(H, A_, offset_, t_Base::getNumberOfVariables());
                }


                /// @copydoc humoto::constraints::BodyAMixin::getATA
                void addATA(Eigen::MatrixXd &H) const
                {
                    etools::addATA(H, A_, offset_);
                }


            public:
                // Access matrices
                /**
                 * @brief Get matrix A from general constraints: 'A*x = b', 'lb <= A*x <= ub'.
                 *
                 * @return matrix A
                 */
                Eigen::MatrixXd &  getA()
                {
                    return (A_);
                }

                /// @copydoc getA
                const Eigen::MatrixXd &  getA() const
                {
                    return (A_);
                }


                /**
                 * @brief Get offset of body part in the constriaints.
                 *
                 * @return offset
                 */
                std::ptrdiff_t getOffset() const
                {
                    return (offset_);
                }


                /**
                 * @brief Set offset of body part in the constriaints.
                 *
                 * @param[in] offset
                 */
                void setOffset(const std::ptrdiff_t offset)
                {
                    offset_ = offset;
                }


                /// @copydoc humoto::constraints::ConstraintsBase::copyBodyTo
                void copyBodyTo(Eigen::MatrixXd & A, const Location & location) const
                {
                    HUMOTO_ASSERT(  (t_Base::getNumberOfVariables() == static_cast<std::size_t>(A.cols()))
                                    && (offset_ + A_.cols() <= A.cols()),
                                    "Numbers of columns in constraint matrices do not match.");

                    std::ptrdiff_t right_space_size = t_Base::getNumberOfVariables() - offset_ - A_.cols();

                    A.middleRows(location.offset_, location.length_)
                        <<  Eigen::MatrixXd::Zero(location.length_, offset_),
                            A_,
                            Eigen::MatrixXd::Zero(location.length_, right_space_size);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::copyBodyTo
                void copyNegativeBodyTo(Eigen::MatrixXd & A, const Location & location) const
                {
                    HUMOTO_ASSERT(  (t_Base::getNumberOfVariables() == static_cast<std::size_t>(A.cols()))
                                    && (offset_ + A_.cols() <= A.cols()),
                                    "Numbers of columns in constraint matrices do not match.");

                    std::ptrdiff_t right_space_size = t_Base::getNumberOfVariables() - offset_ - A_.cols();

                    A.middleRows(location.offset_, location.length_)
                        <<  Eigen::MatrixXd::Zero(location.length_, offset_),
                            -A_,
                            Eigen::MatrixXd::Zero(location.length_, right_space_size);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getNumberOfConstraints
                std::size_t getNumberOfConstraints() const
                {
                    return (A_.rows());
                }


                /// @copydoc humoto::constraints::BodyAMixin::getProduct
                double getProduct(  const std::size_t i,
                                    const Eigen::VectorXd & vector) const
                {
                    return(A_.row(i) * vector.segment(offset_, A_.cols()));
                }
        };


        /**
         * @brief Mixin representing matrix of indices 'I' in a simple constraint.
         *
         * @tparam t_Base Base class: @ref humoto::constraints::ConstraintsBase or
         * @ref humoto::TaskBase wrapped with other mixins.
         */
        template <class t_Base>
            class HUMOTO_LOCAL BodyIMixin : public t_Base
        {
            protected:
                humoto::IndexVector I_; // vector of indices


            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~BodyIMixin() {}


                /**
                 * @brief Default constructor
                 */
                BodyIMixin() {}




                /**
                 * @brief Initialize I and gains
                 *
                 * @param[in] number_of_constraints     number of constraints
                 * @param[in] initialize_matrices       initialize matrices using defaults
                 */
                virtual void    resetBody(  const std::size_t  number_of_constraints = 0,
                                            const bool initialize_matrices = false)
                {
                    I_.resize(number_of_constraints);
                }



                /// @copydoc humoto::constraints::ContainerBase::logBody
                void    logBody(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                const LogEntryName &parent = LogEntryName(),
                                const std::string &name = "constraints") const
                {
                    logger.log(LogEntryName(parent).add(name).add("A"), I_);
                }


                /// @copydoc humoto::constraints::BodyAMixin::getATA
                void getATA(Eigen::MatrixXd &H) const
                {
                    //H.setZero(getNumberOfVariables(), getNumberOfVariables());
                    H.resize(getNumberOfVariables(), getNumberOfVariables());
                    H.triangularView<Eigen::Lower>().setZero();
                    addATA(H);
                }


                /// @copydoc humoto::constraints::BodyAMixin::getATA
                void addATA(Eigen::MatrixXd &H) const
                {
                    for (EigenIndex i = 0; i < I_.rows(); ++i)
                    {
                        H(I_[i], I_[i]) += 1;
                    }
                }


            public:
                using t_Base::getNumberOfVariables;



                /// @copydoc humoto::constraints::ConstraintsBase::copyBodyTo
                virtual void copyBodyTo(Eigen::MatrixXd & A, const Location & location) const
                {
                    HUMOTO_ASSERT(  static_cast<std::size_t>(A.cols()) == getNumberOfVariables(),
                                    "Wrong number of variables.");
                    HUMOTO_ASSERT(  static_cast<std::size_t>(I_.rows()) == location.length_,
                                    "Wrong number of constraints.");

                    A.block(location.offset_, 0, location.length_, getNumberOfVariables()).setZero();

                    for (std::size_t i = 0; i < location.length_; ++i)
                    {
                        A(location.offset_ + i, I_[i]) = 1.0;
                    }
                }


                /// @copydoc humoto::constraints::ConstraintsBase::copyBodyTo
                virtual void copyNegativeBodyTo(Eigen::MatrixXd & A, const Location & location) const
                {
                    HUMOTO_ASSERT(  static_cast<std::size_t>(A.cols()) == getNumberOfVariables(),
                                    "Wrong number of variables.");
                    HUMOTO_ASSERT(  static_cast<std::size_t>(I_.rows()) == location.length_,
                                    "Wrong number of constraints.");

                    A.block(location.offset_, 0, location.length_, getNumberOfVariables()).setZero();

                    for (std::size_t i = 0; i < location.length_; ++i)
                    {
                        A(location.offset_ + i, I_[i]) = -1.0;
                    }
                }


                /**
                 * @brief Get indices
                 *
                 * @return indices
                 */
                humoto::IndexVector &  getIndices()
                {
                    return (I_);
                }


                /// @copydoc getIndices
                const humoto::IndexVector &  getIndices() const
                {
                    return (I_);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getNumberOfConstraints
                std::size_t getNumberOfConstraints() const
                {
                    return (I_.rows());
                }


                /// @copydoc humoto::constraints::BodyAMixin::getProduct
                double getProduct(  const std::size_t i,
                                    const Eigen::VectorXd & vector) const
                {
                    return(vector(I_(i)));
                }
        };



        /**
         * @brief Mixin representing matrix of indices 'I' and matrix of gains 'G'
         * in a simple constraint.
         *
         * @tparam t_Base Base class: @ref humoto::constraints::ConstraintsBase or
         * @ref humoto::TaskBase wrapped with other mixins.
         */
        template <class t_Base>
            class HUMOTO_LOCAL BodyGIMixin : public BodyIMixin<t_Base>
        {
            protected:
                using BodyIMixin<t_Base>::I_;
                Eigen::VectorXd     I_gains_;


            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~BodyGIMixin() {}
                BodyGIMixin() {}


                /**
                 * @brief Initialize I and gains
                 *
                 * @param[in] number_of_constraints     number of constraints
                 * @param[in] initialize_matrices       initialize matrices using defaults
                 */
                void resetBody( const std::size_t  number_of_constraints = 0,
                                const bool initialize_matrices = false)
                {
                    BodyIMixin<t_Base>::resetBody(number_of_constraints, initialize_matrices);
                    I_gains_.setConstant(number_of_constraints, 1.0);
                }



                /// @copydoc humoto::constraints::ContainerBase::logBody
                void    logBody(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                const LogEntryName &parent = LogEntryName(),
                                const std::string &name = "constraints") const
                {
                    LogEntryName subname = parent;
                    subname.add(name);
                    BodyIMixin<t_Base>::logBody(logger, subname, "");
                    logger.log(LogEntryName(subname).add("Agains"), I_gains_);
                }



                /// @copydoc humoto::constraints::BodyAMixin::getATA
                void getATA(Eigen::MatrixXd &H) const
                {
                    //H.setZero(getNumberOfVariables(), getNumberOfVariables());
                    H.resize(getNumberOfVariables(), getNumberOfVariables());
                    H.triangularView<Eigen::Lower>().setZero();
                    addATA(H);
                }


                /// @copydoc humoto::constraints::BodyAMixin::getATA
                void addATA(Eigen::MatrixXd &H) const
                {
                    for (EigenIndex i = 0; i < I_.rows(); ++i)
                    {
                        H(I_[i], I_[i]) += I_gains_[i]*I_gains_[i];
                    }
                }


            public:
                using t_Base::getNumberOfVariables;


                /**
                 * @brief Get gains for each element in simple objective
                 *
                 * @return gains
                 */
                Eigen::VectorXd &  getIGains()
                {
                    return (I_gains_);
                }

                /// @copydoc getIGains
                const Eigen::VectorXd&  getIGains() const
                {
                    return (I_gains_);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::copyBodyTo
                void copyBodyTo(Eigen::MatrixXd & A, const Location & location) const
                {
                    HUMOTO_ASSERT(  static_cast<std::size_t>(A.cols()) == getNumberOfVariables(),
                                    "Wrong number of variables.");
                    HUMOTO_ASSERT(  static_cast<std::size_t>(I_.rows()) == location.length_,
                                    "Wrong number of constraints.");

                    A.block(location.offset_, 0, location.length_, getNumberOfVariables()).setZero();

                    for (std::size_t i = 0; i < location.length_; ++i)
                    {
                        A(location.offset_ + i, I_[i]) = I_gains_[i];
                    }
                }


                /// @copydoc humoto::constraints::ConstraintsBase::copyBodyTo
                void copyNegativeBodyTo(Eigen::MatrixXd & A, const Location & location) const
                {
                    HUMOTO_ASSERT(  static_cast<std::size_t>(A.cols()) == getNumberOfVariables(),
                                    "Wrong number of variables.");
                    HUMOTO_ASSERT(  static_cast<std::size_t>(I_.rows()) == location.length_,
                                    "Wrong number of constraints.");

                    A.block(location.offset_, 0, location.length_, getNumberOfVariables()).setZero();

                    for (std::size_t i = 0; i < location.length_; ++i)
                    {
                        A(location.offset_ + i, I_[i]) = -I_gains_[i];
                    }
                }


                /// @copydoc humoto::constraints::BodyAMixin::getProduct
                double getProduct(  const std::size_t i,
                                    const Eigen::VectorXd & vector) const
                {
                    return(I_gains_(i) * vector(I_(i)));
                }
        };
        /// @}


        // ======================================================================================
        // ======================================================================================
        // ======================================================================================

        /**
         * @defgroup ConstraintsActiveSetDetermination  Determination of active set
         *
         * @ingroup ConstraintsImplementation
         *
         * A set of mixins which implement determination of active set based on
         * the solution.
         */

        /**
         * @addtogroup ConstraintsActiveSetDetermination
         * @{
         */

        /**
         * @brief This mixin provides functionality for determination of active
         * constraints.
         *
         * @tparam t_Base base class
         */
        template <class t_Base>
            class HUMOTO_LOCAL ActiveSetDeterminationLUMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ActiveSetDeterminationLUMixin() {}
                ActiveSetDeterminationLUMixin() {}

            public:
                /// @copydoc humoto::constraints::ConstraintsBase::determineActiveSet
                void determineActiveSet(ActiveSetConstraints & active_set,
                                        const Location & location,
                                        const Solution & solution) const
                {
                    HUMOTO_ASSERT(  static_cast<std::size_t>(t_Base::getNumberOfConstraints()) == location.length_,
                                    std::string("Cannot determine active set: internal error."));

                    for(std::size_t i = 0; i < location.length_; ++i)
                    {
                        if (t_Base::getLowerBounds()[i] > t_Base::getUpperBounds()[i])
                        {
                            active_set[location.offset_ + i] = ConstraintActivationType::UNDEFINED;
                        }
                        else
                        {
                            if (t_Base::getLowerBounds()[i] == t_Base::getUpperBounds()[i])
                            {
                                active_set[location.offset_ + i] = ConstraintActivationType::EQUALITY;
                            }
                            else
                            {
                                double product = t_Base::getProduct(i, solution.get_x());

                                if (product <= t_Base::getLowerBounds()[i])
                                {
                                    active_set[location.offset_ + i] = ConstraintActivationType::LOWER_BOUND;
                                }
                                else
                                {
                                    if (product >= t_Base::getUpperBounds()[i])
                                    {
                                        active_set[location.offset_ + i] = ConstraintActivationType::UPPER_BOUND;
                                    }
                                    else
                                    {
                                        active_set[location.offset_ + i] = ConstraintActivationType::INACTIVE;
                                    }
                                }
                            }
                        }
                    }
                }
        };


        /// @copydoc ActiveSetDeterminationLUMixin
        template <class t_Base>
            class HUMOTO_LOCAL ActiveSetDeterminationLMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ActiveSetDeterminationLMixin() {}
                ActiveSetDeterminationLMixin() {}

            public:
                /// @copydoc humoto::constraints::ConstraintsBase::determineActiveSet
                void determineActiveSet(ActiveSetConstraints & active_set,
                                        const Location & location,
                                        const Solution & solution) const
                {
                    HUMOTO_ASSERT(  static_cast<std::size_t>(t_Base::getNumberOfConstraints()) == location.length_,
                                    std::string("Cannot determine active set: internal error."));

                    for(std::size_t i = 0; i < location.length_; ++i)
                    {
                        if (t_Base::getProduct(i, solution.get_x()) <= t_Base::getLowerBounds()[i])
                        {
                            active_set[location.offset_ + i] = ConstraintActivationType::LOWER_BOUND;
                        }
                        else
                        {
                            active_set[location.offset_ + i] = ConstraintActivationType::INACTIVE;
                        }
                    }
                }
        };


        /// @copydoc ActiveSetDeterminationLUMixin
        template <class t_Base>
            class HUMOTO_LOCAL ActiveSetDeterminationUMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ActiveSetDeterminationUMixin() {}
                ActiveSetDeterminationUMixin() {}

            public:
                /// @copydoc humoto::constraints::ConstraintsBase::determineActiveSet
                void determineActiveSet(ActiveSetConstraints & active_set,
                                        const Location & location,
                                        const Solution & solution) const
                {
                    HUMOTO_ASSERT(  static_cast<std::size_t>(t_Base::getNumberOfConstraints()) == location.length_,
                                    std::string("Cannot determine active set: internal error."));

                    for(std::size_t i = 0; i < location.length_; ++i)
                    {
                        if (t_Base::getProduct(i, solution.get_x()) >= t_Base::getUpperBounds()[i])
                        {
                            active_set[location.offset_ + i] = ConstraintActivationType::UPPER_BOUND;
                        }
                        else
                        {
                            active_set[location.offset_ + i] = ConstraintActivationType::INACTIVE;
                        }
                    }
                }
        };


        /// @copydoc ActiveSetDeterminationLUMixin
        template <class t_Base>
            class HUMOTO_LOCAL ActiveSetDeterminationBMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ActiveSetDeterminationBMixin() {}
                ActiveSetDeterminationBMixin() {}

            public:
                /// @copydoc humoto::constraints::ConstraintsBase::determineActiveSet
                void determineActiveSet(ActiveSetConstraints & active_set,
                                        const Location & location,
                                        const Solution & solution) const
                {
                    active_set.set(location, ConstraintActivationType::EQUALITY);
                }
        };
        /// @}


        // ======================================================================================
        // ======================================================================================
        // ======================================================================================

        /**
         * @defgroup ConstraintsViolationsComputation Violations of constraints
         *
         * @ingroup ConstraintsImplementation
         *
         * A set of mixins which implement computation of violations of
         * constraints based on the solution.
         */

        /**
         * @addtogroup ConstraintsViolationsComputation
         * @{
         */

        /**
         * @brief This mixin provides functionality for compuation of violations of
         * the bounds.
         *
         * @tparam t_Base base class
         */
        template <class t_Base>
            class HUMOTO_LOCAL ViolationsComputationLUMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ViolationsComputationLUMixin() {}
                ViolationsComputationLUMixin() {}

            public:
                /// @copydoc humoto::constraints::ConstraintsBase::computeViolations
                void computeViolations( ViolationsConstraints & violations,
                                        const Location & location,
                                        const Solution & solution) const
                {
                    HUMOTO_ASSERT(  static_cast<std::size_t>(t_Base::getNumberOfConstraints()) == location.length_,
                                    std::string("Cannot determine active set: internal error."));

                    for(std::size_t i = 0; i < location.length_; ++i)
                    {
                        double product = t_Base::getProduct(i, solution.get_x());

                        violations.set( location.offset_ + i,
                                        product - t_Base::getLowerBounds()[i],
                                        product - t_Base::getUpperBounds()[i]);
                    }
                }
        };


        /// @copydoc ViolationsComputationLUMixin
        template <class t_Base>
            class HUMOTO_LOCAL ViolationsComputationLMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ViolationsComputationLMixin() {}
                ViolationsComputationLMixin() {}

            public:
                /// @copydoc humoto::constraints::ConstraintsBase::computeViolations
                void computeViolations( ViolationsConstraints & violations,
                                        const Location & location,
                                        const Solution & solution) const
                {
                    HUMOTO_ASSERT(  static_cast<std::size_t>(t_Base::getNumberOfConstraints()) == location.length_,
                                    std::string("Cannot determine active set: internal error."));

                    for(std::size_t i = 0; i < location.length_; ++i)
                    {
                        double product = t_Base::getProduct(i, solution.get_x());

                        violations.set( location.offset_ + i,
                                        product - t_Base::getLowerBounds()[i],
                                        0.0);
                    }
                }
        };


        /// @copydoc ViolationsComputationLUMixin
        template <class t_Base>
            class HUMOTO_LOCAL ViolationsComputationUMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ViolationsComputationUMixin() {}
                ViolationsComputationUMixin() {}

            public:
                /// @copydoc humoto::constraints::ConstraintsBase::computeViolations
                void computeViolations( ViolationsConstraints & violations,
                                        const Location & location,
                                        const Solution & solution) const
                {
                    HUMOTO_ASSERT(  static_cast<std::size_t>(t_Base::getNumberOfConstraints()) == location.length_,
                                    std::string("Cannot determine active set: internal error."));

                    for(std::size_t i = 0; i < location.length_; ++i)
                    {
                        double product = t_Base::getProduct(i, solution.get_x());

                        violations.set( location.offset_ + i,
                                        0.0,
                                        product - t_Base::getUpperBounds()[i]);
                    }
                }
        };


        /// @copydoc ViolationsComputationLUMixin
        template <class t_Base>
            class HUMOTO_LOCAL ViolationsComputationBMixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ViolationsComputationBMixin() {}
                ViolationsComputationBMixin() {}

            public:
                /// @copydoc humoto::constraints::ConstraintsBase::computeViolations
                void computeViolations( ViolationsConstraints & violations,
                                        const Location & location,
                                        const Solution & solution) const
                {
                    HUMOTO_ASSERT(  static_cast<std::size_t>(t_Base::getNumberOfConstraints()) == location.length_,
                                    std::string("Cannot determine active set: internal error."));

                    for(std::size_t i = 0; i < location.length_; ++i)
                    {
                        double product = t_Base::getProduct(i, solution.get_x());

                        violations.set( location.offset_ + i,
                                        product - t_Base::getB()[i],
                                        product - t_Base::getB()[i]);
                    }
                }
        };


        /// @copydoc ViolationsComputationLUMixin
        template <class t_Base>
            class HUMOTO_LOCAL ViolationsComputationB0Mixin : public t_Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ViolationsComputationB0Mixin() {}
                ViolationsComputationB0Mixin() {}

            public:
                /// @copydoc humoto::constraints::ConstraintsBase::computeViolations
                void computeViolations( ViolationsConstraints & violations,
                                        const Location & location,
                                        const Solution & solution) const
                {
                    HUMOTO_ASSERT(  static_cast<std::size_t>(t_Base::getNumberOfConstraints()) == location.length_,
                                    std::string("Cannot determine active set: internal error."));

                    for(std::size_t i = 0; i < location.length_; ++i)
                    {
                        double product = t_Base::getProduct(i, solution.get_x());

                        violations.set( location.offset_ + i,
                                        product,
                                        product);
                    }
                }
        };
        /// @}
    }
}
