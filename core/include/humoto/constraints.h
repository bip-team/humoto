/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

/**
 * @ingroup _INTERNAL_
 * @defgroup ConstraintsImplementation Constraints
 */


namespace humoto
{
    namespace constraints
    {
#define HUMOTO_PARENT_CLASS_SHORTHAND   CopyTwoSidedInequalityToALMixin < CopyAnyToALUMixin < ViolationsComputationLUMixin < ActiveSetDeterminationLUMixin < BodyAMixin< BoundsLUMixin <t_Base> > > > > >
        /**
         * @brief Constraints 'lb <= A*x <= ub'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsALU : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsALU() {}
                ConstraintsALU() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getA;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getLowerBounds;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getUpperBounds;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::ALU);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getA().rows() == getLowerBounds().rows())
                                    && (getA().rows() == getUpperBounds().rows()),
                                    "Size of A does not match sizes of the bounds.");

                    HUMOTO_ASSERT(  (getA().cols() > 0) && (static_cast<std::size_t>(getA().cols()) == getNumberOfVariables()),
                                    "Incorrect number of columns in A.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND


#define HUMOTO_PARENT_CLASS_SHORTHAND CopyLowerInequalityToALMixin < CopyAnyToALUMixin < ViolationsComputationLMixin < ActiveSetDeterminationLMixin < BodyAMixin< BoundsLMixin <t_Base> > > > > >
        /**
         * @brief Constraints 'lb <= A*x'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsAL : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsAL() {}
                ConstraintsAL() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getA;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getLowerBounds;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::AL);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getA().rows() == getLowerBounds().rows()),
                                    "Size of A does not match sizes of the bounds.");

                    HUMOTO_ASSERT(  (getA().cols() > 0) && (static_cast<std::size_t>(getA().cols()) == getNumberOfVariables()),
                                    "Incorrect number of columns in A.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND


#define HUMOTO_PARENT_CLASS_SHORTHAND CopyUpperInequalityToALMixin < CopyAnyToALUMixin < ViolationsComputationUMixin < ActiveSetDeterminationUMixin < BodyAMixin< BoundsUMixin <t_Base> > > > > >
        /**
         * @brief Constraints 'A*x <= ub'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsAU : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsAU() {}
                ConstraintsAU() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getA;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getUpperBounds;


                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::AU);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  getA().rows() == getUpperBounds().rows(),
                                    "Size of A does not match sizes of the bounds.");

                    HUMOTO_ASSERT(  (getA().cols() > 0) && (static_cast<std::size_t>(getA().cols()) == getNumberOfVariables()),
                                    "Incorrect number of columns in A.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND



#define HUMOTO_PARENT_CLASS_SHORTHAND CopyAnyToALUMixin < CopyEqualityToABMixin < ViolationsComputationBMixin < ActiveSetDeterminationBMixin < BodyAMixin< BoundsBMixin <t_Base> > > > > >
        /**
         * @brief Constraints 'A*x = b'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsAB : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsAB() {}
                ConstraintsAB() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getA;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getB;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::AB);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getA().rows() == getB().rows()),
                                    "Size of A does not match size of b.");

                    HUMOTO_ASSERT(  (getA().cols() > 0) && (static_cast<std::size_t>(getA().cols()) == getNumberOfVariables()),
                                    "Incorrect number of columns in A.");
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void getATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATA(H);
                    g.noalias() = getA().transpose()*getB();
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void addATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATA(H);
                    g.noalias() += getA().transpose()*getB();
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND



#define HUMOTO_PARENT_CLASS_SHORTHAND CopyAnyToALUMixin < CopyEqualityToABMixin < ViolationsComputationB0Mixin < ActiveSetDeterminationBMixin < BodyAMixin< BoundsB0Mixin <t_Base> > > > > >
        /**
         * @brief Constraints 'A*x = 0'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsAB0 : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsAB0() {}
                ConstraintsAB0() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getA;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::AB0);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getA().cols() > 0) && (static_cast<std::size_t>(getA().cols()) == getNumberOfVariables()),
                                    "Incorrect number of columns in A.");
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void getATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATA(H);
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATb(g);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void addATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATA(H);
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATb(g);
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND


        // ======================================================================================
        // ======================================================================================
        // ======================================================================================


#define HUMOTO_PARENT_CLASS_SHORTHAND   CopyTwoSidedInequalityToALMixin < CopyAnyToALUMixin < ViolationsComputationLUMixin < ActiveSetDeterminationLUMixin < BodyASMixin< BoundsLUMixin <t_Base> > > > > >
        /**
         * @brief Constraints 'lb <= A*S*x <= ub'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsASLU : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsASLU() {}
                ConstraintsASLU() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getA;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getOffset;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getLowerBounds;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getUpperBounds;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::ASLU);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getA().rows() == getLowerBounds().rows())
                                    && (getA().rows() == getUpperBounds().rows()),
                                    "Size of A does not match sizes of the bounds.");

                    HUMOTO_ASSERT(  (getA().cols() > 0)
                                    && (static_cast<std::size_t>(getA().cols() + getOffset()) <= getNumberOfVariables()),
                                    "Incorrect number of columns in A.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND


#define HUMOTO_PARENT_CLASS_SHORTHAND CopyLowerInequalityToALMixin < CopyAnyToALUMixin < ViolationsComputationLMixin <ActiveSetDeterminationLMixin < BodyASMixin< BoundsLMixin <t_Base> > > > > >
        /**
         * @brief Constraints 'lb <= A*S*x'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsASL : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsASL() {}
                ConstraintsASL() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getA;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getOffset;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getLowerBounds;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::ASL);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getA().rows() == getLowerBounds().rows()),
                                    "Size of A does not match sizes of the bounds.");

                    HUMOTO_ASSERT(  (getA().cols() > 0)
                                    && (static_cast<std::size_t>(getA().cols() + getOffset()) <= getNumberOfVariables()),
                                    "Incorrect number of columns in A.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND


#define HUMOTO_PARENT_CLASS_SHORTHAND CopyUpperInequalityToALMixin < CopyAnyToALUMixin < ViolationsComputationUMixin < ActiveSetDeterminationUMixin < BodyASMixin< BoundsUMixin <t_Base> > > > > >
        /**
         * @brief Constraints 'A*S*x <= ub'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsASU : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsASU() {}
                ConstraintsASU() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getOffset;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getA;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getUpperBounds;


                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::ASU);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  getA().rows() == getUpperBounds().rows(),
                                    "Size of A does not match sizes of the bounds.");

                    HUMOTO_ASSERT(  (getA().cols() > 0)
                                    && (static_cast<std::size_t>(getA().cols() + getOffset()) <= getNumberOfVariables()),
                                    "Incorrect number of columns in A.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND



#define HUMOTO_PARENT_CLASS_SHORTHAND CopyAnyToALUMixin < CopyEqualityToABMixin < ViolationsComputationBMixin < ActiveSetDeterminationBMixin < BodyASMixin< BoundsBMixin <t_Base> > > > > >
        /**
         * @brief Constraints 'A*S*x = b'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsASB : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsASB() {}
                ConstraintsASB() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getOffset;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getA;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getB;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::ASB);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getA().rows() == getB().rows()),
                                    "Size of A does not match size of b.");

                    HUMOTO_ASSERT(  (getA().cols() > 0)
                                    && (static_cast<std::size_t>(getA().cols() + getOffset()) <= getNumberOfVariables()),
                                    "Incorrect number of columns in A.");
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void getATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATA(H);

                    g.resize(getNumberOfVariables());
                    std::ptrdiff_t right_space_size = getNumberOfVariables() - getOffset() - getA().cols();
                    g   <<  Eigen::VectorXd::Zero(getOffset()),
                            getA().transpose()*getB(),
                            Eigen::VectorXd::Zero(right_space_size);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void addATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATA(H);

                    g.segment(getOffset(), getA().cols()).noalias() += getA().transpose()*getB();
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND



#define HUMOTO_PARENT_CLASS_SHORTHAND CopyAnyToALUMixin < CopyEqualityToABMixin < ViolationsComputationB0Mixin < ActiveSetDeterminationBMixin < BodyASMixin< BoundsB0Mixin <t_Base> > > > > >
        /**
         * @brief Constraints 'A*S*x = 0'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsASB0 : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsASB0() {}
                ConstraintsASB0() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getOffset;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getA;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::ASB0);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getA().cols() > 0)
                                    && (static_cast<std::size_t>(getA().cols() + getOffset()) <= getNumberOfVariables()),
                                    "Incorrect number of columns in A.");
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void getATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATA(H);
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATb(g);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void addATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATA(H);
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATb(g);
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND


        // ======================================================================================
        // ======================================================================================
        // ======================================================================================


#define HUMOTO_PARENT_CLASS_SHORTHAND CopyTwoSidedInequalityToALMixin < CopyAnyToALUMixin < ViolationsComputationLUMixin < ActiveSetDeterminationLUMixin < BodyGIMixin< BoundsLUMixin <t_Base> > > > > >
        /**
         * @brief Constraints 'lb <= G * x[I] <= ub'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsGILU : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsGILU() {}
                ConstraintsGILU() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIndices;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIGains;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getLowerBounds;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getUpperBounds;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::GILU);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getIndices().rows() == getLowerBounds().rows())
                                    && (getIndices().rows() == getUpperBounds().rows()),
                                    "Size of I does not match sizes of the bounds.");

                    HUMOTO_ASSERT(  (getIndices().rows() == getIGains().rows()),
                                    "Size of the vector of gains must match the number of constraints.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND



#define HUMOTO_PARENT_CLASS_SHORTHAND CopyLowerInequalityToALMixin < CopyAnyToALUMixin < ViolationsComputationLMixin < ActiveSetDeterminationLMixin < BodyGIMixin< BoundsLMixin <t_Base> > > > > >
        /**
         * @brief Constraints 'lb <= G * x[I]'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsGIL : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsGIL() {}
                ConstraintsGIL() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIndices;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIGains;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getLowerBounds;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::GIL);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  getIndices().rows() == getLowerBounds().rows(),
                                    "Size of I does not match sizes of the bounds.");

                    HUMOTO_ASSERT(  (getIndices().rows() == getIGains().rows()),
                                    "Size of the vector of gains must match the number of constraints.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND


#define HUMOTO_PARENT_CLASS_SHORTHAND CopyUpperInequalityToALMixin < CopyAnyToALUMixin < ViolationsComputationUMixin < ActiveSetDeterminationUMixin < BodyGIMixin< BoundsUMixin <t_Base> > > > > >
        /**
         * @brief Constraints 'G * x[I] <= ub'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsGIU : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsGIU() {}
                ConstraintsGIU() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIndices;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIGains;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getUpperBounds;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::GIU);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  getIndices().rows() == getUpperBounds().rows(),
                                    "Size of I does not match sizes of the bounds.");

                    HUMOTO_ASSERT(  (getIndices().rows() == getIGains().rows()),
                                    "Size of the vector of gains must match the number of constraints.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND


#define HUMOTO_PARENT_CLASS_SHORTHAND CopyAnyToALUMixin < CopyEqualityToABMixin < ViolationsComputationBMixin < ActiveSetDeterminationBMixin < BodyGIMixin< BoundsBMixin <t_Base> > > > > >
        /**
         * @brief Constraints 'G*x[I] = b'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsGIB : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsGIB() {}
                ConstraintsGIB() {}


                /// @copydoc humoto::constraints::BoundsB0Mixin::getATb
                void addATb(Eigen::VectorXd &g) const
                {
                    for (EigenIndex i = 0; i < getIndices().rows(); ++i)
                    {
                        g[getIndices()[i]] += getIGains()[i] * getB()[i];
                    }
                }


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIndices;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIGains;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getB;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::GIB);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getIndices().rows() == getB().rows()),
                                    "Size of I does not match size of b.");

                    HUMOTO_ASSERT(  (getIndices().rows() == getIGains().rows()),
                                    "Size of the vector of gains must match the number of constraints.");
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void getATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATA(H);
                    g.setZero(getNumberOfVariables());
                    addATb(g);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void addATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATA(H);
                    addATb(g);
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND


#define HUMOTO_PARENT_CLASS_SHORTHAND CopyAnyToALUMixin < CopyEqualityToABMixin < ViolationsComputationB0Mixin < ActiveSetDeterminationBMixin < BodyGIMixin< BoundsB0Mixin <t_Base> > > > > >
        /**
         * @brief Constraints 'G*x[I] = 0'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsGIB0 : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsGIB0() {}
                ConstraintsGIB0() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIndices;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIGains;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::GIB0);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void getATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATA(H);
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATb(g);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void addATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATA(H);
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATb(g);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getIndices().rows() == getIGains().rows()),
                                    "Size of the vector of gains must match the number of constraints.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND


        // ======================================================================================
        // ======================================================================================
        // ======================================================================================


#define HUMOTO_PARENT_CLASS_SHORTHAND CopyTwoSidedInequalityToALMixin < CopyAnyToALUMixin < CopySimpleToILUMixin < ViolationsComputationLUMixin < ActiveSetDeterminationLUMixin < BodyIMixin< BoundsLUMixin <t_Base> > > > > > >
        /**
         * @brief Constraints 'lb <= x[I] <= ub'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsILU : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsILU() {}
                ConstraintsILU() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIndices;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getLowerBounds;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getUpperBounds;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::ILU);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getIndices().rows() == getLowerBounds().rows())
                                    && (getIndices().rows() == getUpperBounds().rows()),
                                    "Size of I does not match sizes of the bounds.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND


#define HUMOTO_PARENT_CLASS_SHORTHAND CopyLowerInequalityToALMixin < CopyAnyToALUMixin < CopySimpleToILUMixin < ViolationsComputationLMixin < ActiveSetDeterminationLMixin < BodyIMixin< BoundsLMixin <t_Base> > > > > > >
        /**
         * @brief Constraints 'lb <= x[I]'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsIL : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsIL() {}
                ConstraintsIL() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIndices;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getLowerBounds;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::IL);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  getIndices().rows() == getLowerBounds().rows(),
                                    "Size of I does not match sizes of the bounds.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND



#define HUMOTO_PARENT_CLASS_SHORTHAND CopyUpperInequalityToALMixin < CopyAnyToALUMixin < CopySimpleToILUMixin < ViolationsComputationUMixin < ActiveSetDeterminationUMixin < BodyIMixin< BoundsUMixin <t_Base> > > > > > >
        /**
         * @brief Constraints 'x[I] <= ub'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsIU : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsIU() {}
                ConstraintsIU() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getNumberOfVariables;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIndices;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getUpperBounds;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::IU);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  getIndices().rows() == getUpperBounds().rows(),
                                    "Size of I does not match sizes of the bounds.");
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND



#define HUMOTO_PARENT_CLASS_SHORTHAND CopyAnyToALUMixin < CopySimpleToILUMixin < CopyEqualityToABMixin < ViolationsComputationBMixin < ActiveSetDeterminationBMixin < BodyIMixin< BoundsBMixin <t_Base> > > > > > >
        /**
         * @brief Constraints 'x[I] = b'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsIB : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsIB() {}
                ConstraintsIB() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIndices;
                using HUMOTO_PARENT_CLASS_SHORTHAND::getB;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::IB);
                }



                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const
                {
                    HUMOTO_ASSERT(  (getIndices().rows() == getB().rows()),
                                    "Size of I does not match size of b.");
                }

                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void getATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATA(H);
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATb(g);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void addATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATA(H);
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATb(g);
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND



#define HUMOTO_PARENT_CLASS_SHORTHAND CopyAnyToALUMixin < CopySimpleToILUMixin < CopyEqualityToABMixin < ViolationsComputationB0Mixin < ActiveSetDeterminationBMixin < BodyIMixin< BoundsB0Mixin <t_Base> > > > > > >
        /**
         * @brief Constraints 'x[I] = 0'.
         *
         * @tparam t_Base Base class (@ref humoto::constraints::ConstraintsBase or @ref humoto::TaskBase)
         */
        template <class t_Base>
            class HUMOTO_LOCAL ConstraintsIB0 : public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConstraintsIB0() {}
                ConstraintsIB0() {}


            public:
                using HUMOTO_PARENT_CLASS_SHORTHAND::getIndices;



                /// @copydoc humoto::constraints::ConstraintsBase::getType
                ConstraintType::Type getType() const
                {
                    return (ConstraintType::IB0);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::checkConsistency
                void checkConsistency() const {}


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void getATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATA(H);
                    HUMOTO_PARENT_CLASS_SHORTHAND::getATb(g);
                }


                /// @copydoc humoto::constraints::ConstraintsBase::getATAandATb
                void addATAandATb(Eigen::MatrixXd &H, Eigen::VectorXd &g) const
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATA(H);
                    HUMOTO_PARENT_CLASS_SHORTHAND::addATb(g);
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND
    }
}
