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
     * @brief Violations corresponding to #Constraints class.
     */
    class HUMOTO_LOCAL ViolationsConstraints
    {
        private:
            Eigen::VectorXd         data_;

            Eigen::VectorXd         lower_;
            Eigen::VectorXd         upper_;


        public:
            /**
             * @brief Access violation of a constraint (const)
             *
             * @param[in] constraint_index
             *
             * @return reference to the violation
             */
            double    operator[] (const std::size_t  constraint_index) const
            {
                HUMOTO_ASSERT(  ( constraint_index < static_cast<std::size_t>(data_.size()) ),
                                "Violation index is out of bounds.");
                return (data_[constraint_index]);
            }


            /**
             * @brief Access violation of a lower/upper bounds
             *
             * @param[in] constraint_index
             *
             * @return reference to the violation
             */
            double    getLower (const std::size_t  constraint_index) const
            {
                HUMOTO_ASSERT(  ( constraint_index < static_cast<std::size_t>(lower_.size()) ),
                                "Violation index is out of bounds.");
                return (lower_[constraint_index]);
            }


            /// @copydoc humoto::ViolationsConstraints::getLower
            double    getUpper (const std::size_t  constraint_index) const
            {
                HUMOTO_ASSERT(  ( constraint_index < static_cast<std::size_t>(upper_.size()) ),
                                "Violation index is out of bounds.");
                return (upper_[constraint_index]);
            }


            /**
             * @brief Set violation of a constraint
             *
             * @param[in] constraint_index
             * @param[in] lower_bound_diff  A*x - lb (0.0 if lb is undefined)
             * @param[in] upper_bound_diff  A*x - ub (0.0 if ub is undefined)
             */
            void set (  const std::size_t  constraint_index,
                        const double lower_bound_diff = 0.0,
                        const double upper_bound_diff = 0.0)
            {
                HUMOTO_ASSERT(  ( constraint_index < static_cast<std::size_t>(data_.rows()) ),
                                "Violation index is out of bounds.");

                lower_[constraint_index] = lower_bound_diff;
                upper_[constraint_index] = upper_bound_diff;

                if (lower_bound_diff >= 0)
                {
                    if (upper_bound_diff <= 0)
                    {
                        data_[constraint_index] = 0.0;
                    }
                    else
                    {
                        data_[constraint_index] = upper_bound_diff;
                    }
                }
                else
                {
                    data_[constraint_index] = lower_bound_diff;
                }
            }


            /**
             * @brief Size of the the vector of violations.
             *
             * @return Size of the the vector of violations.
             */
            std::size_t size() const
            {
                return(data_.rows());
            }



            /**
             * @brief Set all violations to zero
             */
            void setZero()
            {
                data_.setZero();
                lower_.setZero();
                upper_.setZero();
            }



            /**
             * @brief Set violations of a set of constraints to zero
             *
             * @param[in] location
             */
            void setZero(   const Location                          &location)
            {
                HUMOTO_ASSERT(location.checkLength( size() ), "Incorrect location in a vector of violations.");

                data_.segment(location.offset_, location.length_).setZero();
                lower_.segment(location.offset_, location.length_).setZero();
                upper_.segment(location.offset_, location.length_).setZero();
            }


            /**
             * @brief Set violations of a set of constraints
             *
             * @param[in] location          location in this vector of violations
             * @param[in] other_violations  other vector of violations
             */
            void copyTo(const Location              &location,
                        const ViolationsConstraints  &other_violations)
            {
                HUMOTO_ASSERT(location.checkLength( size() ), "Incorrect location in a vector of violations.");

                data_.segment(location.offset_, location.length_) = other_violations.data_;
                lower_.segment(location.offset_, location.length_) = other_violations.lower_;
                upper_.segment(location.offset_, location.length_) = other_violations.upper_;
            }


            /**
             * @brief Set violations of a set of constraints
             *
             * @param[in] other_violations  other vector of violations
             * @param[in] location          location in other vector of violations
             */
            void copyFrom(  const ViolationsConstraints  &other_violations,
                            const Location              &location)
            {
                HUMOTO_ASSERT(location.checkLength( other_violations.size() ), "Incorrect location in a vector of violations.");
                HUMOTO_ASSERT(location.length_ == size(), "Incorrect location in a vector of violations.");

                data_ = other_violations.data_.segment(location.offset_, location.length_);
                lower_ = other_violations.lower_.segment(location.offset_, location.length_);
                upper_ = other_violations.upper_.segment(location.offset_, location.length_);
            }


            /**
             * @brief Initialize vector of violations
             *
             * @param[in] other_violations  other vector of violations
             * @param[in] location          location in other vector of violations
             */
            void initialize(const ViolationsConstraints  &other_violations,
                            const Location              &location)
            {
                HUMOTO_ASSERT(location.checkLength( other_violations.size() ), "Internal error.");

                if (size() != location.length_)
                {
                    data_.resize(location.length_);
                    lower_.resize(location.length_);
                    upper_.resize(location.length_);
                }

                copyFrom(other_violations, location);
            }


            /**
             * @brief Initialize vector of violations
             *
             * @param[in] number_of_constraints number of constraints for each level
             */
            void initialize(const std::size_t  number_of_constraints)
            {
                data_.resize(number_of_constraints);
                lower_.resize(number_of_constraints);
                upper_.resize(number_of_constraints);
            }



            /**
             * @brief Log violations
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name
             */
            void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                        const LogEntryName & parent = LogEntryName(),
                        const std::string & name = "") const
            {
                LogEntryName subname = parent; 
                subname.add(name);

                logger.log(LogEntryName(subname).add("violations"), data_);
                logger.log(LogEntryName(subname).add("lower_bound_diff"), lower_);
                logger.log(LogEntryName(subname).add("upper_bound_diff"), upper_);
            }
    };


    /**
     * @brief Violations corresponding to a hierarchy of #Constraints.
     */
    class HUMOTO_LOCAL Violations
    {
        private:
            std::vector< ViolationsConstraints >    data_;


        public:
            /**
             * @brief Access violations of a given level (const)
             *
             * @param[in] level_index index of the level
             *
             * @return reference to violations.
             */
            const ViolationsConstraints &  operator[] (const std::size_t  level_index) const
            {
                HUMOTO_ASSERT(  level_index < size(), "Internal error.");
                return (data_[level_index]);
            }


            /**
             * @brief Access violations of a given level
             *
             * @param[in] level_index index of the level
             *
             * @return reference to violations.
             */
            ViolationsConstraints &  operator[] (const std::size_t  level_index)
            {
                HUMOTO_ASSERT(  level_index < size(), "Internal error.");
                return (data_[level_index]);
            }


            /**
             * @brief Set violations of all constraints on the given level
             *
             * @param[in] level_index   index of the level
             */
            void setZero(const std::size_t  level_index)
            {
                HUMOTO_ASSERT(  level_index < size(), "Internal error.");
                data_[level_index].setZero();
            }


            /**
             * @brief Size
             *
             * @return Size
             */
            std::size_t size() const
            {
                return(data_.size());
            }


            /**
             * @brief Reset
             */
            void reset()
            {
                data_.clear();
            }


            /**
             * @brief Initialize violations
             *
             * @param[in] number_of_levels number of levels
             * @param[in] number_of_constraints number of constraints for each level
             */
            void initialize(const std::size_t                  number_of_levels,
                            const std::vector< std::size_t >   &number_of_constraints)
            {
                reset();
                data_.resize(number_of_levels);

                for (std::size_t i = 0; i < number_of_levels; ++i)
                {
                    data_[i].initialize(number_of_constraints[i]);
                }
            }


            /**
             * @brief Log violations
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                        const LogEntryName & parent = LogEntryName(),
                        const std::string & name = "violations") const
            {
                LogEntryName subname = parent; subname.add(name);
                for (std::size_t i = 0; i < data_.size(); ++i)
                {
                    data_[i].log(logger, LogEntryName(subname).add(i), "");
                }
            }
    };
}
