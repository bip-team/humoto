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
     * @brief Type of constraint activation
     */
    class ConstraintActivationType
    {
        public:
            enum Type
            {
                /// Udefined
                UNDEFINED = 0,

                /// Constraint is inactive
                INACTIVE = 1,

                /// Lower bound is activated
                LOWER_BOUND = 2,

                /// Upper bound is activated
                UPPER_BOUND = 3,

                /// Constraint is an equality (always active)
                EQUALITY = 4
            };
    };


    /**
     * @brief Active set corresponding to #Constraints class.
     */
    class HUMOTO_LOCAL ActiveSetConstraints
    {
        private:
            std::vector<humoto::ConstraintActivationType::Type>   data_;

        public:
            /**
             * @brief Access activation flag of a constraint (const)
             *
             * @param[in] constraint_index
             *
             * @return reference to the activation flag
             */
            const humoto::ConstraintActivationType::Type &  operator[] (const std::size_t  constraint_index) const
            {
                HUMOTO_ASSERT(  ( constraint_index < data_.size() ),
                                "Active set index is out of bounds.");
                return (data_[constraint_index]);
            }


            /**
             * @brief Access activation flag of a constraint
             *
             * @param[in] constraint_index
             *
             * @return reference to the activation flag
             */
            humoto::ConstraintActivationType::Type &  operator[] (const std::size_t  constraint_index)
            {
                HUMOTO_ASSERT(  ( constraint_index < data_.size() ),
                                "Active set index is out of bounds.");
                return (data_[constraint_index]);
            }


            /**
             * @brief Size of the active set.
             *
             * @return Size of the active set.
             */
            std::size_t size() const
            {
                return(data_.size());
            }


            /**
             * @brief Reset active set.
             */
            void reset()
            {
                data_.clear();
            }


            /**
             * @brief Set type of all constraints
             *
             * @param[in] type  type of the constraints
             */
            void set( const ConstraintActivationType::Type  type)
            {
                set(Location(0, size()), type);
            }



            /**
             * @brief Set type of a set of constraints
             *
             * @param[in] location
             * @param[in] type
             */
            void set(   const Location                          &location,
                        const ConstraintActivationType::Type    type)
            {
                HUMOTO_ASSERT(location.checkLength( size() ), "Incorrect location in an active set.");
                std::fill(  data_.begin() + location.front(),
                            data_.begin() + location.end(),
                            type);
            }


            /**
             * @brief Set type of a set of constraints
             *
             * @param[in] location          location in this active set
             * @param[in] other_active_set  other active set
             */
            void copyTo(const Location              &location,
                        const ActiveSetConstraints  &other_active_set)
            {
                HUMOTO_ASSERT(location.checkLength( size() ), "Incorrect location in an active set.");

                std::copy(  other_active_set.data_.begin(),
                            other_active_set.data_.end(),
                            data_.begin() + location.front());
            }


            /**
             * @brief Set type of a set of constraints
             *
             * @param[in] other_active_set  other active set
             * @param[in] location          location in other active set
             */
            void copyFrom(  const ActiveSetConstraints  &other_active_set,
                            const Location              &location)
            {
                HUMOTO_ASSERT(location.checkLength( other_active_set.size() ), "Incorrect location in an active set.");
                HUMOTO_ASSERT(location.length_ == size(), "Incorrect location in an active set.");

                std::copy(  other_active_set.data_.begin() + location.front(),
                            other_active_set.data_.begin() + location.end(),
                            data_.begin());
            }


            /**
             * @brief Initialize active set
             *
             * @param[in] number_of_constraints number of constraints for each level
             */
            void initialize( const std::size_t number_of_constraints)
            {
                reset();
                data_.resize(number_of_constraints);
            }


            /**
             * @brief Initialize active set
             *
             * @param[in] other_active_set  other active set
             * @param[in] location          location in other active set
             */
            void initialize(const ActiveSetConstraints  &other_active_set,
                            const Location              &location)
            {
                HUMOTO_ASSERT(location.checkLength( other_active_set.size() ), "Internal error.");

                if (size() != location.length_)
                {
                    reset();
                    data_.resize(location.length_);
                }

                copyFrom(other_active_set, location);
            }


            /**
             * @brief Initialize active set
             *
             * @param[in] number_of_constraints number of constraints for each level
             * @param[in] type  type of the constraints
             */
            void initialize(const std::size_t              number_of_constraints,
                            const ConstraintActivationType::Type  type)
            {
                reset();
                data_.resize(number_of_constraints, type);
            }



            /**
             * @brief Shift active set forward while preserving the length,
             * the trailing elements are initialized with the given type.
             *
             * @param[in] shift_size shift size
             * @param[in] type       type of the trailing elements
             */
            void shift( const std::size_t shift_size,
                        const ConstraintActivationType::Type type)
            {
                HUMOTO_ASSERT(  shift_size < size(),
                                "Shift size exceeded size of the active set.");

                std::rotate(data_.begin(),
                            data_.begin() + shift_size,
                            data_.end());

                std::fill(  data_.begin() + (size() - shift_size),
                            data_.end(),
                            type);
            }


            /**
             * @brief Returns number of active inequality constraints.
             *
             * @return number of active inequality constraints.
             */
            std::size_t     countActiveInequalities() const
            {
                std::size_t num_activated_inequalities = 0;

                for (std::size_t i = 0; i < data_.size(); ++i)
                {
                    switch (data_[i])
                    {
                        case ConstraintActivationType::LOWER_BOUND:
                        case ConstraintActivationType::UPPER_BOUND:
                            ++num_activated_inequalities;
                            break;
                        default:
                            break;
                    }
                }

                return(num_activated_inequalities);
            }


            /**
             * @brief Log an active set
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                        const LogEntryName & parent = LogEntryName(),
                        const std::string & name = "") const
            {
                logger.log(LogEntryName(parent).add(name), data_);
            }
    };


    /**
     * @brief Active set corresponding to a hierarchy of #Constraints.
     */
    class HUMOTO_LOCAL ActiveSet
    {
        private:
            std::vector< ActiveSetConstraints >    data_;


        public:
            /**
             * @brief Access active set of a given level (const)
             *
             * @param[in] level_index index of the level
             *
             * @return reference to the active set.
             */
            const humoto::ActiveSetConstraints &  operator[] (const std::size_t  level_index) const
            {
                HUMOTO_ASSERT(  level_index < size(), "Internal error.");
                return (data_[level_index]);
            }


            /**
             * @brief Access active set of a given level
             *
             * @param[in] level_index index of the level
             *
             * @return reference to the active set.
             */
            humoto::ActiveSetConstraints &  operator[] (const std::size_t  level_index)
            {
                HUMOTO_ASSERT(  level_index < size(), "Internal error.");
                return (data_[level_index]);
            }


            /**
             * @brief Set type of all constraints on the given level
             *
             * @param[in] level_index   index of the level
             * @param[in] type  type of the constraints
             */
            void set(   const std::size_t  level_index,
                        const ConstraintActivationType::Type  type)
            {
                HUMOTO_ASSERT(  level_index < size(), "Internal error.");
                data_[level_index].set(type);
            }


            /**
             * @brief Size of the active set.
             *
             * @return Size of the active set.
             */
            std::size_t size() const
            {
                return(data_.size());
            }


            /**
             * @brief Reset active set.
             */
            void reset()
            {
                data_.clear();
            }


            /**
             * @brief Initialize active set
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
             * @brief Log an active set
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                        const LogEntryName & parent = LogEntryName(),
                        const std::string & name = "active_set") const
            {
                LogEntryName subname = parent;
                subname.add(name);
                for (std::size_t i = 0; i < data_.size(); ++i)
                {
                    data_[i].log(logger, LogEntryName(subname).add(i), "");
                }
            }
    };
}
