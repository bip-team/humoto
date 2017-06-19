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
     * @brief Abstract base class (a task in a stack of tasks / hierarchy).
     *
     * @attention Implementation of this class should perform weighting during
     * formulation (in function form()).
     */
    class HUMOTO_LOCAL TaskBase : public constraints::ConstraintsBase, public config::StrictConfigurableBase
    {
        // The functions collected here are supposed to be called by
        // OptimizationProblem class members only.
        friend class HierarchyLevel;

        private:
            std::string         string_description_;

            ActiveSetConstraints    active_set_guess_;


        private:
            /**
             * @brief Prepare formulation of a task.
             *
             * @param[in] number_of_variables
             *
             * @attention This function is called automatically.
             */
            void preForm(const std::size_t number_of_variables)
            {
                is_modified_ = true;

                setNumberOfVariables(number_of_variables);
            }


            /**
             * @brief Finalize formulation of a task.
             *
             * @attention This function is called automatically.
             */
            void postForm()
            {
                // ckecks
                try
                {
                    checkConsistency();
                }
                catch (const std::exception &e)
                {
                    HUMOTO_THROW_MSG(std::string("Task '") + getDescription() + "' is inconsistent: " + e.what());
                }

                HUMOTO_ASSERT(  (active_set_guess_.size() == 0) || (active_set_guess_.size() == getNumberOfConstraints()),
                                std::string("Wrong size of the active set guess. Task: ") + getDescription());
            }


            /**
             * @brief Copy the actual active set of the task from the given
             * active set.
             *
             * @param[in] active_set active set
             * @param[in] location location of the active set
             */
            void copyActualActiveSetFrom(   const ActiveSetConstraints &active_set,
                                            const Location  &location)
            {
                active_set_actual_.initialize(active_set, location);
            }


            /**
             * @brief Copy the active set guess of the task to the given active
             * set.
             *
             * @param[in] location location of the active set
             * @param[in] active_set active set
             */
            void copyActiveSetGuessTo(  ActiveSetConstraints &active_set,
                                        const Location  &location) const
            {
                active_set.copyTo( location,
                                   active_set_guess_);
            }


        protected:
            bool                is_modified_;


        protected:
            #define HUMOTO_CONFIG_ENTRIES
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS

            ActiveSetConstraints    active_set_actual_;

            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~TaskBase() {}


            TaskBase()
            {
                setDefaults();
            }


            const std::string & getConfigSectionID() const
            {
                return (string_description_);
            }


            virtual void setDefaults()
            {
                is_modified_ = true;
                //string_description_ = "TaskBase";
            }



            /**
             * @brief Initialize active set guess with defaults.
             *
             * @param[in] sol_structure     structure of the solution
             * @param[in] model_base        model
             * @param[in] control_problem   control problem
             *
             * @attention This function is called automatically (after form())
             * and can be redefined in derfived tasks. It doesn't make sense to
             * redefine it for equality tasks.
             */
            virtual void guessActiveSet(const humoto::SolutionStructure &sol_structure,
                                        const humoto::Model &model_base,
                                        const humoto::ControlProblem &control_problem)
            {
                std::size_t num_ctr = getNumberOfConstraints();

                if (is_modified_ || (0 == active_set_actual_.size()))
                {
                    // default initialization
                    if (isEquality())
                    {
                        active_set_guess_.initialize(num_ctr, ConstraintActivationType::EQUALITY);
                    }
                    else
                    {
                        active_set_guess_.initialize(num_ctr, ConstraintActivationType::INACTIVE);
                    }
                }
                else
                {
                    // reuse actual active set from the previous iteration
                    HUMOTO_ASSERT( (num_ctr == active_set_actual_.size()),
                                   std::string("The task is marked as unmodified, "
                                               "but the size of the active set is not the same. Task: ")
                                   + getDescription());

                    active_set_guess_ = active_set_actual_;
                }
            }


            /**
             * @brief Form the task.
             *
             * @param[in] sol_structure     structure of the solution
             * @param[in] model_base        model
             * @param[in] control_problem   control problem
             *
             * @attention This function is called automatically and must be
             * defined in derived tasks.
             */
            virtual void    form(   const humoto::SolutionStructure &sol_structure,
                                    const humoto::Model             &model_base,
                                    const humoto::ControlProblem    &control_problem) = 0;



            /**
             * @brief By default the task is assumed to be modified. Use this
             * function to mark it as unmodified to avoid unnecessary
             * recomputations.
             *
             * @attention This method should be called from form() method only.
             *
             * @attention If this function is called on a modified task, the
             * result is undefined.
             */
            void markAsUnmodified()
            {
                is_modified_ = false;
            }


            /**
             * @brief Get actual active set.
             *
             * @return actual active set.
             */
            const ActiveSetConstraints & getActualActiveSet() const
            {
                return (active_set_actual_);
            }


            /**
             * @brief Get active set guess.
             *
             * @return active set guess.
             */
            ActiveSetConstraints & getActiveSetGuess()
            {
                return (active_set_guess_);
            }


            /**
             * @brief Set description of the task.
             *
             * @return description
             */
            void setDescription(const std::string &description)
            {
                string_description_ = description;
            }


            /**
             * @brief Log task.
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            virtual void    logTask(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                    const LogEntryName & parent = LogEntryName(),
                                    const std::string & name = "task") const = 0;

        public:
            /**
             * @brief Get description of the task.
             *
             * @return description
             */
            const std::string  getDescription() const
            {
                return (string_description_.c_str());
            }


            /**
             * @brief Log task.
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            void    log(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                        const LogEntryName & parent = LogEntryName(),
                        const std::string & name = "task") const
            {
                LogEntryName subname = parent;
                subname.add(name);

                logger.log(LogEntryName(subname).add("id"), getDescription());
                logTask(logger, subname, "");

                active_set_guess_.log(logger, subname, "active_set_guess");
                active_set_actual_.log(logger, subname, "active_set_actual");

                ConstraintsBase::log(logger, subname, "");
            }
    };



    /**
     * @brief General task mixin -- should be used in building general tasks
     *
     * @tparam t_Constraints Base class representing general constraints
     */
    template<class t_Constraints>
        class HUMOTO_LOCAL GeneralTaskBaseMixin : public t_Constraints
    {
        protected:
            double              gain_;


        protected:
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskBase); \
                HUMOTO_CONFIG_SCALAR_(gain);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~GeneralTaskBaseMixin() {}


            /**
             * @brief Constructor
             *
             * @param[in] description   description of the task
             * @param[in] gain          gain
             */
            GeneralTaskBaseMixin(const std::string &description,
                                 const double gain = 0.0)
            {
                t_Constraints::setDescription(description);
                setGain(gain);
            }


            virtual void setDefaults()
            {
                setGain(0.0);
            }

            void finalize() {}


            /**
             * @brief Log task.
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            virtual void    logTask(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                    const LogEntryName & parent = LogEntryName(),
                                    const std::string & name = "task") const
            {
                logger.log(LogEntryName(parent).add(name).add("gain"), getGain());
            }


        public:
            /**
             * @brief Sets gain.
             *
             * @param[in] gain new gain
             */
            void setGain(const double gain)
            {
                gain_ = gain;
                t_Constraints::is_modified_ = true;
            }


            /**
             * @brief Returns gain.
             *
             * @return gain
             */
            double getGain() const
            {
                return (gain_);
            }
    };


    /**
     * @brief simple task mixin -- should be used in building general tasks
     *
     * @tparam t_Constraints Base class representing simple constraints
     */
    template<class t_Constraints>
        class HUMOTO_LOCAL WeightedSimpleTaskBaseMixin : public t_Constraints
    {
        // The functions collected here are supposed to be called by
        // OptimizationProblem class members only.
        friend class HierarchyLevel;

        protected:
            double              gain_;


        protected:
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskBase); \
                HUMOTO_CONFIG_SCALAR_(gain);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~WeightedSimpleTaskBaseMixin() {}


            /**
             * @brief Constructor
             *
             * @param[in] description   description of the task
             * @param[in] gain          gain
             */
            WeightedSimpleTaskBaseMixin(const std::string &description,
                                        const double gain = 0.0)
            {
                t_Constraints::setDescription(description);
                setGain(gain);
            }


            virtual void setDefaults()
            {
                setGain(0.0);
            }
            void finalize() {}


            /**
             * @brief Log task.
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            virtual void    logTask(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                    const LogEntryName & parent = LogEntryName(),
                                    const std::string & name = "task") const
            {
                logger.log(LogEntryName(parent).add(name).add("gain"), getGain());
            }


        public:
            /**
             * @brief Sets gain.
             *
             * @param[in] gain new gain
             */
            virtual void setGain(const double gain)
            {
                gain_ = gain;
                t_Constraints::is_modified_ = true;
            }


            /**
             * @brief Returns gain.
             *
             * @return gain
             */
            double getGain() const
            {
                return (gain_);
            }
    };



    /**
     * @brief simple task mixin -- should be used in building general tasks
     *
     * @tparam t_Constraints Base class representing simple constraints
     */
    template<class t_Constraints>
        class HUMOTO_LOCAL SimpleTaskBaseMixin : public t_Constraints
    {
        // The functions collected here are supposed to be called by
        // OptimizationProblem class members only.
        friend class HierarchyLevel;


        protected:
            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~SimpleTaskBaseMixin() {}


            /**
             * @brief Constructor
             *
             * @param[in] description   description of the task
             */
            SimpleTaskBaseMixin(const std::string &description)
            {
                t_Constraints::setDescription(description);
            }



            /**
             * @brief Log task.
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            virtual void    logTask(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                    const LogEntryName & parent = LogEntryName(),
                                    const std::string & name = "task") const
            {
            }
    };


#ifdef HUMOTO_DOXYGEN_PROCESSING
    // Define fake classes instead of typedefs used below in order to force
    // doxygen to create corresponding documentation pages.
#define HUMOTO_TASK_CLASS_GENERATOR(parent, name) class name : public parent {}
#else
#define HUMOTO_TASK_CLASS_GENERATOR(parent, name) typedef parent name
#endif

    /**
     * @ingroup Tasks
     *
     * @addtogroup BaseTasks    Task base classes
     * @brief Base classes for user-defined tasks
     * @{
     */
    /// @brief Task:   lb <= A*x <= ub
    HUMOTO_TASK_CLASS_GENERATOR(GeneralTaskBaseMixin< constraints::ConstraintsALU< TaskBase> >, TaskALU);
    /// @brief Task: lb <= A*x
    HUMOTO_TASK_CLASS_GENERATOR(GeneralTaskBaseMixin< constraints::ConstraintsAL< TaskBase> >, TaskAL);
    /// @brief Task: A*x <= ub
    HUMOTO_TASK_CLASS_GENERATOR(GeneralTaskBaseMixin< constraints::ConstraintsAU< TaskBase> >, TaskAU);
    /// @brief Task: A*x = b
    HUMOTO_TASK_CLASS_GENERATOR(GeneralTaskBaseMixin< constraints::ConstraintsAB< TaskBase> >, TaskAB);
    /// @brief Task: A*x = 0
    HUMOTO_TASK_CLASS_GENERATOR(GeneralTaskBaseMixin< constraints::ConstraintsAB0< TaskBase> >, TaskAB0);


    /// @brief Task: lb <= A*S*x <= ub
    HUMOTO_TASK_CLASS_GENERATOR(GeneralTaskBaseMixin< constraints::ConstraintsASLU< TaskBase> >, TaskASLU);
    /// @brief Task: lb <= A*S*x
    HUMOTO_TASK_CLASS_GENERATOR(GeneralTaskBaseMixin< constraints::ConstraintsASL< TaskBase> >, TaskASL);
    /// @brief Task: A*S*x <= ub
    HUMOTO_TASK_CLASS_GENERATOR(GeneralTaskBaseMixin< constraints::ConstraintsASU< TaskBase> >, TaskASU);
    /// @brief Task: A*S*x = b
    HUMOTO_TASK_CLASS_GENERATOR(GeneralTaskBaseMixin< constraints::ConstraintsASB< TaskBase> >, TaskASB);
    /// @brief Task: A*S*x = 0
    HUMOTO_TASK_CLASS_GENERATOR(GeneralTaskBaseMixin< constraints::ConstraintsASB0< TaskBase> >, TaskASB0);


    /// @brief Task: lb <= diag(G)*x[I] <= ub
    HUMOTO_TASK_CLASS_GENERATOR(WeightedSimpleTaskBaseMixin< constraints::ConstraintsGILU<TaskBase> >, TaskGILU);
    /// @brief Task: lb <= diag(G)*x[I]
    HUMOTO_TASK_CLASS_GENERATOR(WeightedSimpleTaskBaseMixin< constraints::ConstraintsGIL< TaskBase> >, TaskGIL);
    /// @brief Task: diag(G)*x[I] <= ub
    HUMOTO_TASK_CLASS_GENERATOR(WeightedSimpleTaskBaseMixin< constraints::ConstraintsGIU< TaskBase> >, TaskGIU);
    /// @brief Task: diag(G)*x[I] = b
    HUMOTO_TASK_CLASS_GENERATOR(WeightedSimpleTaskBaseMixin< constraints::ConstraintsGIB< TaskBase> >, TaskGIB);
    /// @brief Task: diag(G)*x[I] = 0
    HUMOTO_TASK_CLASS_GENERATOR(WeightedSimpleTaskBaseMixin< constraints::ConstraintsGIB0<TaskBase> >, TaskGIB0);


    /// @brief Task: lb <= x[I] <= ub
    HUMOTO_TASK_CLASS_GENERATOR(SimpleTaskBaseMixin< constraints::ConstraintsILU< TaskBase> >, TaskILU);
    /// @brief Task: lb <= x[I]
    HUMOTO_TASK_CLASS_GENERATOR(SimpleTaskBaseMixin< constraints::ConstraintsIL< TaskBase> >, TaskIL);
    /// @brief Task: x[I] <= ub
    HUMOTO_TASK_CLASS_GENERATOR(SimpleTaskBaseMixin< constraints::ConstraintsIU< TaskBase> >, TaskIU);
    /// @brief Task: x[I] = b
    HUMOTO_TASK_CLASS_GENERATOR(SimpleTaskBaseMixin< constraints::ConstraintsIB< TaskBase> >, TaskIB);
    /// @brief Task: x[I] = 0
    HUMOTO_TASK_CLASS_GENERATOR(SimpleTaskBaseMixin< constraints::ConstraintsIB0< TaskBase> >, TaskIB0);
    /// @}
#undef HUMOTO_TASK_CLASS_GENERATOR

    typedef boost::shared_ptr<humoto::TaskBase> TaskSharedPointer;
}
