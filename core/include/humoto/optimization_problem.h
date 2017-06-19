/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    /**
     * @brief This class represents one level of a hierarchy.
     */
    class HUMOTO_LOCAL HierarchyLevel
    {
        // This class needs access to some non-public methods
        friend class OptimizationProblem;

        private:
            std::size_t            number_of_general_constraints_;
            std::size_t            number_of_simple_constraints_;

            std::size_t            number_of_equality_constraints_;
            std::size_t            number_of_twosided_constraints_;


        private:
            /**
             * @brief Reset variables.
             */
            void reset()
            {
                number_of_general_constraints_ = 0;
                number_of_simple_constraints_ = 0;

                number_of_equality_constraints_ = 0;
                number_of_twosided_constraints_ = 0;
            }


        protected:
            /**
             * @brief Form hierarchy.
             *
             * @param[in] sol_structure                     initialized solution structure
             * @param[in] model                             model of the system
             * @param[in] control_problem                   control problem associated with the model
             * @param[in] is_active_set_guessing_enabled
             */
            void    form(   const humoto::SolutionStructure & sol_structure,
                            const humoto::Model             & model,
                            const humoto::ControlProblem    & control_problem,
                            const bool                      is_active_set_guessing_enabled)
            {
                reset();

                // loop over tasks
                HUMOTO_ASSERT(tasks_.size() > 0, "No tasks on a hierarchy level.");

                for (   std::list<TaskInfo>::iterator it = tasks_.begin();
                        it != tasks_.end();
                        ++it)
                {
                    it->ptr_->preForm(sol_structure.getNumberOfVariables());
                    it->ptr_->form(sol_structure, model, control_problem);
                    if (is_active_set_guessing_enabled)
                    {
                       it->ptr_->guessActiveSet(sol_structure, model, control_problem);
                    }
                    it->ptr_->postForm();


                    std::size_t num_ctr = it->ptr_->getNumberOfConstraints();

                    if (it->ptr_->isEquality())
                    {
                        number_of_equality_constraints_ += num_ctr;
                    }

                    if (it->ptr_->isTwoSidedInequality())
                    {
                        number_of_twosided_constraints_ += num_ctr;
                    }

                    if (it->ptr_->isSimple())
                    {
                        // number_of_general_constraints_ is added since all
                        // simple constraints are added after general constraints
                        it->location_ = Location(number_of_simple_constraints_ + number_of_general_constraints_, num_ctr);
                        number_of_simple_constraints_ += num_ctr;
                    }
                    else
                    {
                        it->location_ = Location(number_of_general_constraints_, num_ctr);
                        number_of_general_constraints_ += num_ctr;
                    }
                }

                HUMOTO_ASSERT(getNumberOfConstraints() > 0, "No constraints on a hierarchy level.");
            }



            /**
             * @brief Form active set guess
             *
             * @param[out] active_set active set guess
             */
            void guessActiveSet(ActiveSetConstraints &active_set) const
            {
                for (   std::list<TaskInfo>::const_iterator it = tasks_.begin();
                        it != tasks_.end();
                        ++it)
                {
                    it->ptr_->copyActiveSetGuessTo(active_set, it->location_);
                }
            }


            /**
             * @brief Process actual active set: extract active sets of
             * individual tasks.
             *
             * @param[in] active_set
             */
            void processActiveSet(const humoto::ActiveSetConstraints & active_set)
            {
                for (   std::list<TaskInfo>::iterator it = tasks_.begin();
                        it != tasks_.end();
                        ++it)
                {
                    it->ptr_->copyActualActiveSetFrom(active_set, it->location_);
                }
            }



            /**
             * @brief Determine active set based on the solution
             *
             * @param[out] active_set active set
             * @param[in] solution solution
             */
            void determineActiveSet(ActiveSetConstraints    & active_set,
                                    const Solution          & solution) const
            {
                for (   std::list<TaskInfo>::const_iterator it = tasks_.begin();
                        it != tasks_.end();
                        ++it)
                {
                    it->ptr_->determineActiveSet(active_set, it->location_, solution);
                }
            }



            /**
             * @brief Compute violations based on the solution
             *
             * @param[out] violations violations
             * @param[in] solution solution
             */
            void computeViolations( ViolationsConstraints   & violations,
                                    const Solution          & solution) const
            {
                for (   std::list<TaskInfo>::const_iterator it = tasks_.begin();
                        it != tasks_.end();
                        ++it)
                {
                    it->ptr_->computeViolations(violations, it->location_, solution);
                }
            }


            /**
             * @brief Add task to the optimization problem.
             *
             * @param[in,out] task_pointer  pointer to a task
             */
            void pushTask(TaskSharedPointer task_pointer)
            {
                if (task_pointer->isSimple())
                {
                    tasks_.push_back(TaskInfo(task_pointer));
                }
                else
                {
                    tasks_.push_front(TaskInfo(task_pointer));
                }
            }


        public:
            /**
             * @brief Information about a task in a hierarchy
             */
            class HUMOTO_LOCAL TaskInfo
            {
                public:
                    /// task pointer
                    TaskSharedPointer ptr_;

                    /// location of the task constraints on a hierarchy level
                    Location        location_;


                public:
                    /**
                     * @brief Constructor
                     *
                     * @param[in] ptr pointer
                     * @param[in] offset
                     * @param[in] length
                     */
                    TaskInfo   (TaskSharedPointer ptr,
                                const std::size_t offset = 0,
                                const std::size_t length = 0)
                        : location_(offset, length)
                    {
                        ptr_ = ptr;
                    }
            };


            /**
             * @brief General constraints are added in the beginning, simple --
             * in the end. It is easier to process tasks this way and
             * formulation of the Hessian can be slightly faster, if objective
             * includes general and simple tasks.
             */
            std::list<TaskInfo>     tasks_;


        public:
            /**
             * @brief Default constructor
             */
            HierarchyLevel()
            {
                reset();
            }


            /**
             * @brief Generates (general) objective containing all constraints on this level
             *
             * @param[in] sol_structure     structure of the solution
             * @param[out] all_constraints  constraints
             */
            void getAllConstraints( constraints::ContainerALU         & all_constraints,
                                    const humoto::SolutionStructure & sol_structure) const
            {
                all_constraints.reset(getNumberOfConstraints(),
                                      sol_structure.getNumberOfVariables());

                for (   std::list<TaskInfo>::const_iterator it = tasks_.begin();
                        it != tasks_.end();
                        ++it)
                {
                    it->ptr_->copyTo(all_constraints, it->location_.offset_);
                }
            }


            /**
             * @brief Generates (general) objective containing all constraints on this level
             *
             * @param[out] ineq_constraints inequality constraints
             * @param[out] eq_constraints   equality constraints
             * @param[in] sol_structure     structure of the solution
             */
            void getInEqualityConstraints(  constraints::ContainerAL          & ineq_constraints,
                                            constraints::ContainerAB          & eq_constraints,
                                            const humoto::SolutionStructure & sol_structure) const
            {
                ineq_constraints.reset( getNumberOfInequalityConstraints() + number_of_twosided_constraints_,
                                        sol_structure.getNumberOfVariables());

                eq_constraints.reset(   getNumberOfEqualityConstraints(),
                                        sol_structure.getNumberOfVariables());


                std::size_t ineq_offset = 0;
                std::size_t eq_offset = 0;

                for (   std::list<TaskInfo>::const_iterator it = tasks_.begin();
                        it != tasks_.end();
                        ++it)
                {
                    if (it->ptr_->isEquality())
                    {
                        eq_offset = it->ptr_->copyTo(eq_constraints, eq_offset);
                    }
                    else
                    {
                        ineq_offset = it->ptr_->copyTo(ineq_constraints, ineq_offset);
                    }
                }
            }


            /**
             * @brief Generates (general) objective containing all equality constraints on this level
             *
             * @param[out] eq_constraints   equality constraints
             * @param[in] sol_structure     structure of the solution
             */
            void getEqualityConstraints(  constraints::ContainerAB          & eq_constraints,
                                          const humoto::SolutionStructure & sol_structure) const
            {
                eq_constraints.reset(   getNumberOfEqualityConstraints(),
                                        sol_structure.getNumberOfVariables());


                std::size_t eq_offset = 0;

                for (   std::list<TaskInfo>::const_iterator it = tasks_.begin();
                        it != tasks_.end();
                        ++it)
                {
                    if (it->ptr_->isEquality())
                    {
                        eq_offset = it->ptr_->copyTo(eq_constraints, eq_offset);
                    }
                }
            }


            /**
             * @brief Generates objective containing all simple constraints on this level
             *
             * @param[in] sol_structure     structure of the solution
             * @param[out] constraints      constraints
             *
             * @attention Simple constraints always precede general
             * constraints, e.g., in an active set.
             */
            void getSimpleConstraints(  constraints::ContainerILU & constraints,
                                        const humoto::SolutionStructure & sol_structure) const
            {
                constraints.reset(getNumberOfSimpleConstraints(),
                                  sol_structure.getNumberOfVariables());


                std::size_t offset = 0;
                for (   std::list<TaskInfo>::const_iterator it = tasks_.begin();
                        (it != tasks_.end());
                        ++it)
                {
                    if (it->ptr_->isSimple())
                    {
                        offset = it->ptr_->copyTo(constraints, offset);
                    }
                }
            }


            /**
             * @brief Generates objective containing all general constraints on this level
             *
             * @param[in] sol_structure     structure of the solution
             * @param[out] constraints      constraints
             *
             * @attention Simple constraints always precede general
             * constraints, e.g., in an active set.
             */
            void getGeneralConstraints( constraints::ContainerALU & constraints,
                                        const humoto::SolutionStructure & sol_structure) const
            {
                constraints.reset(getNumberOfGeneralConstraints(),
                                  sol_structure.getNumberOfVariables());

                std::size_t offset = 0;
                for (   std::list<TaskInfo>::const_iterator it = tasks_.begin();
                        (it != tasks_.end()) && (!it->ptr_->isSimple());
                        ++it)
                {
                    offset = it->ptr_->copyTo(constraints, offset);
                }
            }


            /**
             * @brief Get total number of constraints
             *
             * @return total number of constraints
             */
            std::size_t getNumberOfConstraints() const
            {
                return (number_of_general_constraints_
                        +
                        number_of_simple_constraints_);
            }


            /**
             * @brief Get number of simple constraints
             *
             * @return number of simple constraints
             */
            std::size_t getNumberOfSimpleConstraints() const
            {
                return (number_of_simple_constraints_);
            }


            /**
             * @brief Get number of general constraints
             *
             * @return number of general constraints
             */
            std::size_t getNumberOfGeneralConstraints() const
            {
                return (number_of_general_constraints_);
            }


            /**
             * @brief Get number of inequality constraints
             *
             * @return number of inequality constraints
             */
            std::size_t getNumberOfEqualityConstraints() const
            {
                return (number_of_equality_constraints_);
            }


            /**
             * @brief Get number of inequality constraints
             *
             * @return number of inequality constraints
             */
            std::size_t getNumberOfInequalityConstraints() const
            {
                return (getNumberOfConstraints() - getNumberOfEqualityConstraints());
            }


            /**
             * @brief True if all constraints on this level are simple
             *
             * @return true/false
             */
            bool isSimple() const
            {
                return (getNumberOfConstraints() == number_of_simple_constraints_);
            }


            /**
             * @brief True if all constraints on this level are equalitites
             *
             * @return true/false
             */
            bool isEquality() const
            {
                return (getNumberOfConstraints() == number_of_equality_constraints_);
            }


            /**
             * @brief Form objective of a QP
             *
             * @param[out] H hessian
             * @param[out] g gradient
             */
            void getObjective(  Eigen::MatrixXd &H,
                                Eigen::VectorXd &g) const
            {
                HUMOTO_ASSERT(isEquality(), "Objective cannot be formed using inequality tasks.");

                for (   std::list<TaskInfo>::const_iterator it = tasks_.begin();
                        it != tasks_.end();
                        ++it)
                {
                    if (it == tasks_.begin())
                    {
                        it->ptr_->getATAandATb(H, g);
                    }
                    else
                    {
                        it->ptr_->addATAandATb(H, g);
                    }
                }
                etools::convertLLTtoSymmetric(H);

                g = -g;
            }


            /**
             * @brief Log hierarchy as a set of tasks
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            void    log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                           const LogEntryName & parent = LogEntryName(),
                           const std::string & name = "") const
            {
                std::size_t i = 0;

                LogEntryName subname = parent;
                subname.add(name);
                for (   std::list<TaskInfo>::const_iterator it = tasks_.begin();
                        it != tasks_.end();
                        ++it, ++i)
                {
                    LogEntryName subname_loop = subname;
                    subname_loop.add(i);
                    it->location_.log(logger, subname_loop);
                    it->ptr_->log(logger, subname_loop, "");
                }
            }
    };



    /**
     * @brief An optimization problem  [initialize_stack.m, simulation_loop.m]
     *
     * A stack of tasks / hierarchy, convertible to a QP in some cases.
     */
    class HUMOTO_LOCAL OptimizationProblem
    {
        protected:
            std::size_t     number_of_levels_;

            std::vector< humoto::HierarchyLevel >       hierarchy_;
            std::vector< std::size_t >                 number_of_constraints_;



        protected:
            /**
             * @brief Form hierarchy.
             *
             * @param[out] solution                         initialized solution
             * @param[in] model                             model of the system
             * @param[in] control_problem                   control problem associated with the model
             * @param[in] is_active_set_guessing_enabled
             */
            void    formHierarchy(  humoto::Solution                & solution,
                                    const humoto::Model             & model,
                                    const humoto::ControlProblem    & control_problem,
                                    const bool                      is_active_set_guessing_enabled)
            {
                control_problem.initSolutionStructure(solution);


                // loop over levels
                HUMOTO_ASSERT(getNumberOfLevels() > 0, "Empty hierarchy.");
                for (std::size_t i = 0; i < getNumberOfLevels(); ++i)
                {
                    hierarchy_[i].form(solution, model, control_problem, is_active_set_guessing_enabled);
                    number_of_constraints_[i] = hierarchy_[i].getNumberOfConstraints();
                }
            }


            /**
             * @brief Form active set guess
             *
             * @param[out] active_set active set guess
             */
            void guessActiveSet(ActiveSet &active_set) const
            {
                active_set.initialize(number_of_levels_, number_of_constraints_);

                for (std::size_t i = 0; i < getNumberOfLevels(); ++i)
                {
                    hierarchy_[i].guessActiveSet(active_set[i]);
                }
            }


        public:
            /**
             * @brief Default constructor
             */
            OptimizationProblem()
            {
                reset(0);
            }


            /**
             * @brief Add task to the optimization problem.
             *
             * @param[in,out] task_pointer  pointer to a task
             * @param[in] level_index       level in the hierarchy
             *
             * @todo It might be a good idea to check for collisions of task ids.
             */
            void pushTask(  TaskSharedPointer task_pointer,
                            const std::size_t level_index)
            {
                HUMOTO_ASSERT(level_index < number_of_levels_, "Wrong hierarchy level number.");
                hierarchy_[level_index].pushTask(task_pointer);
            }


            /**
             * @brief Form hierarchy.
             *
             * @param[out] solution         initialized solution
             * @param[in] model             model of the system
             * @param[in] control_problem   control problem associated with the model
             */
            void    form(   humoto::Solution &solution,
                            const humoto::Model & model,
                            const humoto::ControlProblem & control_problem)
            {
                formHierarchy(solution, model, control_problem, false);
            }


            /**
             * @brief Form hierarchy.
             *
             * @param[out] solution         initialized solution
             * @param[out] active_set_guess active set guess
             * @param[in] model             model of the system
             * @param[in] control_problem   control problem associated with the model
             */
            void    form(   humoto::Solution                & solution,
                            humoto::ActiveSet               & active_set_guess,
                            const humoto::Model             & model,
                            const humoto::ControlProblem    & control_problem)
            {
                formHierarchy(solution, model, control_problem, true);
                guessActiveSet(active_set_guess);
            }


            /**
             * @brief Form hierarchy.
             *
             * @param[out] solution         initialized solution
             * @param[out] solution_guess   guess of the solution
             * @param[out] active_set_guess active set guess
             * @param[in] model             model of the system
             * @param[in] control_problem   control problem associated with the model
             * @param[in] old_solution      old solution
             */
            void    form(   humoto::Solution                & solution,
                            humoto::Solution                & solution_guess,
                            humoto::ActiveSet               & active_set_guess,
                            const humoto::Model             & model,
                            const humoto::ControlProblem    & control_problem,
                            const humoto::Solution          & old_solution)
            {
                formHierarchy(solution, model, control_problem, true);
                control_problem.guessSolution(solution_guess, old_solution);
                guessActiveSet(active_set_guess);
            }


            /**
             * @brief Form hierarchy.
             *
             * @param[out] solution         initialized solution
             * @param[out] solution_guess   guess of the solution
             * @param[in] model             model of the system
             * @param[in] control_problem   control problem associated with the model
             * @param[in] old_solution      old solution
             */
            void    form(   humoto::Solution                & solution,
                            humoto::Solution                & solution_guess,
                            const humoto::Model             & model,
                            const humoto::ControlProblem    & control_problem,
                            const humoto::Solution          & old_solution)
            {
                formHierarchy(solution, model, control_problem, false);
                control_problem.guessSolution(solution_guess, old_solution);
            }


            /**
             * @brief Process actual active set: extract active sets of
             * individual tasks.
             *
             * @param[in] active_set
             */
            void processActiveSet(const humoto::ActiveSet & active_set)
            {
                HUMOTO_ASSERT(  (active_set.size() == getNumberOfLevels()),
                                "Wrong number of levels in the active set.");

                for (std::size_t i = 0; i < getNumberOfLevels(); ++i)
                {
                    hierarchy_[i].processActiveSet(active_set[i]);
                }
            }



            /**
             * @brief Determine active set based on the solution. This active
             * set may differ from the active set returned by the solver due to
             * tolerances.
             *
             * @param[out] active_set   active set
             * @param[in] solution      solution
             */
            void    determineActiveSet( humoto::ActiveSet               & active_set,
                                        const humoto::Solution          & solution) const
            {
                active_set.initialize(number_of_levels_, number_of_constraints_);

                for (std::size_t i = 0; i < getNumberOfLevels(); ++i)
                {
                    hierarchy_[i].determineActiveSet(active_set[i], solution);
                }
            }



            /**
             * @brief Compute violations based on the solution.
             *
             * @param[out] violations   violations
             * @param[in] solution      solution
             */
            void    computeViolations ( humoto::Violations              & violations,
                                        const humoto::Solution          & solution) const
            {
                violations.initialize(number_of_levels_, number_of_constraints_);

                for (std::size_t i = 0; i < getNumberOfLevels(); ++i)
                {
                    hierarchy_[i].computeViolations(violations[i], solution);
                }
            }


            /**
             * @brief Reset the optimization problem
             *
             * @param[in] number_of_levels  new number of levels
             */
            void reset( const std::size_t number_of_levels)
            {
                number_of_levels_       = number_of_levels;

                hierarchy_.clear();
                hierarchy_.resize(number_of_levels_);

                number_of_constraints_.resize(number_of_levels_);
            }


            /**
             * @brief Access hierarchy level (const)
             *
             * @param[in] level_index index of the level
             *
             * @return reference to a hierarchy level.
             */
            const humoto::HierarchyLevel &  operator[] (const std::size_t  level_index) const
            {
                return (hierarchy_[level_index]);
            }


            /**
             * @brief Access hierarchy level
             *
             * @param[in] level_index index of the level
             *
             * @return reference to a hierarchy level.
             */
            humoto::HierarchyLevel &  operator[] (const std::size_t  level_index)
            {
                return (hierarchy_[level_index]);
            }


            /**
             * @brief Returns number of levels in the hierarchy.
             *
             * @return number of levels in the hierarchy.
             */
            std::size_t getNumberOfLevels() const
            {
                return (number_of_levels_);
            }


            /**
             * @brief Log hierarchy as a set of tasks
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            void    log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                           const LogEntryName & parent = LogEntryName(),
                           const std::string & name = "") const
            {
                LogEntryName subname = parent;
                subname.add("tasks");

                for (std::size_t i = 0; i < getNumberOfLevels(); ++i)
                {
                    hierarchy_[i].log(logger, LogEntryName(subname).add(i), "");
                }
            }


            /**
             * @brief Initialize active set of the hierarchy.
             *
             * @param[in,out] active_set active set.
             */
            void    initializeActiveSet(humoto::ActiveSet &active_set) const
            {
                active_set.initialize(number_of_levels_, number_of_constraints_);
            }
    };
}
