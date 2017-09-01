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



            /**
             * @brief Form a QP problem.
             *
             * @tparam t_QPConstraints QP constraints type
             *
             * @param[in] qp_problem        QP problem
             * @param[in] sol_structure     solution structure
             * @param[in] initialize_upper_triangular_part of the Hessian
             */
            template<class t_QPConstraints>
                void getQPProblem(  QPProblemBase<t_QPConstraints> & qp_problem,
                                    const humoto::SolutionStructure & sol_structure,
                                    const bool initialize_upper_triangular_part = true)
            {
                std::size_t    number_of_levels = getNumberOfLevels();

                HUMOTO_ASSERT(  (number_of_levels <= 2) && (number_of_levels >= 1),
                                "Too many or too few levels.");


                std::size_t     objective_level = number_of_levels - 1;

                qp_problem.initializeObjective(hierarchy_[objective_level], initialize_upper_triangular_part);

                if (number_of_levels > 1)
                {
                    qp_problem.initializeConstraints(hierarchy_[0], sol_structure);
                }
            }
    };
}
