/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief  Solver interface classes
*/

#pragma once

namespace humoto
{
    /**
     * @brief Parameters of a solver
     */
    class HUMOTO_LOCAL SolverParametersBase : public humoto::config::ConfigurableBase
    {
        #define HUMOTO_CONFIG_SECTION_ID "SolverParametersBase"
        #define HUMOTO_CONFIG_CONSTRUCTOR SolverParametersBase
        #define HUMOTO_CONFIG_ENTRIES \
            HUMOTO_CONFIG_SCALAR_(crash_on_any_failure) \
            HUMOTO_CONFIG_SCALAR_(solve_two_levels_as_qp)
        #include HUMOTO_CONFIG_DEFINE_ACCESSORS


        protected:
            void setDefaults()
            {
                crash_on_any_failure_ = true;
                solve_two_levels_as_qp_ = true;
            }


            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~SolverParametersBase() {}

            SolverParametersBase()
            {
                setDefaults();
            }


        public:
            /// Throw an exception if solver does not return sucessfully.
            bool crash_on_any_failure_;

            /// Enable conversion of an optimization problem with two priority levels to a QP.
            bool solve_two_levels_as_qp_;
    };



    /**
     * @brief Base solver class
     *
     * @tparam t_SolverParameters class representing parameters of the solver,
     * should inherit from #SolverParametersBase
     */
    template <class t_SolverParameters>
        class HUMOTO_LOCAL Solver
    {
        protected:
            t_SolverParameters  parameters_;


        protected:
            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~Solver() {}
            Solver() {}


            /**
             * @brief Check return status and crash if it indicates a failure
             * and crashing is enabled.
             *
             * @param[in] solution
             */
            void checkResults(const humoto::Solution   &solution) const
            {
                if ((true == parameters_.crash_on_any_failure_)
                        &&
                    (SolverStatus::OK != solution.return_status_))
                {
                    HUMOTO_THROW_MSG(std::string("Solver failed: ") + solution.getStatusDescription());
                }
            }


            /**
             * @brief Checks input, does nothing by default, may include extra
             * checks in mixins.
             *
             * @param[in] hierarchy
             */
            virtual void checkInputs(const humoto::OptimizationProblem   &hierarchy) const
            {
            }


            /**
             * @brief Reset solver
             */
            virtual void reset()
            {
            }


            /**
             * @brief Initialize solver
             *
             * @param[in] hierarchy hierarchy
             * @param[in] sol_structure solution structure
             */
            virtual void initialize(  const humoto::OptimizationProblem   &hierarchy,
                                      const humoto::SolutionStructure     &sol_structure) = 0;

            /**
             * @brief Solve the hierarchy
             *
             * @param[out] solution solution
             * @param[in] hierarchy hierarchy
             */
            virtual void solveHierarchy(humoto::Solution                    &solution,
                                        const humoto::OptimizationProblem   &hierarchy) = 0;



        public:
            /**
             * @brief Construct solver with specified parameters.
             *
             * @param[in] parameters parameters
             */
            void setParameters(const t_SolverParameters &parameters)
            {
                parameters_ = parameters;
                reset();
            }


            /**
             * @brief Solve an optimization problem (using previously
             * specified parameters)
             *
             * @param[out] solution
             * @param[in] hierarchy
             */
            void solve( humoto::Solution                    &solution,
                        const humoto::OptimizationProblem   &hierarchy)
            {
                checkInputs(hierarchy);
                initialize(hierarchy, solution);
                solveHierarchy(solution, hierarchy);
                checkResults(solution);
            }


            /**
             * @brief Solve an optimization problem (using previously
             * specified parameters)
             *
             * @param[out] solution
             * @param[in] hierarchy
             * @param[in] parameters parameters
             */
            void solve( humoto::Solution                    &solution,
                        const humoto::OptimizationProblem   &hierarchy,
                        const t_SolverParameters            &parameters)
            {
                parameters_ = parameters;

                checkInputs(hierarchy);
                initialize(hierarchy, solution);
                solveHierarchy(solution, hierarchy);
                checkResults(solution);
            }
    };



    /**
     * @brief Mixin solver interface (solution guessing)
     *
     * @tparam t_Solver base solver class
     */
    template <class t_Solver>
        class HUMOTO_LOCAL SolverGuessSolutionMixin : public t_Solver
    {
        protected:
            using t_Solver::initialize;
            using t_Solver::solveHierarchy;
            using t_Solver::checkResults;
            using t_Solver::checkInputs;

            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~SolverGuessSolutionMixin() {}
            SolverGuessSolutionMixin() {}


            /**
             * @brief Set solution guess
             *
             * @param[in] solution_guess
             */
            virtual void setSolutionGuess(const humoto::Solution & solution_guess) = 0;


        public:
            using t_Solver::solve;


            /**
             * @brief Solve an optimization problem (using previously
             * specified parameters)
             *
             * @param[out] solution
             * @param[in] hierarchy
             * @param[in] solution_guess
             */
            void solve( humoto::Solution                    &solution,
                        const humoto::OptimizationProblem   &hierarchy,
                        const humoto::Solution              &solution_guess)
            {
                checkInputs(hierarchy);
                initialize(hierarchy, solution);
                setSolutionGuess(solution_guess);
                solveHierarchy(solution, hierarchy);
                checkResults(solution);
            }
    };



    /**
     * @brief Mixin solver interface (active set guessing)
     *
     * @tparam t_Solver base solver class
     */
    template <class t_Solver>
        class HUMOTO_LOCAL SolverGuessActiveSetMixin : public t_Solver
    {
        protected:
            using t_Solver::initialize;
            using t_Solver::solveHierarchy;
            using t_Solver::checkResults;
            using t_Solver::checkInputs;

            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~SolverGuessActiveSetMixin() {}
            SolverGuessActiveSetMixin() {}


            /**
             * @brief Set active set guess
             *
             * @param[in] hierarchy
             * @param[in] active_set
             */
            virtual void setActiveSet(  const humoto::ActiveSet             &active_set,
                                        const humoto::OptimizationProblem   &hierarchy) = 0;

            /**
             * @brief Get active set
             *
             * @param[out] active_set
             * @param[in] hierarchy
             */
            virtual void getActiveSet(  humoto::ActiveSet                   &active_set,
                                        const humoto::OptimizationProblem   &hierarchy) = 0;

        public:
            using t_Solver::solve;


            /**
             * @brief Solve an optimization problem (using previously
             * specified parameters)
             *
             * @param[out] solution
             * @param[out] active_set
             * @param[in] hierarchy
             * @param[in] active_set_guess
             */
            void solve( Solution                            &solution,
                        ActiveSet                           &active_set,
                        const humoto::OptimizationProblem   &hierarchy,
                        const ActiveSet                     &active_set_guess)
            {
                checkInputs(hierarchy);
                initialize(hierarchy, solution);
                setActiveSet(active_set_guess, hierarchy);
                solveHierarchy(solution, hierarchy);
                checkResults(solution);
                getActiveSet(active_set, hierarchy);
            }


            /**
             * @brief Solve an optimization problem (using previously
             * specified parameters)
             *
             * @param[out] solution
             * @param[in] hierarchy
             * @param[in] active_set_guess
             */
            void solve( Solution                            &solution,
                        const humoto::OptimizationProblem   &hierarchy,
                        const ActiveSet                     &active_set_guess)
            {
                checkInputs(hierarchy);
                initialize(hierarchy, solution);
                setActiveSet(active_set_guess, hierarchy);
                solveHierarchy(solution, hierarchy);
                checkResults(solution);
            }


            /**
             * @brief Solve an optimization problem (using previously
             * specified parameters)
             *
             * @param[out] solution
             * @param[out] active_set
             * @param[in] hierarchy
             */
            void solve( humoto::Solution                    &solution,
                        humoto::ActiveSet                   &active_set,
                        const humoto::OptimizationProblem   &hierarchy)
            {
                checkInputs(hierarchy);
                initialize(hierarchy, solution);
                solveHierarchy(solution, hierarchy);
                checkResults(solution);
                getActiveSet(active_set, hierarchy);
            }
    };



    /**
     * @brief Mixin solver interface (active set and solution guessing)
     *
     * @tparam t_Solver base solver class
     */
    template <class t_Solver>
        class HUMOTO_LOCAL SolverGuessSolutionActiveSetMixin : public t_Solver
    {
        protected:
            using t_Solver::initialize;
            using t_Solver::solveHierarchy;
            using t_Solver::checkResults;
            using t_Solver::checkInputs;
            using t_Solver::getActiveSet;
            using t_Solver::setActiveSet;
            using t_Solver::setSolutionGuess;


            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~SolverGuessSolutionActiveSetMixin() {}
            SolverGuessSolutionActiveSetMixin() {}


        public:
            using t_Solver::solve;


            /**
             * @brief Solve an optimization problem (using previously
             * specified parameters)
             *
             * @param[out] solution
             * @param[out] active_set
             * @param[in] hierarchy
             * @param[in] solution_guess
             * @param[in] active_set_guess
             */
            void solve( Solution                            &solution,
                        ActiveSet                           &active_set,
                        const humoto::OptimizationProblem   &hierarchy,
                        const humoto::Solution              &solution_guess,
                        const ActiveSet                     &active_set_guess)
            {
                checkInputs(hierarchy);
                initialize(hierarchy, solution);
                setSolutionGuess(solution_guess);
                setActiveSet(active_set_guess, hierarchy);
                solveHierarchy(solution, hierarchy);
                checkResults(solution);
                getActiveSet(active_set, hierarchy);
            }


            /**
             * @brief Solve an optimization problem (using previously
             * specified parameters)
             *
             * @param[out] solution
             * @param[in] hierarchy
             * @param[in] solution_guess
             * @param[in] active_set_guess
             */
            void solve( Solution                            &solution,
                        const humoto::OptimizationProblem   &hierarchy,
                        const humoto::Solution              &solution_guess,
                        const ActiveSet                     &active_set_guess)
            {
                checkInputs(hierarchy);
                initialize(hierarchy, solution);
                setSolutionGuess(solution_guess);
                setActiveSet(active_set_guess, hierarchy);
                solveHierarchy(solution, hierarchy);
                checkResults(solution);
            }


            /**
             * @brief Solve an optimization problem (using previously
             * specified parameters)
             *
             * @param[out] solution
             * @param[out] active_set
             * @param[in] hierarchy
             * @param[in] solution_guess
             */
            void solve( humoto::Solution                    &solution,
                        humoto::ActiveSet                   &active_set,
                        const humoto::OptimizationProblem   &hierarchy,
                        const humoto::Solution              &solution_guess)
            {
                checkInputs(hierarchy);
                initialize(hierarchy, solution);
                setSolutionGuess(solution_guess);
                solveHierarchy(solution, hierarchy);
                checkResults(solution);
                getActiveSet(active_set, hierarchy);
            }
    };


    /**
     * @brief Mixin solver interface (QP solver)
     *
     * @tparam t_Solver base solver class
     */
    template <class t_Solver>
        class HUMOTO_LOCAL QPSolverMixin : public t_Solver
    {
        protected:
            using   t_Solver::parameters_;


        protected:
            virtual void checkInputs(const humoto::OptimizationProblem   &hierarchy) const
            {
                std::cout << "Check Inputs" << std::endl;
                std::cout << "parameters_.solve_two_levels_as_qp_: " << parameters_.solve_two_levels_as_qp_ << std::endl;
                HUMOTO_ASSERT(  true == parameters_.solve_two_levels_as_qp_,
                                "Interpretation of hierarchies as QPs is disabled.");

                std::size_t     number_of_levels = hierarchy.getNumberOfLevels();

                HUMOTO_ASSERT(  (number_of_levels <= 2) && (number_of_levels >= 1),
                                "This type of hierarchy cannot be solved with a QP solver.");


                std::size_t     objective_level = number_of_levels - 1;

                HUMOTO_ASSERT(  hierarchy[objective_level].isEquality(),
                                "This type of hierarchy cannot be solved with a QP solver.");

                HUMOTO_ASSERT(  hierarchy[objective_level].getNumberOfConstraints() > 0,
                                "Empty objective.");
            }


            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~QPSolverMixin() {}
            QPSolverMixin() {}
    };



    /**
     * @brief Solver class with solution guessing
     *
     * @tparam t_SolverParameters class representing parameters of the solver,
     * should inherit from #SolverParametersBase
     */
    template <class t_SolverParameters>
        class HUMOTO_LOCAL SolverGuessSolution : public SolverGuessSolutionMixin< Solver<t_SolverParameters> >
    {
        protected:
            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~SolverGuessSolution() {}
            SolverGuessSolution() {}
    };


    /**
     * @brief Solver class with active set guessing
     *
     * @tparam t_SolverParameters class representing parameters of the solver,
     * should inherit from #SolverParametersBase
     */
    template <class t_SolverParameters>
        class HUMOTO_LOCAL SolverGuessActiveSet : public SolverGuessActiveSetMixin< Solver<t_SolverParameters> >
    {
        protected:
            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~SolverGuessActiveSet() {}
            SolverGuessActiveSet() {}
    };


    /**
     * @brief Solver class with active set and solution guessing
     *
     * @tparam t_SolverParameters class representing parameters of the solver,
     * should inherit from #SolverParametersBase
     */
    template <class t_SolverParameters>
        class HUMOTO_LOCAL SolverGuessSolutionActiveSet
            : public SolverGuessSolutionActiveSetMixin<
                        SolverGuessSolutionMixin<
                            SolverGuessActiveSetMixin<
                                Solver<t_SolverParameters> > > >
    {
        protected:
            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~SolverGuessSolutionActiveSet() {}
            SolverGuessSolutionActiveSet() {}
    };
}
