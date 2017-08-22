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
     * @ingroup Tasks
     *
     * @defgroup PredefinedTasks    Predefined tasks
     * @brief   Predefined tasks
     */

    /**
     * @brief [task_zerovars.m] Set given variables to zero, i.e. minimize these variables.
     * @ingroup PredefinedTasks
     */
    class HUMOTO_LOCAL TaskZeroVariables : public humoto::TaskGIB0
    {
        #define HUMOTO_CONFIG_ENTRIES \
            HUMOTO_CONFIG_PARENT_CLASS(TaskGIB0) \
            HUMOTO_CONFIG_SCALAR_(variables_id)
        #include HUMOTO_CONFIG_DEFINE_ACCESSORS


        private:
            Location        location_of_variables_;

            std::string     variables_id_;


        public:
            /**
             * @brief Set variables string id to a given value
             *
             * @param[in] variables_id string id of variables
             */
            void setVariablesID(const std::string& variables_id)
            {
                variables_id_ = variables_id;
            }


            /**
             * @brief Constructor
             *
             * @param[in] gain          gain, by default is 2 so that 'sqrt(gain/2) = 1'.
             * @param[in] task_id       string id of the task.
             * @param[in] variables_id  string id of variables: if empty, all variables are minimized.
             */
            TaskZeroVariables(
                    const double gain = 2.0,
                    const std::string &task_id = "Minimize_All_Variables",
                    const std::string &variables_id = "")
                : TaskGIB0(task_id.c_str(), gain)
            {
                location_of_variables_.set(0,0);

                variables_id_ = variables_id;
            }


            /**
             * @brief Form task
             *
             * @param[in] sol_structure     structure of the solution
             * @param[in] model_base        model
             * @param[in] control_problem   control problem
             */
            void form(  const humoto::SolutionStructure &sol_structure,
                        const humoto::Model &model_base,
                        const humoto::ControlProblem &control_problem)
            {
                Location location_of_variables;

                if (variables_id_.size() == 0)
                {
                    location_of_variables.set(0, sol_structure.getNumberOfVariables());
                }
                else
                {
                    location_of_variables = sol_structure.getSolutionPartLocation(variables_id_);
                }


                if ( location_of_variables != location_of_variables_ )
                {
                    // Task was modified
                    location_of_variables_ = location_of_variables;


                    humoto::IndexVector &I  = getIndices();
                    Eigen::VectorXd &gains = getIGains();

                    gains.setConstant(location_of_variables_.length_, getGain());

                    I.resize(location_of_variables_.length_);
                    for (std::size_t i = 0; i < location_of_variables_.length_; ++i)
                    {
                        I[i]     = location_of_variables_.offset_ + i;
                    }
                }
                else
                {
                    // Task is not modified.
                    // In this particular case performance is hardly
                    // improved. This code is just a demo.
                    markAsUnmodified();
                }
            }
    };


    /**
     * @brief Same as humoto::TaskZeroVariables, but the variables need not to
     * be continuous.
     * @ingroup PredefinedTasks
     */
    class HUMOTO_LOCAL TaskZeroSelectedVariables : public humoto::TaskGIB0
    {
        #define HUMOTO_CONFIG_ENTRIES \
            HUMOTO_CONFIG_PARENT_CLASS(TaskGIB0) \
            HUMOTO_CONFIG_COMPOUND_(variables_indices)
        #include HUMOTO_CONFIG_DEFINE_ACCESSORS


        private:
            humoto::IndexVector variables_indices_;


        public:
            /**
             * @brief Constructor
             *
             * @param[in] var_indices   indices of variables
             * @param[in] gain          gain, by default is 2 so that 'sqrt(gain/2) = 1'.
             * @param[in] task_id       string id of the task.
             */
            TaskZeroSelectedVariables(
                    const humoto::IndexVector &var_indices,
                    const double gain = 2.0,
                    const std::string &task_id = "Minimize_Selected_Variables")
                : TaskGIB0(task_id.c_str(), gain)
            {
                variables_indices_ = var_indices;
            }


            /**
             * @brief Form task
             *
             * @param[in] sol_structure     structure of the solution
             * @param[in] model_base        model
             * @param[in] control_problem   control problem
             */
            void form(  const humoto::SolutionStructure &sol_structure,
                        const humoto::Model &model_base,
                        const humoto::ControlProblem &control_problem)
            {
                getIndices() = variables_indices_;

                Eigen::VectorXd &gains = getIGains();
                gains.setConstant(variables_indices_.rows(), getGain());
            }
    };


    /**
     * @brief Infeasible inequality task for testing purposes.
     * @ingroup PredefinedTasks
     */
    class HUMOTO_LOCAL TaskInfeasibleInequality : public humoto::TaskALU
    {
        private:
            std::size_t number_of_variables_;

        public:
            /**
             * @brief Constructor
             *
             * @param[in] gain          gain, by default is 2 so that 'sqrt(gain/2) = 1'.
             * @param[in] task_id       string id of the task.
             */
            TaskInfeasibleInequality(
                    const double gain = 2.0,
                    const std::string &task_id = "Infeasible_Inequality_Task")
                : TaskALU(task_id.c_str(), gain)
            {
                number_of_variables_ = 0;
            }


            /**
             * @brief Form task
             *
             * @param[in] sol_structure     structure of the solution
             * @param[in] model_base        model
             * @param[in] control_problem   control problem
             */
            void form(  const humoto::SolutionStructure &sol_structure,
                        const humoto::Model &model_base,
                        const humoto::ControlProblem &control_problem)
            {
                if (number_of_variables_ != sol_structure.getNumberOfVariables())
                {
                    number_of_variables_ = sol_structure.getNumberOfVariables();

                    getA().resize(1, number_of_variables_);
                    getA().setConstant(1);

                    getLowerBounds().resize(1);
                    getUpperBounds().resize(1);

                    getLowerBounds()[0] = 1;
                    getUpperBounds()[0] = -1;
                }
                else
                {
                    // Task is not modified.
                    // In this particular case perfromance is hardly
                    // improved. This code is just a demo.
                    markAsUnmodified();
                }
            }
    };
}
