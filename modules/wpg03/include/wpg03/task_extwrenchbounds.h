/**
    @file
    @author  Don Joven Agravante
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA, 2014-2015 CNRS. Licensed under the Apache
    License, Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace wpg03
    {
        /**
         * @brief [task_copbounds.m]
         */
        class TaskExtWrenchBounds : public humoto::TaskILU
        {
            public:
                TaskExtWrenchBounds() : TaskILU("TaskExtWrenchBounds")
                {
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
                    const humoto::wpg03::MPCforWPG  &mpc = dynamic_cast <const humoto::wpg03::MPCforWPG &> (control_problem);

                    Location loc_var = sol_structure.getSolutionPartLocation(EXTWRENCH_VARIABLES_ID);

                    humoto::IndexVector &I  = getIndices();
                    Eigen::VectorXd &lb = getLowerBounds();
                    Eigen::VectorXd &ub = getUpperBounds();

                    I.resize(loc_var.length_);
                    lb.resize(loc_var.length_);
                    ub.resize(loc_var.length_);

                    for (unsigned int i = 0; i < loc_var.length_/4; ++i)
                    {
                        for(unsigned int j=0; j<4; ++j)
                        {
                            I[i*4 + j] = loc_var.offset_ + i*4 + j;
                        }
                        lb.segment(i*4, 4) = -mpc.wrench_bounds;
                        ub.segment(i*4, 4) = mpc.wrench_bounds;
                    }
                }
        };
    }
}
