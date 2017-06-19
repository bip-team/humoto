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
        class TaskCoPBounds : public humoto::TaskALU
        {
            public:
                TaskCoPBounds() : TaskALU("TaskCoPBounds")
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

                    Eigen::MatrixXd &A  = getA();
                    Eigen::VectorXd &lb = getLowerBounds();
                    Eigen::VectorXd &ub = getUpperBounds();

                    A.setZero(mpc.Sz_.rows(), sol_structure.getNumberOfVariables());
                    lb.setZero(mpc.Sz_.rows());
                    ub.setZero(mpc.Sz_.rows());

                    A = mpc.Sz_;

                    for (EigenIndex i = 0; i < mpc.Sz_.rows()/2; ++i)
                    {
                        unsigned int state_index = mpc.preview_horizon_.intervals_[i].state_index_;

                        lb.segment(i*2, 2) = mpc.preview_horizon_.states_[state_index].cop_bounds_.col(0);
                        ub.segment(i*2, 2) = mpc.preview_horizon_.states_[state_index].cop_bounds_.col(1);
                    }
                    lb = lb - mpc.sz_;
                    ub = ub - mpc.sz_;
                }
        };
    }
}
