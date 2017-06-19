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
         * @brief [task_cop.m]
         */
        class TaskCoPPosition : public humoto::TaskAB
        {
            public:
                TaskCoPPosition(const double gain = 5.0) 
                    : TaskAB("TaskCoPPosition", gain)
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
                    Eigen::VectorXd &b = getB();

                    A.setZero(mpc.Sz_.rows(), sol_structure.getNumberOfVariables());

                    A = getGain() * mpc.Sz_;
                    b = -getGain() * mpc.sz_;
                };
        };
    }
}
