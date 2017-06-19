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
         * @brief Impedance control on the Center of Mass
         *        position, velocity and acceleration at each time instant
         */
        class TaskCoMImpedance: public humoto::TaskAB
        {
            public:
                TaskCoMImpedance(const double gain = 0.707106781186548)
                    : TaskAB("TaskCoMImpedance", gain)
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

                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getB();

                    A.setZero(mpc.S_.rows(), sol_structure.getNumberOfVariables());

                    A = getGain() * mpc.Gmbk_ * mpc.S_;
                    b = getGain() * (mpc.Select_Fxy_ - mpc.Gmbk_ * mpc.s_);
                };
        };
    }
}
