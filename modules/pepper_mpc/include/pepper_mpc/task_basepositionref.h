/**
    @file
    @author Jan Michalczyk
    @author Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace pepper_mpc
    {
        /**
         * @brief Task for minimizing base position against base reference position
         */
        class HUMOTO_LOCAL TaskBasePositionReference: public humoto::TaskASB
        {
            public:
                explicit TaskBasePositionReference(const double gain = 0.707106781186548) 
                    : TaskASB("TaskBasePositionReference", gain)
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::pepper_mpc::MPCforMG &mpc   = dynamic_cast <const humoto::pepper_mpc::MPCforMG &> (control_problem);

                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getB();

                    setOffset(0);
                    A.noalias() = getGain()*mpc.Aps_;

                    b.resize(A.rows());

                    for (EigenIndex i = 0; i < b.rows()/2; ++i)
                    {
                        b.segment(i*2, 2) = mpc.preview_horizon_.getBaseReferencePosition(i);
                    }

                    b -= mpc.sps_;
                    b *= getGain();
                }
        };
    } //pepper
} //humoto
