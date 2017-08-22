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
    namespace wpg04
    {
        /**
         * @brief [task_cvel.m]
         */
        class HUMOTO_LOCAL TaskCoMVelocity: public humoto::TaskAB
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskAB)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            protected:
                void setDefaults()
                {
                    TaskAB::setDefaults();
                    setGain(0.707106781186548);
                }


                void finalize()
                {
                    TaskAB::finalize();
                }


            public:
                TaskCoMVelocity(const double gain = 0.707106781186548) : TaskAB("TaskCoMVelocity", gain)
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::wpg04::MPCforWPG  &mpc = dynamic_cast <const humoto::wpg04::MPCforWPG &> (control_problem);


                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getB();

                    Eigen::VectorXd cvel_ref;
                    cvel_ref.resize(mpc.getPreviewHorizonLength()*2);

                    for (std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
                    {
                        cvel_ref.segment(i*2, 2) = mpc.preview_horizon_.intervals_[i].cvel_ref_;
                    }

                    A.noalias() = getGain() * (mpc.velocity_selector_ * mpc.S_); // Sv

                    b.noalias() = -getGain() * (mpc.velocity_selector_ * mpc.s_ /*sv*/ - cvel_ref);
                };
        };
    }
}
