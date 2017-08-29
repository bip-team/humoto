/**
    @file
    @author Jan Michalczyk
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
         * @brief Terminal constraint task
         */
        class HUMOTO_LOCAL TaskTerminalConstraint: public humoto::TaskAB
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskAB)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            protected:
                void setDefaults()
                {
                    TaskAB::setDefaults();
                    setGain(1.0);
                }


                void finalize()
                {
                    TaskAB::finalize();
                }


            public:
                TaskTerminalConstraint(const double gain = 1.0)
                    : TaskAB("TaskTerminalConstraint", gain)
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::wpg04::MPCforWPG  &mpc   = dynamic_cast <const humoto::wpg04::MPCforWPG &> (control_problem);
                    const humoto::wpg04::Model      &model_wpg04 = dynamic_cast <const humoto::wpg04::Model &>     (model_base);

                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getB();

                    const std::size_t preview_length = mpc.getPreviewHorizonLength();
                    const std::size_t cstate_length  = model_wpg04.getCState().size();

                    etools::Matrix2x6 D = model_wpg04.getDcpv6(mpc.preview_horizon_.intervals_.back().omega_);

                    A.noalias() =  getGain() * D * mpc.S_.block((preview_length - 1)*cstate_length, 0, cstate_length, sol_structure.getNumberOfVariables());
                    b.noalias() = -getGain() * D * mpc.s_.block((preview_length - 1)*cstate_length, 0, cstate_length, 1);
                };
        };
    }
}
