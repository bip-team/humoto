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
         * @brief base acceleration bounding task
         */
        class HUMOTO_LOCAL TaskBaseAccelerationBounds: public humoto::TaskASLU
        {
            public:
                TaskBaseAccelerationBounds() : TaskASLU("TaskBaseAccelerationBounds", 0.0)
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::pepper_mpc::MPCforMG  &mpc   = dynamic_cast <const humoto::pepper_mpc::MPCforMG&>(control_problem);

                    Eigen::MatrixXd &A  = getA();
                    Eigen::VectorXd &lb = getLowerBounds();
                    Eigen::VectorXd &ub = getUpperBounds();

                    setOffset(0);
                    A.noalias() = mpc.Aas_;

                    const std::size_t preview_horizon_length = mpc.preview_horizon_.getPreviewHorizonLength();
                    lb.resize(preview_horizon_length * 2);
                    ub.resize(preview_horizon_length * 2);

                    for (std::size_t i = 0; i < preview_horizon_length; ++i)
                    {
                        lb.segment(i*2, 2) = mpc.preview_horizon_.getNominalBaseAccelerationBounds(i).col(0);
                        ub.segment(i*2, 2) = mpc.preview_horizon_.getNominalBaseAccelerationBounds(i).col(1);
                    }

                    lb -= mpc.sas_;
                    ub -= mpc.sas_;
                }


                /// @copydoc humoto::TaskBase::guessActiveSet
                void guessActiveSet(const humoto::SolutionStructure &sol_structure,
                                    const humoto::Model &model_base,
                                    const humoto::ControlProblem &control_problem)
                {
                    const humoto::pepper_mpc::MPCforMG &mpc   = dynamic_cast <const humoto::pepper_mpc::MPCforMG &> (control_problem);

                    if (mpc.mpc_parameters_.sampling_time_ms_ > mpc.mpc_parameters_.subsampling_time_ms_)
                    {
                        TaskBase::guessActiveSet(sol_structure, model_base, control_problem);
                    }
                    else
                    {
                        std::size_t num_ctr = mpc.preview_horizon_.getPreviewHorizonLength() * 2;

                        if (getActualActiveSet().size() == 0)
                        {
                            getActiveSetGuess().initialize(num_ctr, ConstraintActivationType::INACTIVE);
                        }
                        else
                        {
                            HUMOTO_ASSERT(  (getActualActiveSet().size() == num_ctr),
                                            "The number of base velocity variables is not supposed to change.");

                            getActiveSetGuess() = getActualActiveSet();

                            getActiveSetGuess().shift(2, ConstraintActivationType::INACTIVE);
                        }
                    }
                }
        };
    }//pepper
}//humoto
