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
         * @brief Task for bounding base velocity to a square
         */
        class HUMOTO_LOCAL TaskBaseVelocityBounds: public humoto::TaskILU
        {
            public:
                TaskBaseVelocityBounds() : TaskILU("TaskBaseVelocityBounds")
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::pepper_mpc::MPCforMG  &mpc = dynamic_cast <const humoto::pepper_mpc::MPCforMG &> (control_problem);

                    Location loc_var = sol_structure.getSolutionPartLocation(BASE_VEL_VARIABLES_ID);

                    humoto::IndexVector &I     = getIndices();
                    Eigen::VectorXd     &lb    = getLowerBounds();
                    Eigen::VectorXd     &ub    = getUpperBounds();

                    I.resize (loc_var.length_);
                    lb.resize(loc_var.length_);
                    ub.resize(loc_var.length_);

                    for (std::size_t i = 0; i < loc_var.length_/2; ++i)
                    {
                        I[i*2]     = loc_var.offset_ + i*2;
                        I[i*2+1]   = loc_var.offset_ + i*2 + 1;

                        lb.segment(i*2, 2) = mpc.preview_horizon_.getNominalBaseVelocityBounds(i).col(0);
                        ub.segment(i*2, 2) = mpc.preview_horizon_.getNominalBaseVelocityBounds(i).col(1);
                    }
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
    } //pepper
} //humoto
