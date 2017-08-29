/**
    @file
    @author  Alexander Sherikov
    @author  Don Joven Agravante
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA, 2014-2015 CNRS. Licensed under the Apache
    License, Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @brief
*/

#pragma once

namespace humoto
{
    namespace wpg04
    {
        /**
         * @brief [task_footstepbounds.m]
         */
        class HUMOTO_LOCAL TaskFootstepBounds : public humoto::TaskILU
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskILU) \
                HUMOTO_CONFIG_SCALAR_(fix_footsteps)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            //private:
            public:
                bool fix_footsteps_;


            protected:
                void setDefaults()
                {
                    TaskILU::setDefaults();
                    fix_footsteps_ = false;
                }


            public:
                TaskFootstepBounds(const bool fix_footsteps = false) : TaskILU("TaskFootstepBounds")
                {
                    fix_footsteps_ = fix_footsteps;
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::wpg04::MPCforWPG  &mpc = dynamic_cast <const humoto::wpg04::MPCforWPG &> (control_problem);

                    Location        loc_var = sol_structure.getSolutionPartLocation(FOOTPOS_VARIABLES_ID);


                    humoto::IndexVector &I  = getIndices();
                    Eigen::VectorXd &lb = getLowerBounds();
                    Eigen::VectorXd &ub = getUpperBounds();

                    I.resize(loc_var.length_);
                    lb.resize(loc_var.length_);
                    ub.resize(loc_var.length_);

                    for(std::size_t i = 0; i < mpc.preview_horizon_.variable_steps_indices_.size(); ++i)
                    {
                        I[i*2]     = loc_var.offset_ + i*2;
                        I[i*2+1]   = loc_var.offset_ + i*2 + 1;

                        lb.segment(i*2, 2) = mpc.preview_horizon_.getFootPositionBounds(i).col(0);
                        ub.segment(i*2, 2) = mpc.preview_horizon_.getFootPositionBounds(i).col(1);

                        // fixed footsteps lb = ub
                        if(fix_footsteps_)
                        {
                            std::size_t interval_index = mpc.preview_horizon_.variable_steps_indices_[i];
                            if(humoto::walking::StanceType::RSS == mpc.preview_horizon_.getWalkState(interval_index).type_)
                            {
                                lb.segment(i*2, 1) = ub.segment(i*2, 1);
                                lb.segment(i*2+1, 1) = ub.segment(i*2+1, 1);
                            }
                            else
                            {
                                lb.segment(i*2, 1) = ub.segment(i*2, 1);
                                ub.segment(i*2+1, 1) = lb.segment(i*2+1, 1);
                            }
                        }
                    }
                }


                /// @copydoc humoto::TaskBase::guessActiveSet
				void guessActiveSet(const humoto::SolutionStructure &sol_structure,
									const humoto::Model &model_base,
									const humoto::ControlProblem &control_problem)
                {
                    Location loc_var = sol_structure.getSolutionPartLocation(FOOTPOS_VARIABLES_ID);
                    std::size_t size_of_the_old_active_set = getActualActiveSet().size();


                    if (fix_footsteps_)
                    {
                        getActiveSetGuess().initialize(loc_var.length_, ConstraintActivationType::EQUALITY);
                    }
                    else
                    {
                        if (size_of_the_old_active_set == 0)
                        {
                            getActiveSetGuess().initialize(loc_var.length_, ConstraintActivationType::INACTIVE);
                        }
                        else
                        {
                            if (size_of_the_old_active_set == loc_var.length_)
                            {
                                getActiveSetGuess() = getActualActiveSet();
                            }
                            else
                            {
                                if (size_of_the_old_active_set > loc_var.length_)
                                {
                                    Location loc_act_set(2, size_of_the_old_active_set - 2);

                                    getActiveSetGuess().initialize(getActualActiveSet(), loc_act_set);
                                }
                                else
                                {
                                    Location loc_act_set(0, size_of_the_old_active_set);

                                    getActiveSetGuess().initialize(loc_var.length_, ConstraintActivationType::INACTIVE);
                                    getActiveSetGuess().copyTo(loc_act_set, getActualActiveSet());
                                }
                            }
                        }
                    }
                }
        };
    }
}
