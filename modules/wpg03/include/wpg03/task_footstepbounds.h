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
         * @brief [task_footstepbounds.m]
         */
        class TaskFootstepBounds : public humoto::TaskILU
        {
            private:
                bool fix_footsteps_;


            public:
                TaskFootstepBounds(const bool fix_footsteps = false) : TaskILU("TaskFootstepBounds")
                {
                    fix_footsteps_ = fix_footsteps;
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


                    Location loc_var = sol_structure.getSolutionPartLocation(FOOTPOS_VARIABLES_ID);

                    humoto::IndexVector &I  = getIndices();
                    Eigen::VectorXd &lb = getLowerBounds();
                    Eigen::VectorXd &ub = getUpperBounds();

                    I.resize(loc_var.length_);
                    lb.resize(loc_var.length_);
                    ub.resize(loc_var.length_);


                    for(unsigned int i = 0; i < mpc.preview_horizon_.variable_steps_indices_.size(); ++i)
                    {
                        I[i*2]     = loc_var.offset_ + i*2;
                        I[i*2+1]   = loc_var.offset_ + i*2 + 1;

                        unsigned int state_index = mpc.preview_horizon_.intervals_[mpc.preview_horizon_.variable_steps_indices_[i]].state_index_;

                        lb.segment(i*2, 2) = mpc.preview_horizon_.states_[state_index].fd_bounds_.col(0);
                        ub.segment(i*2, 2) = mpc.preview_horizon_.states_[state_index].fd_bounds_.col(1);

                        // fixed footsteps lb = ub
                        if(fix_footsteps_)
                        {
                            if(mpc.preview_horizon_.states_[i].type_ == STATE_RSS)
                            {
                                lb.segment(loc_var.length_-2, 1) = ub.segment(loc_var.length_-2, 1);
                                ub.segment(loc_var.length_-1, 1) = lb.segment(loc_var.length_-1, 1);
                            }
                            else
                            {
                                lb.segment(loc_var.length_-2, 1) = ub.segment(loc_var.length_-2, 1);
                                lb.segment(loc_var.length_-1, 1) = ub.segment(loc_var.length_-1, 1);
                            }
                        }
                    }
                }
        };
    }
}
