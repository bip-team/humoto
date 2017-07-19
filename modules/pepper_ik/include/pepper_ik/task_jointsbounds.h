/**
    @file
    @author Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace pepper_ik
    {
        /**
         * @brief Bound joint angles
         */
        template <int t_features>
            class HUMOTO_LOCAL TaskJointsBounds : public humoto::TaskILU
        {
            public:
                TaskJointsBounds () : TaskILU("TaskJointsBounds")
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::pepper_ik::Model<t_features>  &model   =
                        dynamic_cast <const humoto::pepper_ik::Model<t_features> &>(model_base);

                    Location loc_var = sol_structure.getSolutionPartLocation(pepper_ik::JOINTS_VARIABLES_ID);

                    humoto::IndexVector &I     = getIndices();
                    Eigen::VectorXd     &lb    = getLowerBounds();
                    Eigen::VectorXd     &ub    = getUpperBounds();

                    I.resize(loc_var.length_);

                    for (std::size_t i = 0; i < loc_var.length_; ++i)
                    {
                        I[i]     = loc_var.offset_ + i;
                    }

                    lb =    model.constraints_.joint_position_bounds_min_
                            -
                            model.getState().joint_angles_;
                    ub =    model.constraints_.joint_position_bounds_max_
                            -
                            model.getState().joint_angles_;
                }
        };
    } //pepper
} //humoto
