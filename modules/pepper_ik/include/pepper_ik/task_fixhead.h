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
         * @brief Prevents motion of the head
         */
        template <int t_features>
            class HUMOTO_LOCAL TaskFixHead : public humoto::TaskIB0
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskIB0)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            public:
                TaskFixHead() : TaskIB0("TaskFixHead")
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    Location loc_var = sol_structure.getSolutionPartLocation(pepper_ik::JOINTS_VARIABLES_ID);

                    humoto::IndexVector &I     = getIndices();

                    I.resize(2);

                    I[0] = loc_var.offset_ + ModelDescription<t_features>::HeadYaw;
                    I[1] = loc_var.offset_ + ModelDescription<t_features>::HeadPitch;
                }
        };
    } //pepper
} //humoto
