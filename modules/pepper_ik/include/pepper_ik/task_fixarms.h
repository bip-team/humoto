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
         * @brief Prevents motion of arms
         */
        template <int t_features>
            class HUMOTO_LOCAL TaskFixArms : public humoto::TaskIB0
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskIB0)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            public:
                TaskFixArms() : TaskIB0("TaskFixArms")
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    Location loc_var = sol_structure.getSolutionPartLocation(pepper_ik::JOINTS_VARIABLES_ID);

                    humoto::IndexVector &I     = getIndices();

                    I.resize(10);

                    I[0] = loc_var.offset_ + ModelDescription<t_features>::LShoulderPitch;
                    I[1] = loc_var.offset_ + ModelDescription<t_features>::LShoulderRoll ;
                    I[2] = loc_var.offset_ + ModelDescription<t_features>::LElbowYaw     ;
                    I[3] = loc_var.offset_ + ModelDescription<t_features>::LElbowRoll    ;
                    I[4] = loc_var.offset_ + ModelDescription<t_features>::LWristYaw     ;
                                          
                    I[5] = loc_var.offset_ + ModelDescription<t_features>::RShoulderPitch;
                    I[6] = loc_var.offset_ + ModelDescription<t_features>::RShoulderRoll ;
                    I[7] = loc_var.offset_ + ModelDescription<t_features>::RElbowYaw     ;
                    I[8] = loc_var.offset_ + ModelDescription<t_features>::RElbowRoll    ;
                    I[9] = loc_var.offset_ + ModelDescription<t_features>::RWristYaw     ;
                }
        };
    } //pepper
} //humoto
