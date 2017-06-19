/**
    @file
    @author  Don Joven Agravante
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA, 2014-2015 CNRS. Licensed under the Apache
    License, Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "point_mass_model_6z.h"
#include "robot.h"

namespace humoto
{
    namespace wpg03
    {
        /**
         * @brief [initialize_model.m]
         */
        class Model : public PointMassModel6z, public Robot, public humoto::Model
        {
            public:
                etools::Vector2                 current_support_position_;

                humoto::wpg03::ModelStateType   state_type_;
                humoto::wpg03::ModelStateType   next_state_type_;



                /**
                 * @brief Default constructor
                 */
                Model()
                {
                    state_type_         = humoto::wpg03::STATE_DS;
                    next_state_type_    = humoto::wpg03::STATE_UNDEFINED;

                    determineSupportPosition();
                }



                /**
                 * @brief Constructor
                 *
                 * @param[in] robot_parameters parameters of the robot
                 */
                Model(const RobotParameters &robot_parameters) : Robot(robot_parameters)
                {
                    state_type_         = humoto::wpg03::STATE_DS;
                    next_state_type_    = humoto::wpg03::STATE_UNDEFINED;

                    determineSupportPosition();
                }


                /**
                 * @brief Update model state.
                 *
                 * @param[in] model_state model state.
                 */
                void    updateState(const humoto::ModelState &model_state)
                {
                    const humoto::wpg03::ModelState &state = dynamic_cast <const humoto::wpg03::ModelState &> (model_state);

                    humoto::wpg03::ModelStateType   prev_state_type = state_type_;

                    cstate_ = state.cstate_;

                    state_type_ = state.state_type_;
                    next_state_type_ = state.next_state_type_;


                    if (state.next_support_set_)
                    {
                        switch (prev_state_type)
                        {
                            case humoto::wpg03::STATE_LSS:
                                if (next_state_type_ == humoto::wpg03::STATE_DS)
                                {
                                    right_foot_.position_.head(2) = get_ss_position_from_ds (
                                            STATE_RSS,
                                            state.next_support_rotation_,
                                            state.next_support_position_);
                                }
                                else
                                {
                                    right_foot_.position_.head(2) = state.next_support_position_;
                                }
                                break;
                            case humoto::wpg03::STATE_RSS:
                                if (next_state_type_ == humoto::wpg03::STATE_DS)
                                {
                                    left_foot_.position_.head(2) = get_ss_position_from_ds (
                                            STATE_LSS,
                                            state.next_support_rotation_,
                                            state.next_support_position_);
                                }
                                else
                                {
                                    left_foot_.position_.head(2) = state.next_support_position_;
                                }
                                break;

                            case humoto::wpg03::STATE_DS:
                                // no need to update positions of the feet (they must not change)
                                break;

                            default:
                                HUMOTO_THROW_MSG("Unknown state type.");
                                break;
                        }
                    }

                    determineSupportPosition();
                }



                /**
                 * @brief Determines position of support.
                 */
                void determineSupportPosition()
                {
                    switch(state_type_)
                    {
                        case STATE_LSS:
                            current_support_position_ = left_foot_.position_.head(2);
                            break;
                        case STATE_RSS:
                            current_support_position_ = right_foot_.position_.head(2);
                            break;
                        case STATE_DS:
                            /// Assume that in DS feet are aligned.
                            current_support_position_  = (right_foot_.position_.head(2) + left_foot_.position_.head(2)) / 2.;
                            break;
                        case STATE_TDS:
                            switch(next_state_type_)
                            {
                                case STATE_LSS:
                                    current_support_position_ = left_foot_.position_.head(2);
                                    break;
                                case STATE_RSS:
                                    current_support_position_ = right_foot_.position_.head(2);
                                    break;
                                case STATE_DS:
                                    /// Assume that in DS feet are aligned.
                                    current_support_position_  = (right_foot_.position_.head(2) + left_foot_.position_.head(2)) / 2.;
                                    break;
                                default:
                                    HUMOTO_THROW_MSG("Wrong state sequence.");
                                    break;
                            }
                            break;
                        default:
                            HUMOTO_THROW_MSG("Unknown support type.");
                            break;
                    }
                }


                /**
                 * @brief Log
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger &logger,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string &name = "model_state") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    logger.log(LogEntryName(subname).add("state_type")     , state_type_     );
                    logger.log(LogEntryName(subname).add("next_state_type"), next_state_type_);

                    logger.log(LogEntryName(subname).add("cstate"), cstate_);

                    logger.log(LogEntryName(subname).add("current_support_position"), current_support_position_);
                }
        };
    }
}
