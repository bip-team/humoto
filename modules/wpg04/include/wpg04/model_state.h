/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace wpg04
    {
        class HUMOTO_LOCAL ModelState : public humoto::ModelState, public humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "ModelState"
            #define HUMOTO_CONFIG_CONSTRUCTOR ModelState
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_ENUM_(stance_type) \
                HUMOTO_CONFIG_ENUM_(next_stance_type) \
                HUMOTO_CONFIG_MEMBER_CLASS(com_state_, "com_state") \
                HUMOTO_CONFIG_MEMBER_CLASS(feet_.getLeft(), "left_foot_state") \
                HUMOTO_CONFIG_MEMBER_CLASS(feet_.getRight(), "right_foot_state")
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            private:
                /**
                 * @brief Initialize foot state in an ADS
                 *
                 * @param[in] foot_param parameters of the feet
                 * @param[in] left_or_right left or right foot
                 * @param[in] theta orientation of the ADS
                 * @param[in] initial_ds_position position of the ADS.
                 */
                void initializeFoot(const humoto::walking::RobotFootParameters  &foot_param,
                                    const humoto::LeftOrRight::Type               left_or_right,
                                    const double                                theta,
                                    const etools::Vector2               &initial_ds_position)
                {
                    etools::Matrix2 ds_R;
                    ds_R = Eigen::Rotation2Dd(theta);

                    feet_[left_or_right].position_.head(2) = foot_param.getFootPositionFromADS(
                                                                left_or_right,
                                                                ds_R,
                                                                initial_ds_position);
                    feet_[left_or_right].position_.z() = 0.0;
                    feet_[left_or_right].velocity_.setZero();
                    feet_[left_or_right].acceleration_.setZero();
                    feet_[left_or_right].rpy_ << 0., 0., theta;
                    etools::unsetMatrix(feet_[left_or_right].angular_velocity_);
                    etools::unsetMatrix(feet_[left_or_right].angular_acceleration_);
                }


            protected:
                void setDefaults(const humoto::walking::RobotFootParameters  &foot_param,
                                 const double com_height = 0.814)
                {
                    // initialize foot states
                    double theta = 0.;
                    etools::Vector2 initial_ds_position;
                    initial_ds_position.setZero();

                    initializeFoot(foot_param, humoto::LeftOrRight::LEFT,  theta, initial_ds_position);
                    initializeFoot(foot_param, humoto::LeftOrRight::RIGHT, theta, initial_ds_position);


                    // initialize com states
                    com_state_.setDefaults();
                    com_state_.position_.z() = com_height; // Height of the CoM


                    stance_type_         = humoto::walking::StanceType::DS;
                    next_stance_type_    = humoto::walking::StanceType::UNDEFINED;
                }


            public:
                /// State of the CoM
                humoto::rigidbody::PointMassState   com_state_;

                /// States of the feet
                humoto::LeftRightContainer<humoto::rigidbody::RigidBodyState> feet_;

                /// current stance type
                walking::StanceType::Type  stance_type_;

                /// next stance type
                walking::StanceType::Type  next_stance_type_;



            public:
                /**
                 * @brief Default constructor
                 */
                ModelState()
                {
                    setDefaults();
                }


                /**
                 * @brief Construct class using given parameters.
                 *
                 * @param[in] robot_parameters  parameters of the feet
                 * @param[in] com_height        height of the CoM (with respect to the feet)
                 */
                ModelState( const humoto::walking::RobotFootParameters & robot_parameters,
                            const double com_height)
                {
                    setDefaults(robot_parameters, com_height);
                }


                /**
                 * @brief Initialize to default values (HRP2)
                 */
                void setDefaults()
                {
                    humoto::walking::RobotFootParameters robot_parameters;
                    setDefaults(robot_parameters);
                }



                /**
                 * @brief Initialize state using parameters
                 *
                 * @param[in] robot_parameters parameters of the feet
                 * @param[in] com_height        height of the CoM (with respect to the feet)
                 */
                void set (  const humoto::walking::RobotFootParameters & robot_parameters,
                            const double com_height)
                {
                    setDefaults(robot_parameters, com_height);
                }



                /**
                 * @brief Initialize state directly with the states of the CoM and feet.
                 *
                 * @param[in] current_stance    current stance
                 * @param[in] com_state         state of the CoM
                 * @param[in] left_foot_state   state of the left foot
                 * @param[in] right_foot_state  state of the right foot
                 */
                void set(   const walking::StanceType::Type   current_stance,
                            const rigidbody::PointMassState & com_state,
                            const rigidbody::RigidBodyState & left_foot_state,
                            const rigidbody::RigidBodyState & right_foot_state)
                {
                    HUMOTO_ASSERT(  (current_stance == humoto::walking::StanceType::LSS)
                                    || (current_stance == humoto::walking::StanceType::RSS)
                                    || (current_stance == humoto::walking::StanceType::DS),
                                    "The stance type must be LSS, RSS, or DS.");
                    com_state_ = com_state;
                    feet_.setLeft(left_foot_state);
                    feet_.setRight(right_foot_state);

                    stance_type_         = current_stance;
                    next_stance_type_    = humoto::walking::StanceType::UNDEFINED;
                }



                /**
                 * @brief Log
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string &name = "model_state") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    logger.log(LogEntryName(subname).add("stance_type")     , stance_type_     );
                    logger.log(LogEntryName(subname).add("next_stance_type"), next_stance_type_);

                    com_state_.log(logger, subname, "com_state");

                    feet_.getLeft().log(logger, subname, "left_foot_state");
                    feet_.getRight().log(logger, subname, "right_foot_state");
                }
        };
    }
}

