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
         * @brief Parameters
         */
        class HUMOTO_LOCAL RobotParameters : public humoto::walking::RobotFootParameters
        {
            public:
                double com_height_;
                double mass_;


                RobotParameters()
                {
                    setDefaults();
                }


                void setDefaults()
                {
                    com_height_ = 0.814; // Height of the CoM
                    mass_ = 38.05487;

                    humoto::walking::RobotFootParameters::setDefaults();
                }
        };



        /**
         * @brief Class that defines the robot used in walking pattern generation
         *
         *
         *  '"""""""""""""'                 \
         *  |             |                  | max_feet_dist
         *  ._____________.                  |
         *                  } min_feet_dist /
         *         * <- foot position
         *  `^^^^^^' max_step_len
         *
         */
        class Robot : public RobotParameters
        {
            // commonly used abbreviations:
            // ss - single support (1 foot contacting the ground)
            // ds - double support (both feet contacting the ground)

            public:
                Robot();
                Robot(const RobotParameters &);

                double initial_theta_;
                etools::Vector2 initial_ds_position_;
                humoto::rigidbody::RigidBodyState left_foot_;
                humoto::rigidbody::RigidBodyState right_foot_;
                etools::Matrix2 ds_R_;

                void setParameters(const RobotParameters &);
                void processParameters();
                void setDefaultStates();
                etools::Vector2 get_ss_position_from_ds(const ModelStateType foot_type,
                                                        const etools::Matrix2 &ds_R,
                                                        const etools::Vector2 &ds_position);
                etools::Vector2 get_ds_position_from_ss(const ModelStateType foot_type,
                                                        const etools::Matrix2 &ss_R,
                                                        const etools::Vector2 &ss_position);

                double getComHeight() const;
                double getMass() const;
        };


        /**
         * @brief Constructor
         */
        Robot::Robot()
        {
            RobotParameters robot_parameters;

            setParameters(robot_parameters);
        }


        /**
         * @brief Constructor
         */
        Robot::Robot(const RobotParameters &robot_parameters)
        {
            setParameters(robot_parameters);
        }


        /**
         * @brief Get the single support position from double support
         *
         * @param[in] foot_type
         * @param[in] ds_R
         * @param[in] ds_position
         */
        etools::Vector2 Robot::get_ss_position_from_ds(const ModelStateType foot_type,
                                                       const etools::Matrix2 &ds_R,
                                                       const etools::Vector2 &ds_position)
        {
            switch(foot_type)
            {
                case humoto::walking::StanceType::RSS:
                    return (getFootPositionFromADS(humoto::LeftOrRight::RIGHT, ds_R, ds_position));
                case humoto::walking::StanceType::LSS:
                    return (getFootPositionFromADS(humoto::LeftOrRight::LEFT, ds_R, ds_position));
                default:
                    HUMOTO_THROW_MSG("Wrong foot type.");
            }
        }

        /**
         * @brief Get the double support position from single support
         *
         * @param[in] foot_type
         * @param[in] ss_R
         * @param[in] ss_position
         */
        etools::Vector2 Robot::get_ds_position_from_ss(const ModelStateType foot_type,
                                                        const etools::Matrix2 &ss_R,
                                                        const etools::Vector2 &ss_position)
        {
            switch(foot_type)
            {
                case humoto::walking::StanceType::RSS:
                    return (getADSPositionFromFoot(humoto::LeftOrRight::RIGHT, ss_R, ss_position));
                case humoto::walking::StanceType::LSS:
                    return (getADSPositionFromFoot(humoto::LeftOrRight::LEFT, ss_R, ss_position));
                default:
                    HUMOTO_THROW_MSG("Wrong foot type.");
            }
        }



        /**
         * @brief Set the robot's states to default
         */
        void Robot::setDefaultStates()
        {
            initial_theta_ = 0.;
            initial_ds_position_ << 0., 0.;

            // initialize feet states
            ds_R_ = Eigen::Rotation2Dd(initial_theta_);

            left_foot_.position_.head(2) = get_ss_position_from_ds(STATE_LSS, ds_R_, initial_ds_position_);
            left_foot_.position_[2] = 0.0;
            left_foot_.velocity_ << 0., 0., 0.;
            left_foot_.acceleration_ << 0., 0., 0.;
            left_foot_.rpy_ << 0., 0., initial_theta_;

            right_foot_.position_.head(2) = get_ss_position_from_ds(STATE_RSS, ds_R_, initial_ds_position_);
            right_foot_.position_[2] = 0.0;
            right_foot_.velocity_ << 0., 0., 0.;
            right_foot_.acceleration_ << 0., 0., 0.;
            right_foot_.rpy_ << 0., 0., initial_theta_;
        }


        /**
         * @brief Set the robot's walking parameters
         *
         * @param[in] robot_parameters
         */
        void Robot::setParameters(const RobotParameters &robot_parameters)
        {
            RobotParameters::operator= (robot_parameters);

            setDefaultStates();
        }

        double Robot::getMass() const
        {
            return mass_;
        }

        double Robot::getComHeight() const
        {
            return com_height_;
        }
    }
}
