/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA, 2014-2015 CNRS. Licensed under the Apache
    License, Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace rigidbody
    {
        class HUMOTO_LOCAL RigidBodyPose : public humoto::config::ConfigurableBase
        {
            protected:
                #define HUMOTO_CONFIG_SECTION_ID "RigidBodyPose"
                #define HUMOTO_CONFIG_ENTRIES \
                        HUMOTO_CONFIG_COMPOUND_(position); \
                        HUMOTO_CONFIG_COMPOUND_(rpy);
                #include HUMOTO_CONFIG_DEFINE_ACCESSORS


                /**
                 * @brief Set default values
                 */
                void setDefaults()
                {
                    etools::unsetMatrix(position_);
                    etools::unsetMatrix(rpy_);
                }


            public:
                etools::Vector3 position_;
                etools::Vector3 rpy_;


            public:
                HUMOTO_DEFINE_CONFIG_CONSTRUCTORS(RigidBodyPose)


                /**
                 * @brief  Default constructor
                 */
                RigidBodyPose()
                {
                    setDefaults();
                }


                /**
                 * @brief Writes a pose to a stream
                 *
                 * @param[in,out] out output stream
                 * @param[in] pose
                 *
                 * @return output stream
                 *
                 * @attention No leading or trailing spaces.
                 */
                friend std::ostream& operator<< (   std::ostream& out,
                                                    const RigidBodyPose & pose)
                {
                    out << pose.position_.x() << " "
                        << pose.position_.y() << " "
                        << pose.position_.z() << " "
                        << pose.rpy_.x() << " "
                        << pose.rpy_.y() << " "
                        << pose.rpy_.z();

                    return(out);
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
                            const std::string &name = "rigid_body_pose") const
                {
                    LogEntryName subname = parent;
                    subname.add(name);
                    logger.log(LogEntryName(subname).add("position"), position_);
                    logger.log(LogEntryName(subname).add("rpy")     , rpy_);
                }
        };



        /**
         * @brief Class that groups together parameters related to a robot foot
         */
        class HUMOTO_LOCAL RigidBodyState : public PointMassState, public RotaryState
        {
            protected:
                #define HUMOTO_CONFIG_SECTION_ID "RigidBodyState"
                #define HUMOTO_CONFIG_ENTRIES \
                        HUMOTO_CONFIG_PARENT_CLASS(PointMassState); \
                        HUMOTO_CONFIG_PARENT_CLASS(RotaryState);
                #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            public:
                HUMOTO_DEFINE_CONFIG_CONSTRUCTORS(RigidBodyState)


                /**
                 * @brief Default constructor.
                 */
                RigidBodyState()
                {
                    setDefaults();
                }


                /**
                 * @brief Initialize state (everything is set to zeros).
                 */
                void setDefaults()
                {
                    PointMassState::setDefaults();
                    RotaryState::setDefaults();
                }


                /**
                 * @brief Initialize state (everything is set to NaN).
                 */
                void unset()
                {
                    PointMassState::unset();
                    RotaryState::unset();
                }


                /**
                 * @brief Log
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                            const LogEntryName &parent= LogEntryName(),
                            const std::string &name = "rigid_body_state") const
                {
                    PointMassState::log(logger, parent, name);
                    RotaryState::log(logger, parent, name);
                }
        };
    }
}
