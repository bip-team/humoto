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
        /**
         * @brief Class that groups together parmeters related to a robot foot
         */
        class HUMOTO_LOCAL PointMassState : public virtual humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "PointMassState"
            #define HUMOTO_CONFIG_CONSTRUCTOR PointMassState 
            #define HUMOTO_CONFIG_ENTRIES \
                    HUMOTO_CONFIG_COMPOUND_(position) \
                    HUMOTO_CONFIG_COMPOUND_(velocity) \
                    HUMOTO_CONFIG_COMPOUND_(acceleration)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            public:
                etools::Vector3 position_;
                etools::Vector3 velocity_;
                etools::Vector3 acceleration_;


            public:
                /**
                 * @brief Default constructor.
                 */
                PointMassState()
                {
                    setDefaults();
                }


                /**
                 * @brief Initialize state (everything is set to zeros).
                 */
                void setDefaults()
                {
                    position_.setZero();
                    velocity_.setZero();
                    acceleration_.setZero();
                }


                /**
                 * @brief Initialize state (everything is set to NaN).
                 */
                void unset()
                {
                    etools::unsetMatrix(position_);
                    etools::unsetMatrix(velocity_);
                    etools::unsetMatrix(acceleration_);
                }


                /**
                 * @brief Set state
                 *
                 * @param[in] position
                 * @param[in] velocity
                 * @param[in] acceleration
                 */
                void set(   const etools::Vector3 position,
                            const etools::Vector3 velocity = etools::Vector3::Zero(),
                            const etools::Vector3 acceleration = etools::Vector3::Zero())
                {
                    position_     = position;
                    velocity_     = velocity;
                    acceleration_ = acceleration;
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
                            const std::string &name = "point_mass_state") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    logger.log(LogEntryName(subname).add("position")    , position_);
                    logger.log(LogEntryName(subname).add("velocity")    , velocity_);
                    logger.log(LogEntryName(subname).add("acceleration"), acceleration_);
                }
        };
    }
}
