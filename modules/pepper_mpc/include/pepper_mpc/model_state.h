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
    namespace pepper_mpc
    {
        /**
         * @brief
         */
        class HUMOTO_LOCAL ModelState : public humoto::ModelState, public humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "ModelState"
            #define HUMOTO_CONFIG_CONSTRUCTOR ModelState
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_MEMBER_CLASS(base_state_, "base_state") \
                HUMOTO_CONFIG_MEMBER_CLASS(body_state_, "body_state") \
                HUMOTO_CONFIG_SCALAR_(base_mass) \
                HUMOTO_CONFIG_SCALAR_(body_mass)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            protected:
                /**
                 * @brief Set default parameters
                 *
                 * @param[in] base_height
                 * @param[in] body_height
                 * @param[in] body_x_offset offset of the body with respect to the base
                 * @param[in] base_mass
                 * @param[in] body_mass
                 */
                void setParameters( const double base_height = 0.125564931735602,
                                    const double body_height = 7.50001340031501e-01,
                                    const double body_x_offset = -0.00656061094036387,
                                    //const double body_height = 0.763104597149514,
                                    //const double body_x_offset = -0.00671291567566885,
                                    const double base_mass = 16.34234,
                                    const double body_mass = 12.3389)
                {
                    // initialize com states
                    base_state_.setDefaults();
                    body_state_.setDefaults();

                    base_state_.position_.z() = base_height; // Height of the CoM
                    base_state_.rpy_.z() = 0.0;       // Orientation of the base

                    body_state_.position_.z() = body_height; // Height of the CoM
                    body_state_.position_.x() = body_x_offset; // Height of the CoM

                    base_mass_ = base_mass;
                    body_mass_ = body_mass;
                }


                void setDefaults()
                {
                    setParameters();
                }


            public:
                /// State of the base
                humoto::rigidbody::RigidBodyState   base_state_;

                /// State of the body
                humoto::rigidbody::PointMassState   body_state_;

                double  base_mass_;
                double  body_mass_;


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
                 * @param[in] base_height
                 * @param[in] body_height
                 * @param[in] body_x_offset offset of the body with respect to the base
                 */
                ModelState( const double base_height,
                            const double body_height,
                            const double body_x_offset)
                {
                    setParameters(body_height, base_height, body_x_offset);
                }


                /**
                 * @brief Construct class using given parameters.
                 *
                 * @param[in] base_mass
                 * @param[in] body_mass
                 * @param[in] base_position
                 * @param[in] body_position
                 */
                ModelState( const double & base_mass,
                            const double & body_mass,
                            const etools::Vector3 & base_position,
                            const etools::Vector3 & body_position)
                {
                    set(base_mass, body_mass, base_position, body_position);
                }



                /**
                 * @brief Initialize state directly with the states of the CoM and feet.
                 *
                 * @param[in] base_state        state of the base
                 * @param[in] body_state        state of the body
                 */
                void set(   const humoto::rigidbody::RigidBodyState & base_state,
                            const humoto::rigidbody::PointMassState & body_state)
                {
                    base_state_ = base_state;
                    body_state_ = body_state;
                }


                /**
                 * @brief Initialize state.
                 *
                 * @param[in] base_mass
                 * @param[in] body_mass
                 * @param[in] base_position
                 * @param[in] body_position
                 */
                void set(   const double & base_mass,
                            const double & body_mass,
                            const etools::Vector3 & base_position,
                            const etools::Vector3 & body_position)
                {
                    base_mass_ = base_mass;
                    body_mass_ = body_mass;


                    base_state_.setDefaults();
                    body_state_.setDefaults();

                    base_state_.position_ = base_position;
                    body_state_.position_ = body_position;
                }


                /**
                 * @brief Partial update of the state.
                 *
                 * @param[in] base_mass
                 * @param[in] body_mass
                 * @param[in] base_position
                 * @param[in] body_position
                 * @param[in] base_orientation
                 */
                void update(const double & base_mass,
                            const double & body_mass,
                            const etools::Vector3 & base_position,
                            const etools::Vector3 & body_position,
                            const double & base_orientation)
                {
                    base_mass_ = base_mass;
                    body_mass_ = body_mass;

                    base_state_.position_ = base_position;
                    base_state_.rpy_.z() = base_orientation;

                    body_state_.position_ = body_position;
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

                    logger.log(LogEntryName(subname).add("base_mass"), base_mass_);
                    logger.log(LogEntryName(subname).add("body_mass"), body_mass_);

                    base_state_.log(logger, subname, "base_state");
                    body_state_.log(logger, subname, "body_state");
                }
        };
    }
}

