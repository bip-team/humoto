/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace pepper_mpc
    {
        static const char * BASE_VEL_VARIABLES_ID   = "base_vel";
        static const char * BODY_JERK_VARIABLES_ID   = "body_pos";


        /**
         * @brief Parameters of the motion
         */
        class HUMOTO_LOCAL RobotParameters : public humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "RobotParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR RobotParameters
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_SCALAR_(base_radius); \
                HUMOTO_CONFIG_SCALAR_(max_nominal_base_velocity); \
                HUMOTO_CONFIG_SCALAR_(max_nominal_base_acceleration); \
                HUMOTO_CONFIG_SCALAR_(max_base_velocity); \
                HUMOTO_CONFIG_SCALAR_(max_base_acceleration); \
                HUMOTO_CONFIG_COMPOUND_(body_bounds);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            private:
                // meter
                etools::Matrix2    cop_bounds_;
                // meter / second
                etools::Matrix2    nominal_base_velocity_bounds_;
                // meter / second^2
                etools::Matrix2    nominal_base_acceleration_bounds_;
                // meter / second
                etools::Matrix2    base_velocity_bounds_;
                // meter / second^2
                etools::Matrix2    base_acceleration_bounds_;


            protected:
                /**
                 * @brief Finalize initialization
                 */
                void finalize()
                {
                    double cop_bound = getEncircledSquareSide(base_radius_)/2;

                    cop_bounds_ << -cop_bound, cop_bound,
                                   -cop_bound, cop_bound;

                    // ---

                    double velocity_bound = getEncircledSquareSide(max_nominal_base_velocity_)/2;

                    nominal_base_velocity_bounds_ <<   -velocity_bound, velocity_bound,
                                                       -velocity_bound, velocity_bound;


                    velocity_bound = getEncircledSquareSide(max_base_velocity_)/2;

                    base_velocity_bounds_ <<   -velocity_bound, velocity_bound,
                                               -velocity_bound, velocity_bound;

                    // ---

                    double acceleration_bound = getEncircledSquareSide(max_nominal_base_acceleration_)/2;

                    nominal_base_acceleration_bounds_ <<   -acceleration_bound, acceleration_bound,
                                                           -acceleration_bound, acceleration_bound;


                    acceleration_bound = getEncircledSquareSide(max_base_acceleration_)/2;

                    base_acceleration_bounds_ <<   -acceleration_bound, acceleration_bound,
                                                   -acceleration_bound, acceleration_bound;
                }


            public:
                // meter
                double base_radius_;
                // meter / second
                double max_nominal_base_velocity_;
                // meter / second^2
                double max_nominal_base_acceleration_;
                // meter / second
                double max_base_velocity_;
                // meter / second^2
                double max_base_acceleration_;

                // [lb, ub] meter
                etools::Matrix2 body_bounds_;


            public:
                void setDefaults()
                {
                    base_radius_    = 0.7;
                    max_nominal_base_velocity_      = 0.5;
                    max_nominal_base_acceleration_  = 1.0;
                    max_base_velocity_      = 1.4;
                    max_base_acceleration_  = 1.7;

                    body_bounds_    <<  -0.06, 0.06,
                                        -0.03, 0.03;
                    finalize();
                }


                /**
                 * @brief Default constructor
                 */
                RobotParameters()
                {
                    setDefaults();
                }


                /**
                 * @brief Get respective bounds
                 *
                 * @return [lb, ub] 2x2 matrix
                 */
                etools::Matrix2 getCoPBounds() const
                {
                    return(cop_bounds_);
                }

                /// @copydoc getCoPBounds
                etools::Matrix2 getNominalBaseVelocityBounds() const
                {
                    return(nominal_base_velocity_bounds_);
                }

                /// @copydoc getCoPBounds
                etools::Matrix2 getNominalBaseAccelerationBounds() const
                {
                    return(nominal_base_acceleration_bounds_);
                }

                /// @copydoc getCoPBounds
                etools::Matrix2 getBodyBounds() const
                {
                    return(body_bounds_);
                }
        };



        class MotionMode
        {
            public:
                enum Mode
                {
                    UNDEFINED = 0,
                    MAINTAIN_POSITION  = 1,
                    MAINTAIN_VELOCITY  = 2
                };
        };


        /**
         * @brief Parameters of the motion
         */
        class HUMOTO_LOCAL MotionParameters : public humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "MotionParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR MotionParameters
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_COMPOUND_(base_velocity);\
                HUMOTO_CONFIG_COMPOUND_(base_position);\
                HUMOTO_CONFIG_SCALAR_(base_angular_velocity);\
                HUMOTO_CONFIG_ENUM_(motion_mode); \
                HUMOTO_CONFIG_SCALAR_(duration_ms);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            public:
                const static  std::ptrdiff_t  UNLIMITED_DURATION = -1;

                etools::Vector2     base_velocity_;
                etools::Vector2     base_position_;

                double              base_angular_velocity_;
                std::ptrdiff_t      duration_ms_;

                MotionMode::Mode    motion_mode_;


            public:
                /**
                 * @brief Default constructor
                 */
                MotionParameters()
                {
                    setDefaults();
                }


                /**
                 * @brief Default parameters of the walk
                 */
                void setDefaults()
                {
                    setIdle();
                }


                /**
                 * @brief Idle parameters of the motion
                 */
                void setIdle()
                {
                    base_velocity_.setZero();
                    base_position_.setZero();
                    base_angular_velocity_ =  0.0;
                    duration_ms_ = UNLIMITED_DURATION;
                    motion_mode_ = MotionMode::MAINTAIN_VELOCITY;
                }


                /**
                 * @brief Finalize & check
                 */
                void finalize()
                {
                }


                /**
                 * @brief Get base reference velocity
                 *
                 * @return base velocity
                 */
                etools::Vector2 getBaseVelocity() const
                {
                    return(base_velocity_);
                }


                /**
                 * @brief Get base reference position
                 *
                 * @return base position
                 */
                etools::Vector2 getBasePosition() const
                {
                    return(base_position_);
                }


                /**
                 * @brief Get theta increment
                 *
                 * @return theta increment
                 */
                double getBaseAngularVelocity() const
                {
                    return(base_angular_velocity_);
                }
        };



        /**
         * @brief Parameters of the MPC problem.
         */
        class HUMOTO_LOCAL MPCParameters : public humoto::config::StrictConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "MPCParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR MPCParameters
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_SCALAR_(preview_horizon_length); \
                HUMOTO_CONFIG_SCALAR_(sampling_time_ms); \
                HUMOTO_CONFIG_SCALAR_(subsampling_time_ms);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            private:
                /// Sampling time in seconds (T)
                double sampling_time_;
                
                /// Subsampling time in seconds (Ts)
                double subsampling_time_;


            protected:
                /**
                 * @brief Compute some derived variables.
                 */
                void finalize()
                {
                    // convert milliseconds to seconds
                    sampling_time_    = convertMillisecondToSecond(sampling_time_ms_);
                    // convert milliseconds to seconds
                    subsampling_time_ = convertMillisecondToSecond(subsampling_time_ms_);
                }


            public:
                /// Length of the preview horizon (N)
                std::size_t preview_horizon_length_;

                /// Sampling time in milliseconds (T_ms)
                std::size_t sampling_time_ms_;

                /// Subsampling time in milliseconds (Ts_ms)
                std::size_t subsampling_time_ms_;


            public:
                /**
                 * @brief Initialize to default values
                 */
                void setDefaults()
                {
                    preview_horizon_length_         = 15;
                    sampling_time_ms_               = 100;
                    subsampling_time_ms_            = sampling_time_ms_;

                    finalize();
                }


                /**
                 * @brief Constructor.
                 *
                 * @param[in] preview_horizon_len   Length of the preview horizon
                 * @param[in] sampling_time_ms      Sampling time in milliseconds
                 * @param[in] subsampling_time_ms   Subsampling time in milliseconds
                 */
                MPCParameters(  const std::size_t preview_horizon_len = 15,
                                const std::size_t sampling_time_ms = 100,
                                const std::size_t subsampling_time_ms = 100)
                {
                    preview_horizon_length_         = preview_horizon_len;
                    sampling_time_ms_               = sampling_time_ms;
                    subsampling_time_ms_            = subsampling_time_ms;

                    finalize();
                }


                /**
                 * @brief Get sampling time in seconds
                 *
                 * @return sampling time in seconds
                 */
                double getSamplingTime() const
                {
                    return (sampling_time_);
                }
                
                
                /**
                 * @brief Get subsampling time in seconds
                 *
                 * @return subsampling time in seconds
                 */
                double getSubsamplingTime() const
                {
                    return (subsampling_time_);
                }
        };
    }
}

