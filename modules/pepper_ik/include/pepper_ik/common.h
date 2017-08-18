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
    namespace pepper_ik
    {
        /// Root position and orientation
        static const char * ROOT_VARIABLES_ID   = "root_vel";
        /// Joint angles
        static const char * JOINTS_VARIABLES_ID   = "joints_vel";


        /**
         * @brief Parameters of the WBC problem.
         */
        class HUMOTO_LOCAL WBCParameters : public humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "WBCParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR WBCParameters
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_SCALAR_(control_interval_ms); \
                HUMOTO_CONFIG_SCALAR_(joint_angle_error_tolerance); \
                HUMOTO_CONFIG_SCALAR_(motion_parameters_tolerance); \
                HUMOTO_CONFIG_SCALAR_(maximal_number_of_iterations);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            protected:
                /**
                 * @brief Initialize to default values
                 */
                void setDefaults()
                {
                    control_interval_ms_            = 10;
                    joint_angle_error_tolerance_    = 1e-2;
                    maximal_number_of_iterations_   = 3;
                    motion_parameters_tolerance_    = 1e-3;
                }

                void finalize()
                {
                    control_interval_ = convertMillisecondToSecond(control_interval_ms_);
                }


            public:
                double          control_interval_;
                std::size_t     control_interval_ms_;

                double          joint_angle_error_tolerance_;
                double          motion_parameters_tolerance_;

                std::size_t     maximal_number_of_iterations_;


            public:
                WBCParameters()
                {
                    setDefaults();
                    finalize();
                }
        };



        /**
         * @brief Desired motion parameters
         */
        class HUMOTO_LOCAL MotionParameters : public humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "MotionParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR MotionParameters
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_COMPOUND_(base_orientation_rpy); \
                HUMOTO_CONFIG_COMPOUND_(base_com_position); \
                HUMOTO_CONFIG_COMPOUND_(body_com_position);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            public:
                etools::Vector3     base_com_position_;
                etools::Vector3     body_com_position_;

                etools::Vector3     base_orientation_rpy_;


            public:
                MotionParameters()
                {
                    setDefaults();
                }


                /**
                 * @brief Initialize to default values
                 */
                void setDefaults()
                {
                    base_com_position_.x() = 0.002531976618098;
                    base_com_position_.y() = 0.0;
                    base_com_position_.z() = 0.125564931735602;

                    body_com_position_.x() = -0.00402863432226587;
                    body_com_position_.y() = 0.0;
                    body_com_position_.z() = 0.75;

                    base_orientation_rpy_.x() = 0;
                    base_orientation_rpy_.y() = 0;
                    base_orientation_rpy_.z() = 0;
                }
        };
    }
}
