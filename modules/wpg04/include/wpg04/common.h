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
    namespace wpg04
    {
        static const char * COP_VARIABLES_ID       = "CoP";
        static const char * FOOTPOS_VARIABLES_ID   = "footpos";


        /**
         * @brief Class containing options of the walking pattern generator
         */
        class HUMOTO_LOCAL WalkParameters : public humoto::walking::StanceFSMParameters
        {
            #define HUMOTO_CONFIG_SECTION_ID "WalkParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR WalkParameters
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_COMPOUND_(com_velocity)\
                HUMOTO_CONFIG_COMPOUND_(first_stance_com_velocity)\
                HUMOTO_CONFIG_COMPOUND_(last_stance_com_velocity)\
                \
                HUMOTO_CONFIG_SCALAR_(step_height)\
                HUMOTO_CONFIG_SCALAR_(theta_increment)\
                \
                HUMOTO_CONFIG_PARENT_CLASS(humoto::walking::StanceFSMParameters)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            public:
                etools::Vector2    com_velocity_;
                etools::Vector2    first_stance_com_velocity_;
                etools::Vector2    last_stance_com_velocity_;

                double              step_height_;
                double              theta_increment_;


            public:
                /**
                 * @brief Default constructor
                 */
                WalkParameters()
                {
                    setDefaults();
                }



                /**
                 * @brief Default parameters of the walk
                 */
                void setDefaults()
                {
                    com_velocity_               << 0.2, 0.;
                    first_stance_com_velocity_  << 0.2, 0.;
                    last_stance_com_velocity_   << 0.,  0.;
                    step_height_                = 0.07;
                    theta_increment_            = 0.0;

                    humoto::walking::StanceFSMParameters::setDefaults();
                }
        };



        /**
         * @brief Parameters of an MPC problem. [set_parameters_mpc.m]
         */
        class HUMOTO_LOCAL MPCParameters : public humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "MPCParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR MPCParameters
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_SCALAR_(preview_horizon_length)\
                HUMOTO_CONFIG_SCALAR_(sampling_time_ms)\
                HUMOTO_CONFIG_SCALAR_(subsampling_time_ms)\
                HUMOTO_CONFIG_SCALAR_(tds_sampling_time_ms)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            private:
                /// Number of subsamples per sample (TN)
                std::size_t subsamples_num_;

                /// Number of subsamples per doubles support sample (TN_tds)
                std::size_t tds_subsamples_num_;

                /// Sampling time in seconds (T)
                double sampling_time_;

                /// Subsampling time in seconds (Ts)
                double subsampling_time_;

                /// Sampling time of a transitional double support in
                /// seconds (Ttds)
                double tds_sampling_time_;


            protected:
                /**
                 * @brief Compute some derived variables.
                 */
                void finalize()
                {
                    if (sampling_time_ms_ % subsampling_time_ms_ == 0)
                    {
                        subsamples_num_ = sampling_time_ms_ / subsampling_time_ms_;
                    }
                    else
                    {
                        HUMOTO_THROW_MSG("Sampling time should be a multiple of subsampling time.");
                    }


                    if (tds_sampling_time_ms_ % subsampling_time_ms_ == 0)
                    {
                        tds_subsamples_num_ = tds_sampling_time_ms_ / subsampling_time_ms_;
                    }
                    else
                    {
                        HUMOTO_THROW_MSG("Transitional double support sampling time should be a multiple of subsampling time.");
                    }

                    // convert milliseconds to seconds
                    sampling_time_      = convertMillisecondToSecond(sampling_time_ms_);
                    subsampling_time_   = convertMillisecondToSecond(subsampling_time_ms_);
                    tds_sampling_time_  = convertMillisecondToSecond(tds_sampling_time_ms_);
                }


            public:
                /// Length of the preview horizon (N)
                std::size_t preview_horizon_length_;

                /// Sampling time in milliseconds (T_ms)
                std::size_t sampling_time_ms_;

                /// Subsampling time in milliseconds (Ts_ms)
                std::size_t subsampling_time_ms_;

                /// Sampling time of a transitional double support in
                /// milliseconds (Ttds_ms)
                std::size_t tds_sampling_time_ms_;


            public:
                /**
                 * @brief Initialize to default values
                 */
                void setDefaults()
                {
                    preview_horizon_length_         = 16;
                    sampling_time_ms_               = 100;
                    subsampling_time_ms_            = 100;
                    tds_sampling_time_ms_           = 100;

                    finalize();
                }


                /**
                 * @brief Constructor.
                 *
                 * @param[in] preview_horizon_len   Length of the preview horizon
                 * @param[in] sampling_time_ms      Sampling time in milliseconds
                 * @param[in] subsampling_time_ms   Subsampling time in milliseconds
                 * @param[in] tds_sampling_time_ms  Sampling time of a transitional double support in milliseconds
                 */
                MPCParameters(  const std::size_t preview_horizon_len = 16,
                                const std::size_t sampling_time_ms = 100,
                                const std::size_t subsampling_time_ms = 100,
                                const std::size_t tds_sampling_time_ms = 100)
                {
                    preview_horizon_length_         = preview_horizon_len;
                    sampling_time_ms_               = sampling_time_ms;
                    subsampling_time_ms_            = subsampling_time_ms;
                    tds_sampling_time_ms_           = tds_sampling_time_ms;

                    finalize();
                }


                /**
                 * @brief getSubsamplingTime
                 *
                 * @return subsampling time
                 */
                double getSubsamplingTime () const
                {
                    return (subsampling_time_);
                }
        };
    }
}

