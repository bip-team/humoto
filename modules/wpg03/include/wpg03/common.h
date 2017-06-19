/**
    @file
    @author  Alexander Sherikov
    @author  Don Joven Agravante
    @copyright 2014-2017 INRIA, 2014-2015 CNRS. Licensed under the Apache
    License, Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace wpg03
    {
        const char * COMJERK_VARIABLES_ID   = "comjerk";
        const char * FOOTPOS_VARIABLES_ID   = "footpos";
        const char * EXTWRENCH_VARIABLES_ID   = "extwrench";


        /**
         * @brief Possible states of the model
         */
        enum ModelStateType
        {
            /// Undefined
            STATE_UNDEFINED = 0,

            /// Double support
            STATE_DS = 1,

            /// Left single support
            STATE_LSS = 2,

            /// Right single support
            STATE_RSS = 3,

            /// Transitional double support
            STATE_TDS = 4
        };


        class ModelState : public humoto::ModelState
        {
            public:
                bool                            next_support_set_;
                etools::Vector2                 next_support_position_;
                etools::Matrix2                 next_support_rotation_;

                Eigen::Matrix<double, 6, 1>     cstate_;

                humoto::wpg03::ModelStateType   state_type_;
                humoto::wpg03::ModelStateType   next_state_type_;


                ModelState()
                {
                    next_support_set_ = false;
                    next_support_position_ << 0.0, 0.0;
                    next_support_rotation_ << 0.0, 0.0, 0.0, 0.0;

                    cstate_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

                    state_type_         = humoto::wpg03::STATE_UNDEFINED;
                    next_state_type_    = humoto::wpg03::STATE_UNDEFINED;
                }


                void log(   humoto::Logger &logger,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string &name = "model_state") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    logger.log(LogEntryName(subname).add("state_type")       , state_type_     );
                    logger.log(LogEntryName(subname).add("next_state_type")  , next_state_type_);

                    logger.log(LogEntryName(subname).add("next_support_set") , next_support_set_);

                    logger.log(LogEntryName(subname).add("cstate"), cstate_);

                    logger.log(LogEntryName(subname).add("next_support_position"), next_support_position_);
                    logger.log(LogEntryName(subname).add("next_support_rotation"), next_support_rotation_);
                }
        };



        /**
         * @brief Parameters of an MPC problem. [set_parameters_mpc.m]
         */
        class MPCParameters
        {
            private:
                /// Number of subsamples per sample (TN)
                unsigned int subsamples_num_;

                /// Number of subsamples per doubles support sample (TN_tds)
                unsigned int tds_subsamples_num_;

                /// Sampling time in seconds (T)
                double sampling_time_;

                /// Subsampling time in seconds (Ts)
                double subsampling_time_;

                /// Sampling time of a transitional double support in
                /// seconds (Ttds)
                double tds_sampling_time_;


            public:
                /// Length of the preview horizon (N)
                unsigned int preview_horizon_length_;

                /// Sampling time in milliseconds (T_ms)
                unsigned int sampling_time_ms_;

                /// Subsampling time in milliseconds (Ts_ms)
                unsigned int subsampling_time_ms_;

                /// Sampling time of a transitional double support in
                /// milliseconds (Ttds_ms)
                unsigned int tds_sampling_time_ms_;


                /**
                 * @brief Constructor.
                 *
                 * @param[in] preview_horizon_len   Length of the preview horizon
                 * @param[in] sampling_time_ms      Sampling time in milliseconds
                 * @param[in] subsampling_time_ms   Subsampling time in milliseconds
                 * @param[in] tds_sampling_time_ms  Sampling time of a transitional double support in milliseconds
                 */
                MPCParameters(  const unsigned int preview_horizon_len = 16,
                                const unsigned int sampling_time_ms = 100,
                                const unsigned int subsampling_time_ms = 100,
                                const unsigned int tds_sampling_time_ms = 100)
                {
                    preview_horizon_length_         = preview_horizon_len;
                    sampling_time_ms_               = sampling_time_ms;
                    subsampling_time_ms_            = subsampling_time_ms;
                    tds_sampling_time_ms_           = tds_sampling_time_ms;
                }


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
                        HUMOTO_THROW_MSG("Transitional double support samplig time should be a multiple of subsampling time.");
                    }

                    // convert milliseconds to seconds
                    sampling_time_      = convertMillisecondToSecond(sampling_time_ms_);
                    subsampling_time_   = convertMillisecondToSecond(subsampling_time_ms_);
                    tds_sampling_time_  = convertMillisecondToSecond(tds_sampling_time_ms_);
                }


                double getTDSSamplingTime () const
                {
                    return (tds_sampling_time_);
                }


                double getSamplingTime () const
                {
                    return (sampling_time_);
                }

                double getSubsamplingTime () const
                {
                    return (subsampling_time_);
                }
        };
    }
}
