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
        /**
         * @brief A helper class defining one interval of a preview horizon
         */
        class HUMOTO_LOCAL PreviewHorizonInterval
        {
            public:
                // meter
                etools::Matrix2    cop_bounds_;
                // meter / second
                etools::Matrix2    nominal_base_velocity_bounds_;
                // meter / second^2
                etools::Matrix2    nominal_base_acceleration_bounds_;
                // [lb, ub] meter
                etools::Matrix2    body_bounds_;
                // meter / second
                etools::Vector2    base_vel_ref_;
                // meter
                etools::Vector2    base_pos_ref_;

                double base_height_;
                double body_height_;

                double base_mass_;
                double body_mass_;

                std::size_t T_ms_;
                double      T_;

                double              theta_;
                etools::Matrix2    rotation_;


            public:
                /**
                 * @brief Default constructor
                 */
                PreviewHorizonInterval()
                {
                    base_height_ = 0.0;
                    body_height_ = 0.0;

                    base_mass_ = 0.0;
                    body_mass_ = 0.0;

                    T_ms_ = 0;
                    T_ = 0.0;
                    theta_ = 0.0;

                    etools::unsetMatrix(cop_bounds_);
                    etools::unsetMatrix(nominal_base_velocity_bounds_);
                    etools::unsetMatrix(nominal_base_acceleration_bounds_);
                    etools::unsetMatrix(body_bounds_);

                    etools::unsetMatrix(base_vel_ref_);
                    etools::unsetMatrix(base_pos_ref_);
                    etools::unsetMatrix(rotation_);
                }


                /**
                 * @brief Form the preview horizon object
                 *
                 * @param[in] mpc_params
                 * @param[in] model
                 * @param[in] motion_parameters
                 * @param[in] theta
                 *
                 * @return
                 */
                void initialize(double                      & theta,
                                const MPCParameters         &mpc_params,
                                const MotionParameters      &motion_parameters,
                                const Model                 &model)
                {
                    T_ms_ = mpc_params.sampling_time_ms_;
                    T_    = mpc_params.getSamplingTime();

                    cop_bounds_ =                         model.robot_parameters_.getCoPBounds();
                    nominal_base_velocity_bounds_ =       model.robot_parameters_.getNominalBaseVelocityBounds();
                    nominal_base_acceleration_bounds_ =   model.robot_parameters_.getNominalBaseAccelerationBounds();
                    body_bounds_ =                        model.robot_parameters_.getBodyBounds();

                    base_height_ = model.getBaseHeight();
                    body_height_ = model.getBodyHeight();

                    base_mass_ = model.getBaseMass();
                    body_mass_ = model.getBodyMass();

                    theta += motion_parameters.getBaseAngularVelocity()*T_;
                    theta_    = theta;
                    rotation_ = Eigen::Rotation2Dd(theta);

                    base_vel_ref_ = rotation_ * motion_parameters.getBaseVelocity();
                    base_pos_ref_ = motion_parameters.getBasePosition();
                }


                /**
                 * @brief Log
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger & logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string & name = "preview_interval") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    logger.log(LogEntryName(subname).add("base_height")         , base_height_      );
                    logger.log(LogEntryName(subname).add("body_height")         , body_height_      );
                    logger.log(LogEntryName(subname).add("base_mass")           , base_mass_        );
                    logger.log(LogEntryName(subname).add("body_mass")           , body_mass_        );
                    logger.log(LogEntryName(subname).add("T")             , T_          );
                    logger.log(LogEntryName(subname).add("T_ms")          , T_ms_       );
                    logger.log(LogEntryName(subname).add("base_vel_ref")   , base_vel_ref_);
                    logger.log(LogEntryName(subname).add("base_pos_ref")   , base_pos_ref_);
                    logger.log(LogEntryName(subname).add("theta")         , theta_      );
                    logger.log(LogEntryName(subname).add("rotation")  , rotation_);
                    logger.log(LogEntryName(subname).add("body_bounds"), body_bounds_);
                    logger.log(LogEntryName(subname).add("cop_bounds"), cop_bounds_);
                    logger.log(LogEntryName(subname).add("nominal_base_velocity_bounds"), nominal_base_velocity_bounds_);
                    logger.log(LogEntryName(subname).add("nominal_base_acceleration_bounds"), nominal_base_acceleration_bounds_);
                }
        };



        /**
         * @brief Preview horizon of an MPC [form_preview_horizon.m]
         */
        class HUMOTO_LOCAL PreviewHorizon
        {
            public:
                std::vector<PreviewHorizonInterval> intervals_;


            public:
                /**
                 * @brief Form the preview horizon object
                 *
                 * @param[in] mpc_params
                 * @param[in] model
                 * @param[in] motion_parameters
                 *
                 * @return true if successful
                 */
                bool form(  const MPCParameters         &mpc_params,
                            const Model                 &model,
                            const MotionParameters      &motion_parameters)
                {
                    bool preview_horizon_formed = false;


                    if  ((motion_parameters.duration_ms_ == MotionParameters::UNLIMITED_DURATION)
                         ||
                         ( static_cast<std::ptrdiff_t>(mpc_params.preview_horizon_length_ * mpc_params.sampling_time_ms_)
                                        <= motion_parameters.duration_ms_ ))
                    {
                        double theta = model.getBaseOrientation();

                        intervals_.resize(mpc_params.preview_horizon_length_);

                        for (std::size_t i = 0; i < mpc_params.preview_horizon_length_; ++i)
                        {
                            intervals_[i].initialize(theta, mpc_params, motion_parameters, model);
                        }

                        preview_horizon_formed = true;
                    }

                    return (preview_horizon_formed);
                }


                /**
                 * @brief Form the preview horizon object
                 *
                 * @param[in] mpc_params
                 * @param[in] model
                 * @param[in] motion_parameters_deque
                 *
                 * @return true if successful
                 */
                bool form(  const MPCParameters                 &mpc_params,
                            const Model                         &model,
                            const std::deque<MotionParameters>  &motion_parameters_deque)
                {
                    bool preview_horizon_formed = false;


                    double          theta = model.getBaseOrientation();
                    intervals_.resize(mpc_params.preview_horizon_length_);


                    std::size_t i = 0;
                    std::size_t remainder_ms = 0;
                    for (   std::deque<MotionParameters>::const_iterator it = motion_parameters_deque.begin();
                            it != motion_parameters_deque.end();
                            ++it)
                    {
                        if (it->duration_ms_ == MotionParameters::UNLIMITED_DURATION)
                        {
                            for (; i < mpc_params.preview_horizon_length_; ++i)
                            {
                                intervals_[i].initialize(theta, mpc_params, *it, model);
                            }
                            preview_horizon_formed = true;
                            break;
                        }
                        else
                        {
                            std::size_t number_of_iterations = std::min(
                                    static_cast<std::size_t>( (remainder_ms + it->duration_ms_) / mpc_params.sampling_time_ms_),
                                    mpc_params.preview_horizon_length_ - i);

                            for (std::size_t j = 0; j < number_of_iterations; ++i, ++j)
                            {
                                intervals_[i].initialize(theta, mpc_params, *it, model);
                            }

                            if (i == mpc_params.preview_horizon_length_)
                            {
                                preview_horizon_formed = true;
                                break;
                            }
                            else
                            {
                                remainder_ms = static_cast<std::size_t>(
                                        (remainder_ms + it->duration_ms_) % mpc_params.sampling_time_ms_);
                            }
                        }
                    }

                    return (preview_horizon_formed);
                }


                /**
                 * @brief Get preview horizon length
                 *
                 * @return preview horizon length
                 */
                std::size_t    getPreviewHorizonLength() const
                {
                    return (intervals_.size());
                }


                /**
                 * @brief Get nominal base velocity bounds for a given interval
                 *
                 * @param[in] interval_index
                 *
                 * @return 2d matrix [x_base_vel, y_base_vel]
                 */
                etools::Matrix2    getNominalBaseVelocityBounds(const std::size_t interval_index) const
                {
                    return (intervals_[interval_index].nominal_base_velocity_bounds_);
                }


                /**
                 * @brief Get nominal base acceleration bounds for a given interval
                 *
                 * @param[in] interval_index
                 *
                 * @return 2d matrix [x_base_acc, y_base_acc]
                 */
                etools::Matrix2    getNominalBaseAccelerationBounds(const std::size_t interval_index) const
                {
                    return (intervals_[interval_index].nominal_base_acceleration_bounds_);
                }


                /**
                 * @brief Get body bounds
                 *
                 * @param[in] interval_index
                 *
                 * @return 2d matrix [lb, ub]
                 */
                etools::Matrix2    getBodyBounds(const std::size_t interval_index) const
                {
                    return (intervals_[interval_index].body_bounds_);
                }


                /**
                 * @brief Get CoP bounds for given interval
                 *
                 * @param[in] interval_index
                 *
                 * @return 2d matrix [lb, ub]
                 */
                etools::Matrix2    getCoPBounds(const std::size_t interval_index) const
                {
                    return (intervals_[interval_index].cop_bounds_);
                }


                /**
                 * @brief Get base reference velocity
                 *
                 * @param[in] interval_index
                 *
                 * @return 2d vector [base_vel_ref_x, base_vel_ref_y]
                 */
                etools::Vector2    getBaseReferenceVelocity(const std::size_t interval_index) const
                {
                    return (intervals_[interval_index].base_vel_ref_);
                }


                /**
                 * @brief Get base reference position
                 *
                 * @param[in] interval_index
                 *
                 * @return 2d vector [base_pos_ref_x, base_pos_ref_y]
                 */
                etools::Vector2    getBaseReferencePosition(const std::size_t interval_index) const
                {
                    return (intervals_[interval_index].base_pos_ref_);
                }


                /**
                 * @brief Get orientation of the support corresponding to the given interval
                 *
                 * @param[in] interval_index
                 *
                 * @return 2d rotation matrix
                 */
                etools::Matrix2    getRotationMatrix(const std::size_t interval_index) const
                {
                    return (intervals_[interval_index].rotation_);
                }


                /**
                 * @brief Finds interval which contains the time instant with the given
                 * offset from the beginning of the preview horizon.
                 *
                 * @param[in,out] offset_ms input: offset within the preview horizon
                 *                          output: offset within the interval
                 *
                 * @return interval index
                 */
                std::size_t getIntervalIndexByTimeOffset(std::size_t   & offset_ms) const
                {
                    for (std::size_t interval_index = 0; interval_index < intervals_.size(); ++interval_index)
                    {
                        std::size_t interval_duration_ms = intervals_[interval_index].T_ms_;

                        if (offset_ms > interval_duration_ms)
                        {
                            offset_ms -= interval_duration_ms;
                        }
                        else
                        {
                            return (interval_index);
                        }
                    }

                    HUMOTO_THROW_MSG("The requested time offset is outside of the preview horizon.");
                }


                /**
                 * @brief Finds interval which contains the time instant with the given
                 * offset from the beginning of the preview horizon.
                 *
                 * @param[in,out] offset input: offset within the preview horizon
                 *                       output: offset within the interval
                 *
                 * @return interval index
                 */
                std::size_t getIntervalIndexByTimeOffset(double   & offset) const
                {
                    for (std::size_t interval_index = 0; interval_index < intervals_.size(); ++interval_index)
                    {
                        double interval_duration = intervals_[interval_index].T_;

                        if (offset > interval_duration)
                        {
                            offset -= interval_duration;
                        }
                        else
                        {
                            return (interval_index);
                        }
                    }

                    HUMOTO_THROW_MSG("The requested time offset is outside of the preview horizon.");
                }


                /**
                 * @brief Log
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger & logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string & name = "preview_horizon") const
                {
                    LogEntryName subname = parent;
                    subname.add(name).add("interval");

                    // intervals
                    for (std::size_t i = 0; i < intervals_.size(); ++i)
                    {
                        intervals_[i].log(logger, LogEntryName(subname).add(i), "");
                    }
                }
        };
    } // end namespace pepper_mpc
}// end namespace humoto
