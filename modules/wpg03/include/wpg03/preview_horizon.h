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
        /**
         * @brief A helper class defining one interval of a preview horizon
         */
        class PreviewHorizonInterval
        {
            public:
                unsigned int state_index_;       /// in TDS points to the next SS
                etools::Vector2 cvel_ref_;
                double hg_;                     /// Height of the CoM / gravitational acceleration
                double omega_;
                double T_;


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

                    logger.log(LogEntryName(subname).add("state_index"), state_index_);
                    logger.log(LogEntryName(subname).add("hg")         , hg_         );
                    logger.log(LogEntryName(subname).add("omega")      , omega_      );
                    logger.log(LogEntryName(subname).add("T")          , T_          );

                    logger.log(LogEntryName(subname).add("cvel_ref"), cvel_ref_);
                }
        };


        /**
         * @brief Preview horizon of an MPC [form_preview_horizon.m]
         */
        class PreviewHorizon
        {
            public:
                std::vector<unsigned int>   variable_steps_indices_;


                std::vector<PreviewHorizonInterval> intervals_;
                std::vector<WalkState> states_;



                bool form(  const MPCParameters &mpc_params,
                            const Model &model,
                            const WalkFiniteStateMachine &walk_fsm);


                unsigned int    getNumberOfVariableSteps() const;

                void log(humoto::Logger &, const LogEntryName &, const std::string &) const;


            private:
                void get_constraints(const Model &model,
                                     const WalkFiniteStateMachine &walk_fsm,
                                     unsigned int walk_index,
                                     etools::Matrix2 &fd_bounds,
                                     etools::Matrix2 &cop_bounds);
        };


        /**
         * @brief Returns number of variable steps in the preview.
         *
         * @return number of variable steps in the preview
         */
        unsigned int    PreviewHorizon::getNumberOfVariableSteps() const
        {
            return (variable_steps_indices_.size());
        }


        /**
         * @brief Form the preview horizon object
         *
         * @param[in] model
         * @param[in] mpc_params
         * @param[in] walk_fsm
         *
         */
        bool PreviewHorizon::form(  const MPCParameters &mpc_params,
                                    const Model &model,
                                    const WalkFiniteStateMachine &walk_fsm)
        {
            bool                preview_horizon_formed = true;


            // Length of the preview horizon (N)
            unsigned int        preview_horizon_length_ = mpc_params.preview_horizon_length_;

            unsigned int        walk_index = walk_fsm.index_;

            unsigned int        num_intervals_left = 0;


            //is_new_sample_ = walk_fsm.is_new_sample_;
            states_.clear();
            variable_steps_indices_.clear();

            std::vector<unsigned int> support_indices;
            unsigned int pwin_state_index = 0;

            intervals_.resize(preview_horizon_length_);

            unsigned int interval_index = 0;
            while(interval_index < preview_horizon_length_)
            {
                if(walk_index >= walk_fsm.states_.size())
                {
                    preview_horizon_formed = false;
                    return (preview_horizon_formed);
                }

                WalkState state = walk_fsm.states_.at(walk_index);

                PreviewHorizonInterval  interval;
                interval.state_index_ = pwin_state_index;          // in TDS points to the next SS
                interval.cvel_ref_   = state.R_cvel_ref_;
                interval.hg_         = model.getComHeight()/ humoto::g_gravitational_acceleration;  // Height of the CoM / gravitational acceleration
                interval.omega_      = std::sqrt(1./interval.hg_);

                support_indices.push_back(interval_index);

                if (pwin_state_index == 0)
                {
                    // current sampling interval (the 1st in the preview window)
                    if (state.type_ == STATE_TDS)
                    {
                        interval.T_ = convertMillisecondToSecond(walk_fsm.current_step_duration_ms_);
                        ++walk_index;
                        state = walk_fsm.states_.at(walk_index);
                        num_intervals_left = floor(state.duration_ms_ / mpc_params.sampling_time_ms_);
                    }
                    else
                    {
                        interval.T_         = convertMillisecondToSecond(walk_fsm.current_step_duration_ms_ % mpc_params.sampling_time_ms_);
                        num_intervals_left = floor(walk_fsm.current_step_duration_ms_ / mpc_params.sampling_time_ms_);

                        if ((walk_fsm.current_step_duration_ms_ % mpc_params.sampling_time_ms_) == 0)
                        {
                            interval.T_ = mpc_params.getSamplingTime();
                            --num_intervals_left;
                        }
                    }

                    intervals_[interval_index] = interval;
                    ++interval_index;
                }
                else
                {
                    if (state.type_ != STATE_TDS)
                    {
                        HUMOTO_THROW_MSG("Unexpected state type!");
                    }
                    // new variable footstep
                    variable_steps_indices_.push_back(interval_index);

                    // transitional DS
                    interval.T_ = mpc_params.getTDSSamplingTime();


                    intervals_[interval_index] = interval;
                    ++interval_index;

                    // next SS
                    ++walk_index;
                    state = walk_fsm.states_.at(walk_index);

                    num_intervals_left = floor(state.duration_ms_ / mpc_params.sampling_time_ms_);
                }

                interval.T_ = mpc_params.getSamplingTime();

                // fill the other intervals
                unsigned int max_index = std::min(preview_horizon_length_, interval_index + num_intervals_left);
                for(unsigned int i = interval_index; i < max_index; ++i)
                {
                    intervals_[i] = interval;
                }

                interval_index += num_intervals_left;
                get_constraints(model, walk_fsm, walk_index, state.fd_bounds_, state.cop_bounds_);
                states_.push_back(state);
                ++pwin_state_index;
                ++walk_index;
            }

            return (preview_horizon_formed);
        }


        /**
         * @brief Utility Function
         *
         * @param[in] model
         * @param[in] walk_fsm
         * @param[in] walk_index
         * @param[out] fd_bounds
         * @param[out] cop_bounds
         */
        void PreviewHorizon::get_constraints(const Model &model,
                                             const WalkFiniteStateMachine &walk_fsm,
                                             unsigned int walk_index,
                                             etools::Matrix2 &fd_bounds,
                                             etools::Matrix2 &cop_bounds)
        {
            ModelStateType current_state_type = walk_fsm.states_.at(walk_index).type_;
            if (walk_index == 0)
            {
                fd_bounds << 0., 0., 0., 0.;
            }
            else
            {
                ModelStateType preceding_state_type = walk_fsm.states_.at(walk_index-2).type_;
                switch(current_state_type)
                {
                    case STATE_LSS:
                        switch(preceding_state_type)
                        {
                            case STATE_DS:
                                fd_bounds = model.getFootBounds(humoto::LeftOrRight::LEFT, humoto::walking::FootBoundsType::WITH_RESPECT_TO_ADS);
                                break;
                            case STATE_RSS:
                                fd_bounds = model.getFootBounds(humoto::LeftOrRight::LEFT, humoto::walking::FootBoundsType::WITH_RESPECT_TO_SS);
                                break;
                            default:
                                HUMOTO_THROW_MSG("unexpected support type");
                                break;
                        }
                        break;

                    case STATE_RSS:
                        switch(preceding_state_type)
                        {
                            case STATE_DS:
                                fd_bounds = model.getFootBounds(humoto::LeftOrRight::RIGHT, humoto::walking::FootBoundsType::WITH_RESPECT_TO_ADS);
                                break;
                            case STATE_LSS:
                                fd_bounds = model.getFootBounds(humoto::LeftOrRight::RIGHT, humoto::walking::FootBoundsType::WITH_RESPECT_TO_SS);
                                break;
                            default:
                                HUMOTO_THROW_MSG("unexpected support type");
                                break;
                        }
                        break;

                    case STATE_DS:
                        switch(preceding_state_type)
                        {
                            case STATE_LSS:
                                fd_bounds = model.getFootBounds(humoto::LeftOrRight::RIGHT, humoto::walking::FootBoundsType::ALIGNED_TO_SS);
                                break;
                            case STATE_RSS:
                                fd_bounds = model.getFootBounds(humoto::LeftOrRight::LEFT, humoto::walking::FootBoundsType::ALIGNED_TO_SS);
                                break;
                            default:
                                HUMOTO_THROW_MSG("unexpected support type");
                                break;
                        }
                        break;

                    default:
                        HUMOTO_THROW_MSG("unexpected support type");
                        break;
                }
            }


            if (current_state_type == STATE_DS)
            {
                cop_bounds = model.getADSCoPBounds();
            }
            else
            {
                cop_bounds = model.getSSCoPBounds();
            }
        }


        /**
         * @brief Log
         *
         * @param[in,out] logger logger
         * @param[in] parent parent
         * @param[in] name name
         */
        void PreviewHorizon::log(   humoto::Logger & logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                    const LogEntryName &parent = LogEntryName(),
                                    const std::string & name = "preview_horizon") const
        {
            LogEntryName subname_loop;
            LogEntryName subname = parent;
            subname.add(name);

            // variable_steps_indices
            logger.log(LogEntryName(subname).add("variable_steps_indices"), variable_steps_indices_);

            // intervals
            subname_loop = subname;
            subname_loop.add("interval");
            for (unsigned int i = 0; i < intervals_.size(); ++i)
            {
                intervals_[i].log(logger, LogEntryName(subname_loop).add(i), "");
            }


            // states
            subname_loop = subname;
            subname_loop.add("state");
            for (unsigned int i = 0; i < states_.size(); ++i)
            {
                states_[i].log(logger, LogEntryName(subname_loop).add(i), "");
            }
        }
    } // end namespace wpg03
}// end namespace humoto
