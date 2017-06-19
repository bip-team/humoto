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
         * @brief A class containing some parameters of the walking pattern generator
         */
        class WalkOptions
        {
            public:
                ModelStateType first_ss_type_;

                etools::Vector2 com_velocity_;
                etools::Vector2 first_ds_com_velocity_;
                etools::Vector2 last_ds_com_velocity_;

                double step_height_;

                unsigned int ss_duration_ms_;
                unsigned int tds_duration_ms_;
                unsigned int first_ds_duration_ms_;
                unsigned int last_ds_duration_ms_;

                double theta_inc_;

                unsigned int num_steps_;



                /**
                 * @brief Default constructor;
                 */
                WalkOptions()
                {
                    setDefault();
                }


                /**
                 * @brief Set default parameters of the FSM.
                 */
                void setDefault()
                {
                    first_ss_type_ = STATE_RSS;

                    ss_duration_ms_ = 700;
                    tds_duration_ms_ = 100;
                    first_ds_duration_ms_ = 300;
                    last_ds_duration_ms_ = 2900;

                    com_velocity_ << 0.2, 0.;
                    first_ds_com_velocity_ << 0.2, 0.;
                    last_ds_com_velocity_ << 0., 0.;

                    theta_inc_ = 0.;

                    num_steps_ = 7;
                }
        };



        /**
         * @brief A class representing a "state" of the walk
         */
        class WalkState
        {
            public:
                ModelStateType      type_;
                unsigned int        duration_ms_;
                etools::Vector2     cvel_ref_;

                double              theta_;
                etools::Matrix2     rotation_;

                etools::Vector2     R_cvel_ref_;

                double              step_height_;

                etools::Matrix2     fd_bounds_;
                etools::Matrix2     cop_bounds_;


                WalkState()
                {
                    fd_bounds_ << 0., 0., 0., 0.;
                    cop_bounds_ << 0., 0., 0., 0.;
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
                            const std::string & name = "walk_state") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    logger.log(LogEntryName(subname).add("type")         , type_       );
                    logger.log(LogEntryName(subname).add("duration_ms")  , duration_ms_);
                    logger.log(LogEntryName(subname).add("theta")        , theta_      );
                    logger.log(LogEntryName(subname).add("step_height")  , step_height_);

                    logger.log(LogEntryName(subname).add("cvel_ref")  , cvel_ref_);
                    logger.log(LogEntryName(subname).add("rotation")  , rotation_);
                    logger.log(LogEntryName(subname).add("R_cvel_ref"), R_cvel_ref_);
                    logger.log(LogEntryName(subname).add("fd_bounds") , fd_bounds_);
                    logger.log(LogEntryName(subname).add("cop_bounds"), cop_bounds_);
                }
        };



        /**
         * @brief Finite state machine for walking. [initialize_walk_fsm.m]
         */
        class WalkFiniteStateMachine
        {
            public:
                std::vector<WalkState> states_;
                unsigned int index_;
                bool is_new_sample_;
                bool is_end_of_ss_;
                bool is_end_of_fixed_ds_;
                bool is_last_ss;
                unsigned int current_step_duration_ms_;



                WalkFiniteStateMachine( const Model &model,
                                        const MPCParameters &mpc_param,
                                        const WalkOptions &walk_options);

                WalkFiniteStateMachine( const Model &model,
                                        const MPCParameters &mpc_param);

                void initialize(const Model &model,
                                const MPCParameters &mpc_param,
                                const WalkOptions &walk_options);

                void setCoMVelocity(const etools::Vector2 &com_velocity);
                void setThetaIncrement(const double &theta_inc);
                void stopWalking();

                void update(const MPCParameters &mpc_param);

                humoto::wpg03::ModelStateType  getSupportStateType(const unsigned int) const;

                void log(   humoto::Logger &, const humoto::LogEntryName&, const std::string &) const;


            private:
                double init_theta_;
        };



        /**
         * @brief Returns support state type with given index
         *
         * @param[in] index index of the state (0 = current state)
         *
         * @return support state type
         */
        humoto::wpg03::ModelStateType  WalkFiniteStateMachine::getSupportStateType(const unsigned int   index = 0) const
        {
            if (index_ + index >= states_.size())
            {
                return (humoto::wpg03::STATE_UNDEFINED);
            }
            else
            {
                return (states_[index_ + index].type_);
            }
        }



        /**
         * @brief Constructor
         *
         * @param[in] model
         * @param[in] mpc_param
         */
        WalkFiniteStateMachine::WalkFiniteStateMachine(const Model &model, const MPCParameters &mpc_param)
        {
            WalkOptions default_walk_options;
            default_walk_options.setDefault();
            initialize(model, mpc_param, default_walk_options);
        }


        /**
         * @brief Constructor
         *
         * @param[in] model
         * @param[in] mpc_param
         * @param[in] walk_options
         */
        WalkFiniteStateMachine::WalkFiniteStateMachine( const Model &model,
                                                        const MPCParameters &mpc_param,
                                                        const WalkOptions &walk_options)
        {
            initialize(model, mpc_param, walk_options);
        }


        /**
         * @brief Initialize the FSM
         *
         * @param[in] model
         * @param[in] mpc_param
         * @param[in] walk_options
         */
        void WalkFiniteStateMachine::initialize(const Model &model,
                                                const MPCParameters &mpc_param,
                                                const WalkOptions &walk_options)
        {
            if((walk_options.ss_duration_ms_ % mpc_param.sampling_time_ms_) != 0 ||
               (walk_options.tds_duration_ms_ != mpc_param.tds_sampling_time_ms_) ||
               (walk_options.first_ds_duration_ms_ % mpc_param.sampling_time_ms_) != 0 ||
               (walk_options.last_ds_duration_ms_ % mpc_param.sampling_time_ms_) != 0)
            {
                HUMOTO_THROW_MSG("Incorrectly specified duration of a state!");
            }

            if (walk_options.num_steps_ == 1)
            {
                HUMOTO_THROW_MSG("Unsupported number of steps.");
            }

            states_.clear();

            /// Assume aligned feet in DS
            init_theta_ = ( model.left_foot_.rpy_(AngleIndex::YAW) + model.right_foot_.rpy_(AngleIndex::YAW) ) / 2.;

            WalkState state;


            state.type_          = model.state_type_;

            if (state.type_ != STATE_DS)
            {
                HUMOTO_THROW_MSG("Initial state must be DS!");
            }

            state.duration_ms_  = walk_options.first_ds_duration_ms_;
            state.cvel_ref_     = walk_options.first_ds_com_velocity_;
            state.theta_        = init_theta_;

            state.rotation_     = Eigen::Rotation2Dd(state.theta_);
            state.R_cvel_ref_   = state.rotation_ * state.cvel_ref_;

            state.step_height_  = 0.;
            states_.push_back(state);

            if (walk_options.num_steps_ > 0)
            {
                state.type_         = STATE_TDS;
                state.duration_ms_  = walk_options.tds_duration_ms_;
                state.cvel_ref_     = walk_options.com_velocity_;
                state.theta_        = init_theta_;

                state.rotation_     = Eigen::Rotation2Dd(state.theta_);
                state.R_cvel_ref_   = state.rotation_ * state.cvel_ref_;

                state.step_height_  = 0.;

                states_.push_back(state);


                state.type_          = walk_options.first_ss_type_;
                state.duration_ms_   = walk_options.ss_duration_ms_;
                state.step_height_   = walk_options.step_height_;

                states_.push_back(state);
            }

            for(unsigned int i=2; i<walk_options.num_steps_ - 2; ++i)
            {
                ModelStateType next_ss_type;
                if(states_.back().type_ == STATE_RSS)
                {
                    next_ss_type = STATE_LSS;
                }
                else
                {
                    next_ss_type = STATE_RSS;
                }

                state.type_          = STATE_TDS;
                state.duration_ms_   = walk_options.tds_duration_ms_;
                state.cvel_ref_      = walk_options.com_velocity_;
                state.theta_         = states_.back().theta_ + walk_options.theta_inc_;

                state.rotation_      = Eigen::Rotation2Dd(state.theta_);
                state.R_cvel_ref_    = state.rotation_ * state.cvel_ref_;

                state.step_height_   = 0.;

                states_.push_back(state);


                state.type_          = next_ss_type;
                state.duration_ms_   = walk_options.ss_duration_ms_;
                state.step_height_   = walk_options.step_height_;

                states_.push_back(state);
            }

            if (walk_options.num_steps_ > 0)
            {
                state.type_          = STATE_TDS;
                state.duration_ms_   = walk_options.tds_duration_ms_;
                state.cvel_ref_      = walk_options.last_ds_com_velocity_;
                state.theta_         = states_.back().theta_;

                state.rotation_      = Eigen::Rotation2Dd(state.theta_);
                state.R_cvel_ref_    = state.rotation_ * state.cvel_ref_;

                state.step_height_   = 0.;

                states_.push_back(state);


                state.type_          = STATE_DS;
                state.duration_ms_   = walk_options.last_ds_duration_ms_;

                states_.push_back(state);
            }


            index_ = 0; // current step index

            is_new_sample_ = true;
            is_end_of_ss_ = false;
            is_end_of_fixed_ds_ = false;
            is_last_ss = false;

            current_step_duration_ms_  = states_.at(index_).duration_ms_;
        }



        /**
         * @brief Set a new target com velocity
         */
        void WalkFiniteStateMachine::setCoMVelocity(const etools::Vector2 &com_velocity)
        {
            for(unsigned int i=index_; i < states_.size(); ++i)
            {
                switch(states_.at(i).type_)
                {
                    case STATE_RSS:
                    case STATE_LSS:
                        states_.at(i).cvel_ref_     = com_velocity;
                        states_.at(i).R_cvel_ref_   = states_.at(i).rotation_ * com_velocity;
                        break;
                    case STATE_TDS:
                        /// this also changes the com_velocity of the last TDS (it should not)
                        states_.at(i).cvel_ref_     = com_velocity;
                        states_.at(i).R_cvel_ref_   = states_.at(i).rotation_ * com_velocity;
                        break;
                    default:
                        // do nothing
                        break;
                }
            }
        }



        /**
         * @brief Set a new theta increment for turning the robot
         */
        void WalkFiniteStateMachine::setThetaIncrement(const double &theta_inc)
        {
            // initialize from the current state
            double current_theta = states_.at(index_).theta_;
            etools::Matrix2 current_rotation = states_.at(index_).rotation_;

            // update future states
            for(unsigned int i=index_; i < states_.size(); ++i)
            {
                switch(states_.at(i).type_)
                {
                    case STATE_RSS:
                    case STATE_LSS:
                        states_.at(i).theta_ = current_theta;
                        states_.at(i).rotation_ = current_rotation;
                        states_.at(i).R_cvel_ref_ = current_rotation * states_.at(i).cvel_ref_;
                        break;
                    case STATE_TDS:
                        current_theta += theta_inc;
                        current_rotation = Eigen::Rotation2Dd(current_theta);

                        states_.at(i).theta_ = current_theta;
                        states_.at(i).rotation_ = current_rotation;
                        states_.at(i).R_cvel_ref_ = current_rotation * states_.at(i).cvel_ref_;
                        break;
                    default:
                        // do nothing
                        break;
                }
            }
        }



        /**
         * @brief Stops the walking PG by removing states that are safe to remove
         */
        void WalkFiniteStateMachine::stopWalking()
        {
            // Make sure there are states to be removed
            if(index_ < (states_.size()-7))
            {
                ModelStateType last_ss_type = states_.at(states_.size() - 3).type_;
                ModelStateType current_state_type = states_.at(index_).type_;
                ModelStateType next_state_type = states_.at(index_+1).type_;
                unsigned int states_to_keep = 0;

                switch(current_state_type)
                {
                    case STATE_RSS:
                    case STATE_LSS:
                        if(current_state_type ==  last_ss_type)
                        {
                            states_to_keep = 6;
                        }
                        else
                        {
                            states_to_keep = 4;
                        }
                        break;
                    case STATE_TDS:
                        if(next_state_type == last_ss_type)
                        {
                            states_to_keep = 3;
                        }
                        else
                        {
                            states_to_keep = 5;
                        }
                        break;
                    default:
                        // do nothing
                        break;
                }
                // fast-forward the current state to a similar one near the end
                states_.erase(states_.begin()+index_+1, states_.end()-states_to_keep);

                // Correct rotation of last states
                /// velocities might need correcting
                double current_theta = states_.at(index_).theta_;
                etools::Matrix2 current_rotation = states_.at(index_).rotation_;
                for(unsigned int i=states_.size()-states_to_keep; i<states_.size(); ++i)
                {
                    states_.at(i).theta_ = current_theta;
                    states_.at(i).rotation_ = current_rotation;
                }
            }
        }



        /**
         * @brief Update the state machine
         */
        void WalkFiniteStateMachine::update(const MPCParameters &mpc_param)
        {
            is_end_of_ss_ = false;
            is_end_of_fixed_ds_ = false;
            is_new_sample_ = false;


            current_step_duration_ms_  = current_step_duration_ms_ - mpc_param.subsampling_time_ms_;

            if ((current_step_duration_ms_ % mpc_param.sampling_time_ms_) == 0)
            {
                is_new_sample_ = true;
            }

            if (current_step_duration_ms_ == 0)
            {
                index_                      = index_ + 1;
                current_step_duration_ms_   = states_.at(index_).duration_ms_;


                switch(states_.at(index_-1).type_)
                {
                    case STATE_RSS:
                    case STATE_LSS:
                        is_end_of_ss_ = true;
                        break;
                    case STATE_DS:
                        is_end_of_fixed_ds_ = true;
                        break;
                    default:
                        // do nothing
                        break;
                }
            }

            // This assumes that the ending state sequence is: SS - TDS - DS
            if (index_ == states_.size() - 3)
            {
                is_last_ss = true;
            }
        }


        /**
         * @brief Log
         *
         * @param[in,out] logger logger
         * @param[in] parent parent
         * @param[in] name name
         */
        void WalkFiniteStateMachine::log(   humoto::Logger & logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                            const LogEntryName &parent = LogEntryName(),
                                            const std::string & name = "walk_fsm") const
        {
            LogEntryName subname = parent; subname.add(name);

            // states
            LogEntryName subname_loop = subname;
            subname_loop.add("state");
            for (unsigned int i = 0; i < states_.size(); ++i)
            {
                states_[i].log(logger, LogEntryName(subname).add(i), "");
            }


            logger.log(LogEntryName(subname).add("index")                   , index_                   );
            logger.log(LogEntryName(subname).add("is_new_sample")           , is_new_sample_           );
            logger.log(LogEntryName(subname).add("is_end_of_ss")            , is_end_of_ss_            );
            logger.log(LogEntryName(subname).add("is_end_of_fixed_ds")      , is_end_of_fixed_ds_      );
            logger.log(LogEntryName(subname).add("current_step_duration_ms"), current_step_duration_ms_);
        }

    }// end namespace wpg03
}// end namespace humoto

