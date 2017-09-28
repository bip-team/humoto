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
         * @brief
         */
        class HUMOTO_LOCAL Model :  public humoto::Model,
                                    public humoto::pepper_mpc::TwoPointMassModel
        {
            private:
                /**
                 * @brief Determines position of the base.
                 */
                void finalize()
                {
                    current_base_position_ = state_.base_state_.position_.head(2);
                }


            public:
                /// state of the model
                humoto::pepper_mpc::ModelState          state_;

                /// robot parameters
                humoto::pepper_mpc::RobotParameters     robot_parameters_;

                /// 2d position of the base
                etools::Vector2             current_base_position_;


            public:
                /**
                 * @brief Default constructor
                 */
                Model()
                {
                    finalize();
                }


                /**
                 * @brief Construct and initialize robot parameters
                 */
                explicit Model(const humoto::pepper_mpc::RobotParameters &robot_parameters)
                {
                    setParameters(robot_parameters);
                }


                /**
                 * @brief Initialize robot parameters
                 */
                void setParameters(const humoto::pepper_mpc::RobotParameters &robot_parameters)
                {
                    robot_parameters_ = robot_parameters;
                    finalize();
                }



                /**
                 * @brief Returns current body state.
                 *
                 * @return body state
                 */
                humoto::rigidbody::PointMassState   getBodyState() const
                {
                    return (state_.body_state_);
                }


                /**
                 * @brief Returns current base state.
                 *
                 * @return base state
                 */
                humoto::rigidbody::RigidBodyState   getBaseState() const
                {
                    return (state_.base_state_);
                }


                /**
                 * @brief Get MPC state
                 */
                EIGENTOOLS_CONSTANT_SIZE_VECTOR(12) getMPCState() const
                {
                    EIGENTOOLS_CONSTANT_SIZE_VECTOR(12) mpc_state;

                    mpc_state = convertStateRBtoMPC(state_.base_state_, state_.body_state_);
                    return(mpc_state);
                }


                /**
                 * @brief Get base state
                 *
                 * @return base state
                 */
                etools::Vector6 getMPCBaseState() const
                {
                    return(getMPCBaseState(state_));
                }


                /**
                 * @brief Get body state
                 *
                 * @return body state
                 */
                etools::Vector6 getMPCBodyState() const
                {
                    return(getMPCBodyState(state_));
                }



                /**
                 * @brief Get base state
                 *
                 * @param[in] model_state
                 *
                 * @return base state
                 */
                etools::Vector6 getMPCBaseState(const humoto::pepper_mpc::ModelState & model_state) const
                {
                    etools::Vector6 mpc_state;

                    mpc_state = convertBaseStateRBtoMPC(model_state.base_state_);
                    return(mpc_state);
                }


                /**
                 * @brief Get body state
                 *
                 * @param[in] model_state
                 *
                 * @return body state
                 */
                etools::Vector6 getMPCBodyState(const humoto::pepper_mpc::ModelState & model_state) const
                {
                    etools::Vector6 mpc_state;

                    mpc_state = convertBodyStateRBtoMPC(model_state.body_state_);
                    return(mpc_state);
                }


                /**
                 * @brief Get CoP position
                 *
                 * @return CoP position
                 */
                etools::Vector2 getCoP() const
                {
                    return (getCoP(state_));
                }


                /**
                 * @brief Get CoP position
                 *
                 * @param[in] model_state
                 *
                 * @return CoP position
                 */
                etools::Vector2 getCoP(const humoto::pepper_mpc::ModelState & model_state) const
                {
                    etools::Matrix1x3   Dps = getDps3(model_state.base_state_.position_.z(),
                                                      model_state.base_mass_,
                                                      model_state.body_mass_);

                    etools::Matrix1x3   Dpd = getDpd3(model_state.body_state_.position_.z(),
                                                      model_state.base_mass_,
                                                      model_state.body_mass_);

                    etools::Vector2 cop;

                    cop =   etools::GenericBlockKroneckerProduct<1,3>(Dps, 2)*getMPCBaseState(model_state)
                            +
                            etools::GenericBlockKroneckerProduct<1,3>(Dpd, 2)*getMPCBodyState(model_state);

                    return (cop);
                }



                /**
                 * @brief Get next state from the current state
                 *
                 * @param[out] model_state          next model state
                 * @param[in] T                     timestep
                 * @param[in] Ts                    subsampling timestep (use Ts = T if not needed)
                 * @param[in] base_control          control vector
                 * @param[in] body_control          control vector
                 * @param[in] base_height           base height
                 * @param[in] body_height           body height
                 * @param[in] theta                 orientation of the base
                 */
                void getNextState(  humoto::pepper_mpc::ModelState & model_state,
                                    const double T,
                                    const double Ts,
                                    const etools::Vector2 &base_control,
                                    const etools::Vector2 &body_control,
                                    const double base_height,
                                    const double body_height,
                                    const double theta) const
                {
                    getNextState( model_state,
                                  T,
                                  Ts,
                                  getMPCState(),
                                  base_control,
                                  body_control,
                                  base_height,
                                  body_height,
                                  theta);
                }



                /**
                 * @brief Get next state from the given state
                 *
                 * @param[out] model_state          next model state
                 * @param[in] T                     timestep
                 * @param[in] Ts                    subsampling timestep (use Ts = T if not needed)
                 * @param[in] preceding_mpcstate    mpc state
                 * @param[in] base_control          control vector
                 * @param[in] body_control          control vector
                 * @param[in] base_height           base height
                 * @param[in] body_height           body height
                 * @param[in] theta                 orientation of the base
                 */
                void getNextState(  humoto::pepper_mpc::ModelState & model_state,
                                    const double T,
                                    const double Ts,
                                    const EIGENTOOLS_CONSTANT_SIZE_VECTOR(12) & preceding_mpcstate,
                                    const etools::Vector2 &base_control,
                                    const etools::Vector2 &body_control,
                                    const double base_height,
                                    const double body_height,
                                    const double theta) const
                {
                    EIGENTOOLS_CONSTANT_SIZE_VECTOR(12) mpc_state;

                    mpc_state = evaluate(T, Ts, preceding_mpcstate, base_control, body_control);

                    model_state.base_state_ = convertBaseStateMPCtoRB(mpc_state, base_height);
                    model_state.body_state_ = convertBodyStateMPCtoRB(mpc_state, body_height);

                    model_state.base_state_.rpy_.z() = theta;
                }



                /**
                 * @brief Get theta angle of the current state
                 *
                 * @return theta angle
                 */
                double getTheta() const
                {
                    return(getTheta(state_));
                }


                /**
                 * @brief Get theta angle of the given state
                 *
                 * @param[in] model_state
                 *
                 * @return theta angle
                 */
                double getTheta(const humoto::pepper_mpc::ModelState & model_state) const
                {
                    return(model_state.base_state_.rpy_.z());
                }


                /**
                 * @brief Get body height
                 *
                 * @return body height
                 */
                double getBodyHeight() const
                {
                    return (state_.body_state_.position_.z());
                }


                /**
                 * @brief Get base height
                 *
                 * @return base height
                 */
                double getBaseHeight() const
                {
                    return (state_.base_state_.position_.z());
                }



                /**
                 * @brief Get body mass
                 *
                 * @return body mass
                 */
                double getBodyMass() const
                {
                    return (state_.body_mass_);
                }


                /**
                 * @brief Get base mass
                 *
                 * @return base mass
                 */
                double getBaseMass() const
                {
                    return (state_.base_mass_);
                }


                /**
                 * @brief Get base orientation
                 *
                 * @return base orientation
                 */
                double getBaseOrientation() const
                {
                    return (state_.base_state_.rpy_.z());
                }


                /**
                 * @brief Update model state.
                 *
                 * @param[in] model_state model state.
                 */
                void    updateState(const humoto::ModelState &model_state)
                {
                    const humoto::pepper_mpc::ModelState &state = dynamic_cast <const humoto::pepper_mpc::ModelState &> (model_state);
                    state_ = state;
                    finalize();
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
                            const std::string &name = "model") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    state_.log(logger, subname, "state");

                    logger.log(LogEntryName(subname).add("mpc_state")            , getMPCState());
                    logger.log(LogEntryName(subname).add("current_base_position"), current_base_position_);
                }
        };
    }
}
