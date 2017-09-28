/**
    @file
    @author  Alexander Sherikov
    @author  Don Joven Agravante
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA, 2014-2015 CNRS. Licensed under the Apache
    License, Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @brief
*/

#pragma once

namespace humoto
{
    namespace wpg04
    {
        /**
         * @brief Model Predictive Control problem for walking pattern generation [determine_solution_structure.m,
         *  form_rotation_matrices.m, form_foot_pos_matrices.m, form_condensing_matrices.m]
         */
        class HUMOTO_LOCAL MPCforWPG : public humoto::MPC
        {
            private:
                bool solution_is_parsed_;
                humoto::wpg04::MPCParameters    mpc_parameters_;


            private:
                /**
                 * @brief Create the rotation matrices of the MPC problem
                 */
                void formRotationMatrices()
                {
                    R_.setZero(preview_horizon_.getNumberOfVariableSteps());
                    Rh_.setZero(preview_horizon_.intervals_.size());

                    std::size_t index = 0;
                    for(std::size_t i=0; i < (preview_horizon_.walk_states_.size() - 1); ++i)
                    {
                        if (humoto::walking::StanceType::TDS != preview_horizon_.walk_states_[i].type_)
                        {
                            // orientations of the preceding foot positions.
                            R_(index) = preview_horizon_.walk_states_.at(i).rotation_;
                            ++index;
                        }
                    }


                    for(std::size_t i=0; i < preview_horizon_.intervals_.size(); ++i)
                    {
                        Rh_(i) = preview_horizon_.getRotationMatrix(i);
                    }
                }


                /**
                 * @brief Create the foot position matrices
                 *
                 * @param[in] model
                 */
                void formFootPosMatrices(const humoto::wpg04::Model &model)
                {
                    // generate_fixed_step_vector
                    V0_ = model.current_support_position_.replicate(preview_horizon_.intervals_.size(), 1);


                    Ir_.setZero(preview_horizon_.intervals_.size(), preview_horizon_.getNumberOfVariableSteps());

                    for(std::size_t i=0; i < preview_horizon_.variable_steps_indices_.size(); ++i)
                    {
                        EigenIndex row_size = preview_horizon_.intervals_.size() - preview_horizon_.variable_steps_indices_[i];
                        Ir_.block(preview_horizon_.variable_steps_indices_[i], i, row_size, 1).setConstant(1);
                    }

                    etools::GenericBlockKroneckerProduct<1,1> Ir_bmi(Ir_, 2);

                    V_ = Ir_bmi * R_;

                    Vfp_.noalias() = Eigen::Matrix2d::Identity().replicate(
                            preview_horizon_.getNumberOfVariableSteps(),
                            preview_horizon_.getNumberOfVariableSteps()).triangularView<Eigen::Lower>() * R_.getRaw();

                    vfp_ = model.current_support_position_.replicate(preview_horizon_.getNumberOfVariableSteps(), 1);
                }


                /**
                 * @brief Initialize next model state
                 *
                 * @param[in] stance_fsm  walking finite state machine (must be updated before)
                 * @param[in] model model
                 *
                 * @return next model state.
                 */
                humoto::wpg04::ModelState   initializeNextModelState(
                        const humoto::walking::StanceFiniteStateMachine &stance_fsm,
                        const humoto::wpg04::Model                      &model) const
                {
                    humoto::wpg04::ModelState model_state;

                    double T     = preview_horizon_.intervals_[0].T_;
                    double Ts    = mpc_parameters_.getSubsamplingTime();
                    double com_height = preview_horizon_.intervals_[0].com_height_;


                    model_state.com_state_ = model.evaluate(Ts,
                                                            T,
                                                            com_height,
                                                            model.getCState(),
                                                            cop_profile.segment(0, model.Nu_));

                    model_state.stance_type_        = stance_fsm.current_stance_.type_;

                    model_state.next_stance_type_   = (stance_fsm.getNextStance()).type_;


                    switch(model_state.stance_type_) // new
                    {
                        case humoto::walking::StanceType::LSS:
                            model_state.feet_.getRight().unset();
                            model_state.feet_.setLeft(model.getFootState(LeftOrRight::LEFT));
                            break;

                        case humoto::walking::StanceType::RSS:
                            model_state.feet_.getLeft().unset();
                            model_state.feet_.setRight(model.getFootState(LeftOrRight::RIGHT));
                            break;

                        case humoto::walking::StanceType::TDS:
                            if(stance_fsm.is_tds_started_ && (humoto::walking::StanceType::DS != model.state_.stance_type_))
                            {
                                LeftOrRight::Type landing_foot;
                                LeftOrRight::Type support_foot;
                                switch (model.state_.stance_type_) // old
                                {
                                    case humoto::walking::StanceType::LSS:
                                        landing_foot = humoto::LeftOrRight::RIGHT;
                                        break;
                                    case humoto::walking::StanceType::RSS:
                                        landing_foot = humoto::LeftOrRight::LEFT;
                                        break;
                                    default:
                                        HUMOTO_THROW_MSG("Unknown state type.");
                                        break;
                                }
                                support_foot = humoto::LeftOrRight::invert(landing_foot);

                                getFootLandingState(model_state.feet_[landing_foot], model, landing_foot);
                                model_state.feet_[support_foot] = model.getFootState(support_foot);
                            }
                            else
                            {
                                model_state.feet_.setLeft(model.getFootState(LeftOrRight::LEFT));
                                model_state.feet_.setRight(model.getFootState(LeftOrRight::RIGHT));
                            }
                            break;

                        case humoto::walking::StanceType::DS:
                            model_state.feet_.setLeft(model.getFootState(LeftOrRight::LEFT));
                            model_state.feet_.setRight(model.getFootState(LeftOrRight::RIGHT));
                            break;

                        default:
                            HUMOTO_THROW_MSG("Unknown stance type.");
                            break;
                    }


                    return(model_state);
                }



                /**
                 * @brief Determine landing state of a foot
                 *
                 * @param[out] foot_state    state of the landing foot
                 * @param[in] model         model
                 * @param[in] landing_foot  left or right
                 */
                void getFootLandingState(   humoto::rigidbody::RigidBodyState &foot_state,
                                            const humoto::wpg04::Model &model,
                                            const humoto::LeftOrRight::Type landing_foot) const
                {
                    std::size_t interval_index = preview_horizon_.variable_steps_indices_[0];
                    const WalkState & walk_state = preview_horizon_.getWalkState(interval_index);


                    foot_state.setDefaults();
                    etools::Vector2 position_xy;

                    humoto::LeftOrRight::Type     support_foot = LeftOrRight::invert(landing_foot);


                    switch (walk_state.type_) // next stance
                    {
                        case humoto::walking::StanceType::RSS:
                        case humoto::walking::StanceType::LSS:
                            position_xy = footpos_profile.segment(0, 2);
                            break;

                        case humoto::walking::StanceType::DS:
                            if ((humoto::walking::StanceType::RSS == model.state_.stance_type_) ||
                                (humoto::walking::StanceType::LSS == model.state_.stance_type_)  )
                            {
                                position_xy = model.getFootPositionFromADS( landing_foot,
                                                                            walk_state.rotation_,
                                                                            footpos_profile.segment(0, 2));
                            }
                            else
                            {
                                HUMOTO_THROW_MSG("Double support cannot be followed by another double support.");
                            }
                            break;

                        default:
                            HUMOTO_THROW_MSG("Transitional double support cannot have variable position.");
                            break;
                    }
                    double position_z = model.getFootState(support_foot).position_.z();
                    foot_state.position_ << position_xy, position_z;

                    foot_state.rpy_.setZero();
                    foot_state.rpy_[humoto::AngleIndex::YAW] = walk_state.theta_;
                }



            public:
                humoto::wpg04::PreviewHorizon   preview_horizon_;


                etools::DiagonalBlockMatrix<2,2>     R_;
                etools::DiagonalBlockMatrix<2,2>     Rh_;

                Eigen::MatrixXd                 Ir_;
                Eigen::MatrixXd                 V_;
                Eigen::MatrixXd                 V0_;
                Eigen::MatrixXd                 Vfp_;
                Eigen::MatrixXd                 vfp_;

                Eigen::MatrixXd                 S_;
                Eigen::MatrixXd                 s_;

                etools::SelectionMatrix         velocity_selector_;

                Eigen::MatrixXd                 Sdz_;
                Eigen::MatrixXd                 sdz_;


                Eigen::VectorXd                 cop_profile;
                Eigen::VectorXd                 dcop_profile;
                Eigen::VectorXd                 cstate_profile;
                Eigen::VectorXd                 footpos_profile;


            public:
                /**
                 * @brief Constructor
                 */
                MPCforWPG() : velocity_selector_(3,1)
                {
                    solution_is_parsed_ = false;
                }


                /**
                 * @brief Constructor
                 *
                 * @param[in] mpc_parameters parameters of the MPC
                 */
                explicit MPCforWPG(const humoto::wpg04::MPCParameters &mpc_parameters)
                    : velocity_selector_(3,1)
                {
                    solution_is_parsed_ = false;
                    setParameters(mpc_parameters);
                }


                /**
                 * @brief Set parameters
                 *
                 * @param[in] mpc_parameters
                 */
                void setParameters (const humoto::wpg04::MPCParameters  &mpc_parameters)
                {
                    mpc_parameters_ = mpc_parameters;
                }


                /**
                 * @brief Update control problem
                 *
                 * @param[in] model     model of the system
                 * @param[in] stance_fsm  walking finite state machine
                 * @param[in] walk_parameters
                 *
                 * @return ControlProblemStatus::OK/ControlProblemStatus::STOPPED
                 */
                ControlProblemStatus::Status
                    update( const humoto::wpg04::Model                      &model,
                            const humoto::walking::StanceFiniteStateMachine &stance_fsm,
                            const humoto::wpg04::WalkParameters             &walk_parameters)
                {
                    humoto::ControlProblemStatus::Status    control_status = ControlProblemStatus::OK;
                    solution_is_parsed_ = false;

                    if (preview_horizon_.form(  mpc_parameters_,
                                                model,
                                                stance_fsm,
                                                walk_parameters) )
                    {
                        // decision variables
                        sol_structure_.reset();
                        sol_structure_.addSolutionPart(COP_VARIABLES_ID, mpc_parameters_.preview_horizon_length_ * 2);
                        sol_structure_.addSolutionPart(FOOTPOS_VARIABLES_ID, preview_horizon_.getNumberOfVariableSteps() * 2);

                        std::size_t number_of_state_variables = model.Ns_ * mpc_parameters_.preview_horizon_length_;


                        // Initialize matrices
                        formRotationMatrices();
                        formFootPosMatrices(model);


                        // condensing
                        std::vector<etools::Matrix3>    A_matrices;
                        std::vector<etools::Vector3>    B_matrices;

                        std::vector<etools::Matrix1x3>  D_matrices;
                        std::vector<double>              E_matrices;

                        A_matrices.resize(mpc_parameters_.preview_horizon_length_);
                        B_matrices.resize(mpc_parameters_.preview_horizon_length_);
                        D_matrices.resize(mpc_parameters_.preview_horizon_length_);
                        E_matrices.resize(mpc_parameters_.preview_horizon_length_);


                        for (std::size_t i = 0; i < mpc_parameters_.preview_horizon_length_; ++i)
                        {
                            double T = preview_horizon_.intervals_[i].T_;
                            double omega = preview_horizon_.intervals_[i].omega_;

                            A_matrices[i] = model.getA3(T, omega);
                            B_matrices[i] = model.getB3(T, omega);

                            D_matrices[i] = model.getD3(T, omega);
                            E_matrices[i] = model.getE3(T);
                        }

                        etools::GenericBlockMatrix<3,3> Ux;
                        etools::LeftLowerTriangularBlockMatrix<3,1> Uu;
//                        etools::GenericBlockMatrix<3,1> Uu;
                        condense(Ux, Uu, A_matrices, B_matrices);

                        etools::GenericBlockMatrix<1,3> Ox;
                        etools::LeftLowerTriangularBlockMatrix<1,1> Ou;
                        condenseOutput(Ox, Ou, D_matrices, E_matrices, Ux, Uu);


                        etools::GenericBlockMatrix<3,1> UuIr(Uu*Ir_);
                        etools::GenericBlockMatrix<1,1> OuIr(Ou*Ir_);

                        etools::GenericBlockMatrix<3,1> Uu1n(Uu.getRaw().rowwise().sum());
                        etools::GenericBlockMatrix<1,1> Ou1n(Ou.getRaw().rowwise().sum());

                        // -----

                        S_.resize (number_of_state_variables, sol_structure_.getNumberOfVariables ());
                        S_ <<   Uu.getBlockKroneckerProduct(2) * Rh_,
                                UuIr.getBlockKroneckerProduct(2) * R_;

                        s_.noalias() = Uu1n.getBlockKroneckerProduct(2) * model.current_support_position_
                                        +  Ux.getBlockKroneckerProduct(2) * model.getCState();

                        // -----

                        Sdz_.resize (2*mpc_parameters_.preview_horizon_length_, sol_structure_.getNumberOfVariables ());
                        Sdz_ << Ou.getBlockKroneckerProduct(2) * Rh_,
                                OuIr.getBlockKroneckerProduct(2) * R_;
                        sdz_.noalias() = Ou1n.getBlockKroneckerProduct(2) * model.current_support_position_
                                            +  Ox.getBlockKroneckerProduct(2) * model.getCState();

                        control_status = ControlProblemStatus::OK;
                    }
                    else
                    {
                        control_status = ControlProblemStatus::STOPPED;
                    }

                    return (control_status);
                }


                /**
                 * @brief Process solution
                 *
                 * @param[in] solution  solution
                 */
                void    parseSolution(const humoto::Solution &solution)
                {
                    Eigen::VectorXd mpc_variables;
                    Eigen::VectorXd cop_local;
                    Eigen::VectorXd footpos_local;

                    cop_local = solution.getSolutionPart(COP_VARIABLES_ID);
                    footpos_local = solution.getSolutionPart(FOOTPOS_VARIABLES_ID);

                    mpc_variables.resize(cop_local.rows() + footpos_local.rows());
                    mpc_variables << cop_local, footpos_local;

                    cop_profile.noalias() = Rh_ * cop_local + V_ * footpos_local + V0_;
                    dcop_profile.noalias() = Sdz_ * mpc_variables + sdz_;
                    cstate_profile.noalias() = S_ * mpc_variables + s_;

                    footpos_profile.noalias() = Vfp_ * footpos_local + vfp_;

                    solution_is_parsed_ = true;
                }


                /**
                 * @brief Get next model state.
                 *
                 * @param[in] solution  solution
                 * @param[in] stance_fsm  walking finite state machine (must be updated before)
                 * @param[in] model model
                 *
                 * @return next model state.
                 */
                humoto::wpg04::ModelState   getNextModelState(
                        const humoto::Solution                          &solution,
                        const humoto::walking::StanceFiniteStateMachine &stance_fsm,
                        const humoto::wpg04::Model                      &model)
                {
                    HUMOTO_ASSERT(solution_is_parsed_ == false, "The solution is parsed.");
                    parseSolution(solution);
                    return(initializeNextModelState(stance_fsm, model));
                }


                /**
                 * @brief Get next model state.
                 *
                 * @param[in] stance_fsm  walking finite state machine (must be updated before)
                 * @param[in] model model
                 *
                 * @return next model state.
                 */
                humoto::wpg04::ModelState   getNextModelState(
                        const humoto::walking::StanceFiniteStateMachine &stance_fsm,
                        const humoto::wpg04::Model                      &model) const
                {
                    HUMOTO_ASSERT(solution_is_parsed_ == true, "This function can be called only after the solution is parsed.");
                    return(initializeNextModelState(stance_fsm, model));
                }



                /**
                 * @brief Get landing state of a foot.
                 *
                 * @param[in] model model
                 *
                 * @return state of the foot
                 */
                humoto::rigidbody::RigidBodyState getFootLandingState (const humoto::wpg04::Model &model) const
                {
                    HUMOTO_ASSERT(solution_is_parsed_ == true, "This function can be called only after the solution is parsed.");
                    HUMOTO_ASSERT(  (humoto::walking::StanceType::RSS == model.state_.stance_type_) ||
                                    (humoto::walking::StanceType::LSS == model.state_.stance_type_),
                                  "This function can be executed in a single support only.");

                    humoto::rigidbody::RigidBodyState  state;

                    switch (model.state_.stance_type_)
                    {
                        case humoto::walking::StanceType::LSS:
                            getFootLandingState(state, model, humoto::LeftOrRight::RIGHT);
                            break;

                        case humoto::walking::StanceType::RSS:
                            getFootLandingState(state, model, humoto::LeftOrRight::LEFT);
                            break;

                        default:
                            HUMOTO_THROW_MSG("Unknown state type.");
                            break;
                    }

                    return(state);
                }



                /**
                 * @brief Guess solution
                 *
                 * @param[out] solution_guess solution guess
                 * @param[in] old_solution old solution
                 */
                void guessSolution( Solution       &solution_guess,
                                    const Solution &old_solution) const
                {
                    solution_guess.initialize(sol_structure_, 0.0);


                    if ( (old_solution.isNonEmpty()) && (old_solution.getNumberOfVariables() != 0) )
                    {
                        Location old_part = old_solution.getSolutionPartLocation(COP_VARIABLES_ID);
                        Location guess_part = solution_guess.getSolutionPartLocation(COP_VARIABLES_ID);

                        guess_part.length_ -= 2;

                        old_part.offset_ +=2;
                        old_part.length_ -=2;

                        solution_guess.getData(guess_part) = old_solution.getData(old_part);



                        old_part = old_solution.getSolutionPartLocation(FOOTPOS_VARIABLES_ID);
                        guess_part = solution_guess.getSolutionPartLocation(FOOTPOS_VARIABLES_ID);

                        if (old_part.length_ == guess_part.length_)
                        {
                            solution_guess.getData(guess_part) = old_solution.getData(old_part);
                        }
                        else
                        {
                            if (old_part.length_ > guess_part.length_)
                            {
                                old_part.offset_ +=2;
                                old_part.length_ -=2;
                                solution_guess.getData(guess_part) = old_solution.getData(old_part);
                            }
                            else
                            {
                                guess_part.length_ -= 2;
                                solution_guess.getData(guess_part) = old_solution.getData(old_part);
                            }
                        }
                    }
                }


                /**
                 * @brief Computes CoM state at the given time instant
                 *
                 * @param[in] model model
                 * @param[in] time_instant_ms time instant
                 *
                 * @return CoM state.
                 */
                humoto::rigidbody::PointMassState getCoMState(const humoto::wpg04::Model &model,
                                                              const std::size_t time_instant_ms)
                {
                    HUMOTO_ASSERT(solution_is_parsed_ == true, "This function can be called only after the solution is parsed.");
                    std::size_t interval_index;
                    std::size_t time_offset_ms = time_instant_ms;
                    std::size_t interval_duration_ms;
                    double com_height;

                    for (interval_index = 0; interval_index < preview_horizon_.intervals_.size(); ++interval_index)
                    {
                        interval_duration_ms = preview_horizon_.intervals_[interval_index].T_ms_;
                        if (time_offset_ms > interval_duration_ms)
                        {
                            time_offset_ms -= interval_duration_ms;
                        }
                        else
                        {
                            com_height = preview_horizon_.intervals_[interval_index].com_height_;
                            break;
                        }
                    }

                    HUMOTO_ASSERT((interval_index < preview_horizon_.intervals_.size()),
                                "Given time instant is not included in the preview horizon.");


                    etools::Vector6    preceding_cstate;
                    etools::Vector2    control;


                    if (interval_index == 0)
                    {
                        preceding_cstate = model.getCState();
                    }
                    else
                    {
                        preceding_cstate = cstate_profile.segment((interval_index - 1)*model.Ns_, model.Ns_);
                    }
                    control = cop_profile.segment(interval_index*2, 2);

                    humoto::rigidbody::PointMassState com_state;

                    com_state = model.evaluate(convertMillisecondToSecond(time_offset_ms),
                                               convertMillisecondToSecond(interval_duration_ms),
                                               com_height,
                                               preceding_cstate,
                                               control);

                    return (com_state);
                }


                /**
                 * @brief Returns length of the preview horizon
                 *
                 * @return length of preview horizon
                 */
                std::size_t getPreviewHorizonLength() const
                {
                    return (mpc_parameters_.preview_horizon_length_);
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
                            const std::string &name = "mpcwpg") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    preview_horizon_.log(logger, subname);


                    logger.log(LogEntryName(subname).add("R") , R_.getRaw());
                    logger.log(LogEntryName(subname).add("Rh"), Rh_.getRaw());


                    logger.log(LogEntryName(subname).add("V")  , V_);
                    logger.log(LogEntryName(subname).add("V0") , V0_);
                    logger.log(LogEntryName(subname).add("Vfp"), Vfp_);
                    logger.log(LogEntryName(subname).add("vfp"), vfp_);


                    logger.log(LogEntryName(subname).add("S")  , S_);
                    logger.log(LogEntryName(subname).add("s")  , s_);
                    logger.log(LogEntryName(subname).add("Sdz"), Sdz_);
                    logger.log(LogEntryName(subname).add("sdz"), sdz_);


                    logger.log(LogEntryName(subname).add("cop_profile")    , cop_profile);
                    logger.log(LogEntryName(subname).add("dcop_profile")   , dcop_profile);
                    logger.log(LogEntryName(subname).add("cstate_profile") , cstate_profile);
                    logger.log(LogEntryName(subname).add("footpos_profile"), footpos_profile);
                }
        };
    }
}

