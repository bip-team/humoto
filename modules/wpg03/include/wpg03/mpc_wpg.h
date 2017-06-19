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
         * @brief Possible types of MPC for the collaborative carrying task
         */
        enum MPCInteractionType
        {
            /// The normal case
            MPC_NORMAL = 0,

            /// The robot takes the lead
            MPC_LEADER = 1,

            /// The robot follows a communicated intent
            MPC_FOLLOWER = 2
        };


        /**
         * @brief Model Predictive Control problem for walking pattern generation [determine_solution_structure.m,
         *  form_rotation_matrices.m, form_foot_pos_matrices.m, form_condensing_matrices.m]
         */
        class MPCforWPG : public humoto::MPC
        {
            private:
                /**
                 * @brief Create the rotation matrices of the MPC problem
                 */
                void formRotationMatrices()
                {
                    std::vector<Eigen::MatrixXd> pwin_rotation_matrices;

                    for(unsigned int i=0; i < (preview_horizon_.states_.size() - 1); ++i)
                    {
                        pwin_rotation_matrices.push_back(preview_horizon_.states_.at(i).rotation_);
                    }
                    R_ = etools::makeBlockDiagonal(pwin_rotation_matrices);

                    pwin_rotation_matrices.clear();
                    for(unsigned int i=0; i < preview_horizon_.intervals_.size(); ++i)
                    {
                        pwin_rotation_matrices.push_back(   preview_horizon_.states_.at(
                                                                preview_horizon_.intervals_.at(i).
                                                                state_index_).rotation_);
                    }
                    Rh_ = etools::makeBlockDiagonal(pwin_rotation_matrices);
                }


                /**
                 * @brief Create the foot position matrices
                 *
                 * @param[in] model
                 */
                void formFootPosMatrices(const humoto::wpg03::Model &model)
                {
                    // generate_fixed_step_vector
                    Eigen::MatrixXd Vc = etools::Matrix2::Identity().replicate(preview_horizon_.intervals_.size(), 1);
                    V0c_ =  Vc * model.current_support_position_;

                    // generate_var_step_matrix
                    V_ = Eigen::MatrixXd::Zero( preview_horizon_.intervals_.size()*2,
                                                preview_horizon_.getNumberOfVariableSteps()*2);

                    for(unsigned int i=0; i < preview_horizon_.variable_steps_indices_.size(); ++i)
                    {
                        unsigned int row_size = preview_horizon_.intervals_.size() - preview_horizon_.variable_steps_indices_[i];
                        V_.block(preview_horizon_.variable_steps_indices_[i]*2 , i*2, row_size*2, 2) =
                            etools::Matrix2::Identity().replicate(row_size, 1);
                    }

                    V_ = V_*R_;

                    Vfp_ = etools::Matrix2::Identity().replicate(
                            preview_horizon_.getNumberOfVariableSteps(),
                            preview_horizon_.getNumberOfVariableSteps()).triangularView<Eigen::Lower>() * R_;

                    vfp_ = model.current_support_position_.replicate(preview_horizon_.getNumberOfVariableSteps(), 1);
                }


                /**
                 * @brief Process solution
                 *
                 * @param[in] solution  solution
                 * @param[in] mpc_type
                 */
                void    processSolution(const humoto::Solution &solution,
                                        const humoto::wpg03::MPCInteractionType mpc_type = MPC_NORMAL)
                {
                    Eigen::VectorXd mpc_variables;
                    Eigen::VectorXd cop_local;
                    Eigen::VectorXd footpos_local;

                    comjerk_profile = solution.getSolutionPart(COMJERK_VARIABLES_ID);
                    footpos_local = solution.getSolutionPart(FOOTPOS_VARIABLES_ID);

                    switch(mpc_type)
                    {
                        case MPC_NORMAL:
                        case MPC_FOLLOWER:
                        {
                            mpc_variables.resize(comjerk_profile.rows() + footpos_local.rows());
                            mpc_variables << comjerk_profile, footpos_local;
                            break;
                        }
                        case MPC_LEADER:
                        {
                            extwrench_profile = solution.getSolutionPart(EXTWRENCH_VARIABLES_ID);
                            mpc_variables.resize(comjerk_profile.rows() + footpos_local.rows() + extwrench_profile.rows());
                            mpc_variables << comjerk_profile, footpos_local, extwrench_profile;
                            break;
                        }
                        default:
                        {
                            HUMOTO_THROW_MSG("unexpected MPC type");
                            break;
                        }
                    }

                    cop_local = Sz_* mpc_variables + sz_;

                    cop_profile = Rh_ * cop_local + V_ * footpos_local + V0c_;
                    cstate_profile = S_ * mpc_variables + s_;

                    footpos_profile = Vfp_ * footpos_local + vfp_;
                }

            public:
                humoto::wpg03::MPCParameters    mpc_parameters_;
                humoto::wpg03::PreviewHorizon   preview_horizon_;


                Eigen::MatrixXd                 R_;
                Eigen::MatrixXd                 Rh_;

                Eigen::MatrixXd                 V_;
                Eigen::MatrixXd                 V0c_;
                Eigen::MatrixXd                 Vfp_;
                Eigen::MatrixXd                 vfp_;

                Eigen::MatrixXd                 S_;
                Eigen::MatrixXd                 s_;

                Eigen::MatrixXd                 Sv_;
                Eigen::MatrixXd                 sv_;

                Eigen::MatrixXd                 Sz_;
                Eigen::MatrixXd                 sz_;

                Eigen::MatrixXd                 F_;

                Eigen::VectorXd                 cop_profile;
                Eigen::VectorXd                 comjerk_profile;
                Eigen::VectorXd                 cstate_profile;
                Eigen::VectorXd                 footpos_profile;
                Eigen::VectorXd                 extwrench_profile;

                Eigen::MatrixXd                 Gmbk_;
                Eigen::VectorXd                 Select_Fxy_;
                Eigen::VectorXd                 Cref_;
                Eigen::Matrix<double, 2, 4>     selectxy_mat;
                etools::Vector4                 wrench_bounds;

                /**
                 * @brief Constructor
                 */
                MPCforWPG()
                {
                    mpc_parameters_.finalize();
                }


                /**
                 * @brief Constructor
                 *
                 * @param[in] mpc_parameters parameters of the MPC
                 */
                MPCforWPG(const humoto::wpg03::MPCParameters  &mpc_parameters)
                {
                    setParameters(mpc_parameters);
                }


                void setParameters (const humoto::wpg03::MPCParameters  &mpc_parameters)
                {
                    mpc_parameters_ = mpc_parameters;

                    mpc_parameters_.finalize();
                }


                /**
                 * @brief Update control problem
                 *
                 * @param[in] model              model of the system
                 * @param[in] walk_fsm           walking finite state machine
                 * @param[in] mpc_type           interaction behavior type
                 * @param[in] external_wrenches  6N vector of future forces and torques acting on the Center of Mass frame for N preview steps
                 *
                 * @return ControlProblemStatus::OK/ControlProblemStatus::STOPPED
                 */
                ControlProblemStatus::Status
                    update( const humoto::wpg03::Model &model,
                            const humoto::wpg03::WalkFiniteStateMachine &walk_fsm,
                            const Eigen::VectorXd &external_wrenches,
                            const humoto::wpg03::MPCInteractionType mpc_type = MPC_NORMAL)
                {
                    humoto::ControlProblemStatus::Status    control_status = ControlProblemStatus::OK;


                    if (preview_horizon_.form(  mpc_parameters_,
                                                model,
                                                walk_fsm) )
                    {
                        // decision variables
                        sol_structure_.reset();
                        sol_structure_.addSolutionPart(COMJERK_VARIABLES_ID, mpc_parameters_.preview_horizon_length_ * 2);
                        sol_structure_.addSolutionPart(FOOTPOS_VARIABLES_ID, preview_horizon_.getNumberOfVariableSteps() * 2);
                        switch(mpc_type)
                        {
                            case MPC_NORMAL:
                            case MPC_FOLLOWER:
                            {
                                break;
                            }
                            case MPC_LEADER:
                            {
                                sol_structure_.addSolutionPart(EXTWRENCH_VARIABLES_ID, mpc_parameters_.preview_horizon_length_ * 4);
                                break;
                            }
                            default:
                            {
                                HUMOTO_THROW_MSG("unexpected MPC type");
                                break;
                            }
                        }


                        unsigned int number_of_state_variables = model.Ns_ * mpc_parameters_.preview_horizon_length_;


                        // Initialize matrices
                        formRotationMatrices();
                        formFootPosMatrices(model);


                        // condensing
                        std::vector<Eigen::MatrixXd>    A_matrices;
                        std::vector<Eigen::MatrixXd>    B_matrices;
                        std::vector<Eigen::MatrixXd>    D_matrices;
                        std::vector<Eigen::MatrixXd>    F_matrices;

                        Eigen::VectorXd cvel_ref;

                        A_matrices.resize(mpc_parameters_.preview_horizon_length_);
                        B_matrices.resize(mpc_parameters_.preview_horizon_length_);
                        D_matrices.resize(mpc_parameters_.preview_horizon_length_);
                        F_matrices.resize(mpc_parameters_.preview_horizon_length_);

                        cvel_ref.resize(mpc_parameters_.preview_horizon_length_*2);

                        Eigen::VectorXd f_pred;
                        switch(mpc_type)
                        {
                            case MPC_NORMAL:
                            {
                                // process the external wrenches to get the ordering of needed terms
                                f_pred = getReorderedSet(external_wrenches);
                                break;
                            }
                            case MPC_FOLLOWER:
                            {
                                // process the external wrenches to get the ordering of needed terms
                                f_pred = getReorderedSet(external_wrenches);
                                Select_Fxy_ = etools::makeBlockDiagonal(selectxy_mat, mpc_parameters_.preview_horizon_length_) * f_pred;
                                break;
                            }
                            case MPC_LEADER:
                            {
                                break;
                            }
                            default:
                            {
                                HUMOTO_THROW_MSG("unexpected MPC type");
                                break;
                            }
                        }
                        for (unsigned int i = 0; i < mpc_parameters_.preview_horizon_length_; ++i)
                        {
                            double T = preview_horizon_.intervals_[i].T_;
                            double omega = preview_horizon_.intervals_[i].omega_;
                            double f_ext_z = external_wrenches(i*6+2);

                            A_matrices[i] = model.getA6(T);
                            B_matrices[i] = model.getB6(T);

                            // Note: in the math, these matrices start at preview step 1 and end at N
                            //       it assumes that the given external_wrenches are the same with other variables constant over N
                            D_matrices[i] = model.getD6(omega, model.getMass(), f_ext_z);
                            F_matrices[i] = model.getF4(model.getComHeight(), model.getMass(), f_ext_z);

                            cvel_ref.segment(i*2, 2) = preview_horizon_.intervals_[i].cvel_ref_;
                        }

                        Eigen::MatrixXd Ux;
                        Eigen::MatrixXd Uu;

                        condense(Ux, Uu, A_matrices, B_matrices);

                        S_.resize (number_of_state_variables, sol_structure_.getNumberOfVariables ());
                        S_.setZero();
                        S_.topLeftCorner(Uu.rows(), Uu.cols()) = Uu;


                        s_ = Ux*model.cstate_;

                        Sv_ = etools::selectRows(S_, 3, 1);
                        sv_ = etools::selectRows(s_, 3, 1) - cvel_ref;

                        Eigen::MatrixXd Ox;
                        Ox.resize(2*mpc_parameters_.preview_horizon_length_, model.Ns_);
                        Ox = etools::makeBlockDiagonal(D_matrices) * Ux;

                        Eigen::MatrixXd Ou;
                        Ou.setZero(2*mpc_parameters_.preview_horizon_length_, model.Nu_*mpc_parameters_.preview_horizon_length_);
                        Ou = etools::makeBlockDiagonal(D_matrices) * Uu;

                        Sz_.resize(2*mpc_parameters_.preview_horizon_length_, sol_structure_.getNumberOfVariables ());

                        switch(mpc_type)
                        {
                            case MPC_NORMAL:
                            case MPC_FOLLOWER:
                            {
                                F_ = etools::makeBlockDiagonal(F_matrices) * f_pred;

                                Sz_ << Ou, -V_;
                                sz_ = Ox*model.cstate_ + F_ - V0c_;

                                break;
                            }
                            case MPC_LEADER:
                            {
                                Eigen::MatrixXd Of; // TODO: refactor to better mimic math notation
                                Of = etools::makeBlockDiagonal(F_matrices);

                                Sz_ << Ou, -V_, Of;
                                sz_ = Ox*model.cstate_ - V0c_;
                                break;
                            }
                            default:
                            {
                                HUMOTO_THROW_MSG("unexpected MPC type");
                                break;
                            }
                        }
                        Sz_ = Rh_.transpose()*Sz_;
                        sz_ = Rh_.transpose()*sz_;

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
                 * @param[in] walk_fsm  walking finite state machine (must be updated before)
                 * @param[in] model model
                 * @param[in] mpc_type
                 *
                 * @return next model state.
                 */
                humoto::wpg03::ModelState   getNextModelState(
                        const humoto::Solution &solution,
                        const humoto::wpg03::WalkFiniteStateMachine &walk_fsm,
                        const humoto::wpg03::Model &model,
                        const humoto::wpg03::MPCInteractionType mpc_type=MPC_NORMAL)
                {
                    processSolution(solution, mpc_type);

                    humoto::wpg03::ModelState   model_state;

                    double T = preview_horizon_.intervals_[0].T_;

                    Eigen::MatrixXd A = model.getA6(T);
                    Eigen::MatrixXd B = model.getB6(T);


                    model_state.cstate_ = A*model.cstate_ + B * comjerk_profile.segment(0, model.Nu_);


                    model_state.state_type_         = walk_fsm.getSupportStateType(0);
                    model_state.next_state_type_    = walk_fsm.getSupportStateType(1);


                    if (walk_fsm.is_end_of_ss_) // it happens only in TDS
                    {
                        model_state.next_support_set_ = true;
                        model_state.next_support_position_ = footpos_profile.segment(0,2);
                        model_state.next_support_rotation_ = walk_fsm.states_[walk_fsm.index_ + 1].rotation_;
                    }

                    return(model_state);
                }



                /**
                 * @brief Set ext wrench bounds when in MPC_LEADER mode
                 * @param force_x       x component of the force in CoM frame
                 * @param force_y       y component of the force in CoM frame
                 * @param torque_x      x component of the torque in CoM frame
                 * @param torque_y      y component of the torque in CoM frame
                 */
                void setWrenchBounds(const double force_x, const double force_y,
                                     const double torque_x, const double torque_y)
                {
                    wrench_bounds << torque_y, force_x, torque_x, force_y;
                }

                /**
                 * @brief Set tracking gains individually
                 *
                 * @param[in] gain_x_m    feed-forward acceleration gain (inertia) in x
                 * @param[in] gain_x_b    velocity gain (damping) in x
                 * @param[in] gain_x_k    position gain (stiffness) in x
                 * @param[in] gain_y_m    feed-forward acceleration gain (inertia) in y
                 * @param[in] gain_y_b    velocity gain (damping) in y
                 * @param[in] gain_y_k    position gain (stiffness) in y
                 */
                void setTrackingGains(const double gain_x_m, const double gain_x_b, const double gain_x_k,
                                      const double gain_y_m, const double gain_y_b, const double gain_y_k)
                {
                    Eigen::Matrix<double, 2, 6> gains;
                    // This rescales the impedance gains by normalizing on the velocity
                    // It allows the weighting of task-comimpedance to be similar to task-comvelocity
                    // and handles zero damping cases (not usual) without rescaling
                    if((gain_x_b == 0.) && (gain_y_b == 0.))
                    {
                        gains << gain_x_k, 0., gain_x_m, 0., 0., 0.,
                                 0., 0., 0., gain_y_k, 0., gain_y_m;
                        selectxy_mat << 0., 1., 0., 0,
                                        0., 0., 0., 1.;
                    }
                    else if((gain_x_b == 0.) && (gain_y_b != 0.))
                    {
                        gains << gain_x_k, 0., gain_x_m, 0., 0., 0.,
                                 0., 0., 0., gain_y_k/gain_y_b, 1., gain_y_m/gain_y_b;
                        selectxy_mat << 0., 1., 0., 0,
                                        0., 0., 0., 1./gain_y_b;
                    }
                    else if((gain_x_b != 0.) && (gain_y_b == 0.))
                    {
                        gains << gain_x_k/gain_x_b, 1., gain_x_m/gain_x_b, 0., 0., 0.,
                                 0., 0., 0., gain_y_k, 0., gain_y_m;
                        selectxy_mat << 0., 1./gain_x_b, 0., 0,
                                        0., 0., 0., 1.;
                    }
                    else
                    {
                        gains << gain_x_k/gain_x_b, 1., gain_x_m/gain_x_b, 0., 0., 0.,
                                 0., 0., 0., gain_y_k/gain_y_b, 1., gain_y_m/gain_y_b;
                        selectxy_mat << 0., 1./gain_x_b, 0., 0,
                                        0., 0., 0., 1./gain_y_b;
                    }
                    Gmbk_ = etools::makeBlockDiagonal(gains, (mpc_parameters_.preview_horizon_length_));
                }


                /**
                 * @brief Set tracking gains equally for x and y
                 *
                 * @param[in] gain_m    feed-forward acceleration gain (inertia)
                 * @param[in] gain_b    velocity gain (damping)
                 * @param[in] gain_k    position gain (stiffness)
                 */
                void setTrackingGains(const double gain_m, const double gain_b, const double gain_k)
                {
                    setTrackingGains(gain_m, gain_b, gain_k, gain_m, gain_b, gain_k);
                }


                /**
                 * @brief Set tracking gains individually based on a
                 *        critically damped system with unit inertia
                 *
                 * @param[in] gain_x_k    position gain (stiffness) in x
                 * @param[in] gain_y_k    position gain (stiffness) in y
                 */
                void setTrackingGains(const double gain_x_k, const double gain_y_k)
                {
                    setTrackingGains(1., 2.*std::sqrt(gain_x_k), gain_x_k,
                                     1., 2.*std::sqrt(gain_y_k), gain_y_k);
                }


                /**
                 * @brief Set tracking gains equally for x and y based on a
                 *        critically damped system with unit inertia
                 *
                 * @param[in] gain_k    position gain (stiffness)
                 */
                void setTrackingGains(const double gain_k)
                {
                    setTrackingGains(gain_k, gain_k);
                }


                /**
                 * @brief Set the trajectory reference
                 *
                 * @param[in] com_traj_ref    2D reference CoM trajectory - 6N vector where N is the preview horizon length
                 */
                void setTrajectory(const Eigen::VectorXd &com_traj_ref)
                {
                    Cref_ = com_traj_ref;
                }


                /**
                 * @brief Log
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger & logger = HUMOTO_GLOBAL_LOGGER,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string & name = "mpcwpg") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    preview_horizon_.log(logger, subname);


                    logger.log(LogEntryName(subname).add("R"), R_);
                    logger.log(LogEntryName(subname).add("Rh"), Rh_);


                    logger.log(LogEntryName(subname).add("V"), V_);
                    logger.log(LogEntryName(subname).add("V0c"), V0c_);
                    logger.log(LogEntryName(subname).add("Vfp"), Vfp_);
                    logger.log(LogEntryName(subname).add("vfp"), vfp_);

                    logger.log(LogEntryName(subname).add("S"), S_);
                    logger.log(LogEntryName(subname).add("s"), s_);
                    logger.log(LogEntryName(subname).add("Sv"), Sv_);
                    logger.log(LogEntryName(subname).add("sv"), sv_);


                    logger.log(LogEntryName(subname).add("Sz"), Sz_);
                    logger.log(LogEntryName(subname).add("sz"), sz_);
                    logger.log(LogEntryName(subname).add("F"), F_);


                    logger.log(LogEntryName(subname).add("cop_profile"), cop_profile);
                    logger.log(LogEntryName(subname).add("comjerk_profile"), comjerk_profile);
                    logger.log(LogEntryName(subname).add("cstate_profile"), cstate_profile);
                    logger.log(LogEntryName(subname).add("footpos_profile"), footpos_profile);
                    logger.log(LogEntryName(subname).add("extwrench_profile"), extwrench_profile);
                }
        };
    }
}
