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
         * @brief Model Predictive Control problem for walking pattern generation [determine_solution_structure.m,
         *  form_rotation_matrices.m, form_foot_pos_matrices.m, form_condensing_matrices.m]
         */
        class HUMOTO_LOCAL MPCforMG : public humoto::MPC
        {
            private:
                bool solution_is_parsed_;


            private:
                /**
                 * @brief Create the rotation matrices of the MPC problem
                 */
                void formRotationMatrices()
                {
                    R_.setZero(preview_horizon_.intervals_.size());

                    for(std::size_t i=0; i < preview_horizon_.intervals_.size(); ++i)
                    {
                        R_(i) = preview_horizon_.getRotationMatrix(i).transpose();
                    }
                }



                /**
                 * @brief Initialize next model state
                 *
                 * @param[in] model model
                 *
                 * @return next model state.
                 */
                humoto::pepper_mpc::ModelState   initializeNextModelState(
                        const humoto::pepper_mpc::Model                      &model) const
                {
                    humoto::pepper_mpc::ModelState model_state;


                    double T     = preview_horizon_.intervals_[0].T_;
                    double base_height = preview_horizon_.intervals_[0].base_height_;
                    double body_height = preview_horizon_.intervals_[0].body_height_;
                    double theta = preview_horizon_.intervals_[0].theta_;


                    model.getNextState( model_state,
                                        T,
                                        T,
                                        base_controls_.head(2),
                                        body_controls_.head(2),
                                        base_height,
                                        body_height,
                                        theta);


                    return(model_state);
                }


                /**
                 * @brief Condense and recompute all dependent matrices.
                 *
                 * @param[in] model
                 */
                void updateMatrices(const Model     &model)
                {
                    // decision variables
                    sol_structure_.reset();
                    sol_structure_.addSolutionPart(BASE_VEL_VARIABLES_ID, mpc_parameters_.preview_horizon_length_ * 2);
                    sol_structure_.addSolutionPart(BODY_JERK_VARIABLES_ID, mpc_parameters_.preview_horizon_length_ * 2);


                    // Initialize matrices
                    formRotationMatrices();



                    // condensing
                    std::vector< etools::Matrix3 >    As_matrices;
                    std::vector< etools::Vector3 >    Bs_matrices;

                    std::vector< etools::Matrix3 >    Ad_matrices;
                    std::vector< etools::Vector3 >    Bd_matrices;

                    std::vector< etools::Matrix1x3 >    Djs_matrices;
                    std::vector< double >                       Ejs_matrices;

                    etools::DiagonalBlockMatrix<1,3>    Dps;
                    etools::DiagonalBlockMatrix<1,3>    Dpd;


                    As_matrices.resize(mpc_parameters_.preview_horizon_length_);
                    Bs_matrices.resize(mpc_parameters_.preview_horizon_length_);

                    Ad_matrices.resize(mpc_parameters_.preview_horizon_length_);
                    Bd_matrices.resize(mpc_parameters_.preview_horizon_length_);

                    Djs_matrices.resize(mpc_parameters_.preview_horizon_length_);
                    Ejs_matrices.resize(mpc_parameters_.preview_horizon_length_);

                    Dps.setZero(mpc_parameters_.preview_horizon_length_);
                    Dpd.setZero(mpc_parameters_.preview_horizon_length_);


                    for (std::size_t i = 0; i < mpc_parameters_.preview_horizon_length_; ++i)
                    {
                        double T = preview_horizon_.intervals_[i].T_;
                        double base_height = preview_horizon_.intervals_[i].base_height_;
                        double body_height = preview_horizon_.intervals_[i].body_height_;
                        double base_mass = preview_horizon_.intervals_[i].base_mass_;
                        double body_mass = preview_horizon_.intervals_[i].body_mass_;


                        As_matrices[i] = model.getAs3(T);
                        Bs_matrices[i] = model.getBs3(T);

                        Ad_matrices[i] = model.getAd3(T);
                        Bd_matrices[i] = model.getBd3(T);

                        Djs_matrices[i] = model.getDjs3(T);
                        Ejs_matrices[i] = model.getEjs3(T);

                        Dps(i) = model.getDps3(base_height, base_mass, body_mass);
                        Dpd(i) = model.getDpd3(body_height, base_mass, body_mass);
                    }

                    etools::GenericBlockMatrix<3,3> Uxs;
                    etools::LeftLowerTriangularBlockMatrix<3,1> Uus;

                    etools::GenericBlockMatrix<3,3> Uxd;
                    etools::LeftLowerTriangularBlockMatrix<3,1> Uud;

                    condense(Uxs, Uus, As_matrices, Bs_matrices);
                    condense(Uxd, Uud, Ad_matrices, Bd_matrices);

                    etools::GenericBlockMatrix<1,3> Ojxs;
                    etools::LeftLowerTriangularBlockMatrix<1,1> Ojus;

                    condenseOutput(Ojxs, Ojus, Djs_matrices, Ejs_matrices, Uxs, Uus);


                    etools::Vector6     mpc_base_state = model.getMPCBaseState();
                    etools::Vector6     mpc_body_state = model.getMPCBodyState();


                    // -----

                    std::size_t number_of_state_variables = model.Ns_ * mpc_parameters_.preview_horizon_length_;

                    etools::BlockMatrixInterface<   etools::MatrixBlockSizeType::DYNAMIC,
                                                            etools::MatrixBlockSizeType::DYNAMIC,
                                                            etools::MatrixSparsityType::NONE> S_bmi( S_,
                                                                                        number_of_state_variables/2,
                                                                                        sol_structure_.getNumberOfVariables()/2);
                    S_bmi.resize(2, 2);

                    /*
                    // slower
                    S_bmi(0,0) = Uus.getBlockKroneckerProduct(2).evaluate();
                    S_bmi(0,1).setZero();
                    S_bmi(1,0).setZero();
                    S_bmi(1,1) = Uud.getBlockKroneckerProduct(2).evaluate();
                    */

                    Uus.getBlockKroneckerProduct(2).evaluate(S_bmi(0,0));
                    S_bmi(0,1).setZero();
                    S_bmi(1,0).setZero();
                    Uud.getBlockKroneckerProduct(2).evaluate(S_bmi(1,1));


                    s_.resize(number_of_state_variables);
                    s_ <<   Uxs.getBlockKroneckerProduct(2) * mpc_base_state,
                            Uxd.getBlockKroneckerProduct(2) * mpc_body_state;


                    // -----

                    sp_.noalias() = etools::GenericBlockKroneckerProduct<1,3>(Dps*Uxs, 2) * mpc_base_state
                                    +
                                    etools::GenericBlockKroneckerProduct<1,3>(Dpd*Uxd, 2) * mpc_body_state;

                    Ap_.resize(2*mpc_parameters_.preview_horizon_length_, sol_structure_.getNumberOfVariables ());
                    /*
                    // slower
                    Ap_ <<   etools::LeftLowerTriangularBlockKroneckerProduct<1,1>(Dps*Uus, 2).evaluate(),
                             etools::LeftLowerTriangularBlockKroneckerProduct<1,1>(Dpd*Uud, 2).evaluate();
                    */
                    etools::LeftLowerTriangularBlockKroneckerProduct<1,1>(Dps*Uus, 2).evaluate(Ap_.leftCols(sol_structure_.getNumberOfVariables()/2));
                    etools::LeftLowerTriangularBlockKroneckerProduct<1,1>(Dpd*Uud, 2).evaluate(Ap_.rightCols(sol_structure_.getNumberOfVariables()/2));


                    // -----

                    /*
                    // slower
                    sas_.noalias() = etools::GenericBlockKroneckerProduct<1,3>(Uxs.selectRowInBlocksAsMatrix(2), 2) * mpc_base_state;
                    Aas_.resize(2*mpc_parameters_.preview_horizon_length_, sol_structure_.getNumberOfVariables ());
                    Aas_ << etools::LeftLowerTriangularBlockKroneckerProduct<1,1>(Uus.selectRowInBlocksAsMatrix(2), 2).evaluate(),
                            Eigen::MatrixXd::Zero(2*mpc_parameters_.preview_horizon_length_, sol_structure_.getNumberOfVariables()/2);
                    //etools::LeftLowerTriangularBlockKroneckerProduct<1,1>(Uus.selectRowInBlocksAsMatrix(2), 2).evaluate(Aas_);
                    */
                    sas_.noalias() = Uxs.selectRowInBlocks(2).getBlockKroneckerProduct(2) * mpc_base_state;
                    Uus.selectRowInBlocks(2).getBlockKroneckerProduct(2).evaluate(Aas_);

                    // -----

                    /*
                    // slower
                    sps_.noalias() = etools::GenericBlockKroneckerProduct<1,3>(Uxs.selectRowInBlocksAsMatrix(0), 2) * mpc_base_state;
                    Aps_.resize(2*mpc_parameters_.preview_horizon_length_, sol_structure_.getNumberOfVariables ());
                    Aps_ << etools::LeftLowerTriangularBlockKroneckerProduct<1,1>(Uus.selectRowInBlocksAsMatrix(0), 2).evaluate(),
                            Eigen::MatrixXd::Zero(2*mpc_parameters_.preview_horizon_length_, sol_structure_.getNumberOfVariables()/2);
                    //etools::LeftLowerTriangularBlockKroneckerProduct<1,1>(Uus.selectRowInBlocksAsMatrix(0), 2).evaluate(Aps_);
                    */
                    sps_.noalias() = Uxs.selectRowInBlocks(0).getBlockKroneckerProduct(2) * mpc_base_state;
                    Uus.selectRowInBlocks(0).getBlockKroneckerProduct(2).evaluate(Aps_);

                    // -----
                    /*
                    // slower
                    spd_.noalias() =
                        R_ * (  etools::GenericBlockKroneckerProduct<1,3>(Uxs.selectRowInBlocksAsMatrix(0), 2) * mpc_base_state
                                -
                                etools::GenericBlockKroneckerProduct<1,3>(Uxd.selectRowInBlocksAsMatrix(0), 2) * mpc_body_state);
                    Apd_.resize(2*mpc_parameters_.preview_horizon_length_, sol_structure_.getNumberOfVariables ());
                    Apd_ <<  R_ * etools::LeftLowerTriangularBlockKroneckerProduct<1,1>(Uus.selectRowInBlocksAsMatrix(0), 2),
                             - (R_ * etools::LeftLowerTriangularBlockKroneckerProduct<1,1>(Uud.selectRowInBlocksAsMatrix(0), 2));
                    */
                    spd_.noalias() =
                        R_ * (  Uxs.selectRowInBlocks(0).getBlockKroneckerProduct(2) * mpc_base_state
                                -
                                Uxd.selectRowInBlocks(0).getBlockKroneckerProduct(2) * mpc_body_state);
                    Apd_.resize(2*mpc_parameters_.preview_horizon_length_, sol_structure_.getNumberOfVariables ());
                    Apd_ <<  R_ * Uus.selectRowInBlocks(0).getBlockKroneckerProduct(2),
                             - (R_ * Uud.selectRowInBlocks(0).getBlockKroneckerProduct(2));

                    // -----

                    sjs_.noalias() = Ojxs.getBlockKroneckerProduct(2) * mpc_base_state;

                    /*
                    // slower
                    Ajs_.resize(2*mpc_parameters_.preview_horizon_length_, sol_structure_.getNumberOfVariables ());
                    Ajs_ << Ojus.getBlockKroneckerProduct(2).evaluate(),
                            Eigen::MatrixXd::Zero(2*mpc_parameters_.preview_horizon_length_, sol_structure_.getNumberOfVariables()/2);
                    */
                    Ojus.getBlockKroneckerProduct(2).evaluate(Ajs_);
                }


                /**
                 * @brief Computes model state at the given time instant
                 *
                 * @param[in] model model
                 * @param[in] interval_index index of an interval in the preview horizon
                 * @param[in] interval_offset offset from the start of the given interval
                 *
                 * @return model state.
                 */
                humoto::pepper_mpc::ModelState  getModelState(  const humoto::pepper_mpc::Model &model,
                                                                const std::size_t interval_index,
                                                                const double interval_offset)
                {
                    double base_height = preview_horizon_.intervals_[interval_index].base_height_;
                    double body_height = preview_horizon_.intervals_[interval_index].body_height_;

                    double theta_prev;
                    if (0 == interval_index)
                    {
                        theta_prev = model.getTheta();
                    }
                    else
                    {
                        theta_prev = preview_horizon_.intervals_[interval_index-1].theta_;
                    }
                    double theta = theta_prev
                                    + (preview_horizon_.intervals_[interval_index].theta_ - theta_prev)
                                    * interval_offset / preview_horizon_.intervals_[interval_index].T_;


                    EIGENTOOLS_CONSTANT_SIZE_VECTOR(12) preceding_cstate;
                    etools::Vector2                    base_control;
                    etools::Vector2                    body_control;


                    if (interval_index == 0)
                    {
                        preceding_cstate = model.getMPCState();
                    }
                    else
                    {
                        preceding_cstate << cstate_profile_.segment((interval_index - 1)*model.Ns_/2, model.Ns_/2),
                                            cstate_profile_.segment(getPreviewHorizonLength()*model.Ns_/2
                                                                    + (interval_index - 1)*model.Ns_/2, model.Ns_/2);
                    }
                    base_control = base_controls_.segment(interval_index*2, 2);
                    body_control = body_controls_.segment(interval_index*2, 2);


                    humoto::pepper_mpc::ModelState  model_state;

                    model.getNextState( model_state,
                                        preview_horizon_.intervals_[interval_index].T_,
                                        interval_offset,
                                        preceding_cstate,
                                        base_controls_.head(2),
                                        body_controls_.head(2),
                                        base_height,
                                        body_height,
                                        theta);

                    return (model_state);
                }


            public:
                static const std::ptrdiff_t     DEFAULT_PREVIEW_HORIZON_SHIFT = -1;

                humoto::pepper_mpc::PreviewHorizon   preview_horizon_;


                etools::DiagonalBlockMatrix<2,2>     R_;


                Eigen::MatrixXd                 S_;
                Eigen::VectorXd                 s_;

                // CoP
                Eigen::VectorXd                 sp_;
                Eigen::MatrixXd                 Ap_;

                // base acceleration
                Eigen::VectorXd                 sas_;
                Eigen::MatrixXd                 Aas_;

                // base position
                Eigen::VectorXd                 sps_;
                Eigen::MatrixXd                 Aps_;

                // body position
                Eigen::VectorXd                 spd_;
                Eigen::MatrixXd                 Apd_;

                // base jerk
                Eigen::VectorXd                 sjs_;
                Eigen::MatrixXd                 Ajs_;


                // state and output profiles
                Eigen::VectorXd                 cstate_profile_;
                Eigen::VectorXd                 cop_profile_;
                Eigen::VectorXd                 base_jerk_profile_;

                // controls
                Eigen::VectorXd                 base_controls_;
                Eigen::VectorXd                 body_controls_;

                humoto::pepper_mpc::MPCParameters    mpc_parameters_;


            public:
                /**
                 * @brief Constructor
                 */
                MPCforMG()
                {
                    solution_is_parsed_ = false;
                }


                /**
                 * @brief Constructor
                 *
                 * @param[in] mpc_parameters parameters of the MPC
                 */
                explicit MPCforMG(const humoto::pepper_mpc::MPCParameters &mpc_parameters)
                {
                    solution_is_parsed_ = false;
                    setParameters(mpc_parameters);
                }


                /**
                 * @brief Set parameters
                 *
                 * @param[in] mpc_parameters
                 */
                void setParameters (const humoto::pepper_mpc::MPCParameters  &mpc_parameters)
                {
                    mpc_parameters_ = mpc_parameters;
                }


                /**
                 * @brief Update control problem
                 *
                 * @tparam t_MotionParameters   MotionParameters or deque<MotionParameters>
                 *
                 * @param[in] motion_parameters
                 * @param[in] model                 model of the system
                 *
                 * @return ControlProblemStatus::OK/ControlProblemStatus::STOPPED
                 */
                template <class t_MotionParameters>
                    ControlProblemStatus::Status
                        update( const t_MotionParameters    &motion_parameters,
                                const Model                 &model)
                {
                    humoto::ControlProblemStatus::Status    control_status = ControlProblemStatus::OK;
                    solution_is_parsed_ = false;

                    if (preview_horizon_.form(  mpc_parameters_,
                                                model,
                                                motion_parameters))
                    {
                        updateMatrices(model);
                        control_status = ControlProblemStatus::OK;
                    }
                    else
                    {
                        control_status = ControlProblemStatus::STOPPED;
                    }

                    return (control_status);
                }



                /**
                 * @brief Shift preview horizon
                 *
                 * @param[in,out] motion_parameters duration_ms parameter is reduced by time_shift_ms
                 * @param[in] time_shift_ms         shift of the preview horizon on the next iteration
                 */
                void shift( MotionParameters        &motion_parameters,
                            const std::ptrdiff_t    time_shift_ms = DEFAULT_PREVIEW_HORIZON_SHIFT)
                {
                    if (motion_parameters.duration_ms_ != MotionParameters::UNLIMITED_DURATION)
                    {
                        if (time_shift_ms == DEFAULT_PREVIEW_HORIZON_SHIFT)
                        {
                            motion_parameters.duration_ms_ -= mpc_parameters_.sampling_time_ms_;
                        }
                        else
                        {
                            motion_parameters.duration_ms_ -= time_shift_ms;
                        }

                        if (motion_parameters.duration_ms_ < 0)
                        {
                            motion_parameters.duration_ms_ = 0;
                        }
                    }
                }


                /**
                 * @brief Update control problem
                 *
                 * @param[in,out] motion_parameters_deque   duration_ms parameter is reduced by time_shift_ms
                 * @param[in] time_shift_ms                 shift of the preview horizon on the next iteration
                 */
                void shift( std::deque<MotionParameters>    &motion_parameters_deque,
                            const std::ptrdiff_t            time_shift_ms = DEFAULT_PREVIEW_HORIZON_SHIFT)
                {
                    std::ptrdiff_t tmp_time_shift_ms = time_shift_ms;
                    if (time_shift_ms == DEFAULT_PREVIEW_HORIZON_SHIFT)
                    {
                        tmp_time_shift_ms = mpc_parameters_.sampling_time_ms_;
                    }

                    for(;;)
                    {
                        if (motion_parameters_deque.size() > 0)
                        {
                            if (motion_parameters_deque.front().duration_ms_ == MotionParameters::UNLIMITED_DURATION)
                            {
                                // no need to shift
                                break;
                            }
                            else
                            {
                                if (motion_parameters_deque.front().duration_ms_ <= tmp_time_shift_ms)
                                {
                                    tmp_time_shift_ms -= motion_parameters_deque.front().duration_ms_;
                                    motion_parameters_deque.pop_front();
                                }
                                else
                                {
                                    motion_parameters_deque.front().duration_ms_ -= tmp_time_shift_ms;

                                    if (motion_parameters_deque.front().duration_ms_ < 0)
                                    {
                                        motion_parameters_deque.front().duration_ms_ = 0;
                                    }

                                    // successfully shifted
                                    break;
                                }
                            }
                        }
                        else
                        {
                            // deque reduced to zero
                            break;
                        }
                    }
                }


                /**
                 * @brief Update control problem & shift preview horizon
                 *
                 * @tparam t_MotionParameters   MotionParameters or deque<MotionParameters>
                 *
                 * @param[in,out] motion_parameters duration_ms parameter is reduced by time_shift_ms
                 * @param[in] model                 model of the system
                 * @param[in] time_shift_ms         shift of the preview horizon on the next iteration
                 *
                 * @return ControlProblemStatus::OK/ControlProblemStatus::STOPPED
                 */
                template <class t_MotionParameters>
                    ControlProblemStatus::Status
                        updateAndShift( t_MotionParameters      &motion_parameters,
                                        const Model             &model,
                                        const std::ptrdiff_t    time_shift_ms = DEFAULT_PREVIEW_HORIZON_SHIFT)
                {
                    ControlProblemStatus::Status status = update(motion_parameters, model);

                    if (status == ControlProblemStatus::OK)
                    {
                        shift(motion_parameters, time_shift_ms);
                    }
                    return (status);
                }



                /**
                 * @brief Process solution
                 *
                 * @param[in] solution  solution
                 */
                void    parseSolution(const humoto::Solution &solution)
                {
                    cstate_profile_.noalias()       = S_ * solution.get_x() + s_;
                    cop_profile_.noalias()          = Ap_ * solution.get_x() + sp_;
                    base_jerk_profile_.noalias()    = Ajs_ * solution.get_x().segment(0, sol_structure_.getNumberOfVariables()/2)
                                                        + sjs_;

                    base_controls_ = solution.getSolutionPart(BASE_VEL_VARIABLES_ID);
                    body_controls_ = solution.getSolutionPart(BODY_JERK_VARIABLES_ID);

                    solution_is_parsed_ = true;
                }


                /**
                 * @brief Get next model state.
                 *
                 * @param[in] solution  solution
                 * @param[in] model model
                 *
                 * @return next model state.
                 */
                humoto::pepper_mpc::ModelState   getNextModelState(
                        const humoto::Solution                          &solution,
                        const humoto::pepper_mpc::Model                      &model)
                {
                    HUMOTO_ASSERT(solution_is_parsed_ == false, "The solution is parsed.");
                    parseSolution(solution);
                    return(initializeNextModelState(model));
                }


                /**
                 * @brief Get next model state.
                 *
                 * @param[in] model model
                 *
                 * @return next model state.
                 */
                humoto::pepper_mpc::ModelState   getNextModelState(
                        const humoto::pepper_mpc::Model                      &model) const
                {
                    HUMOTO_ASSERT(solution_is_parsed_ == true, "This function can be called only after the solution is parsed.");
                    return(initializeNextModelState(model));
                }



                /**
                 * @brief Guess solution
                 *
                 * @param[in] old_solution old solution
                 * @param[out] solution_guess solution guess
                 */
                void guessSolution( Solution       &solution_guess,
                                    const Solution &old_solution) const
                {
                    solution_guess.initialize(sol_structure_, 0.0);


                    if ( (old_solution.isNonEmpty()) && (old_solution.getNumberOfVariables() != 0) )
                    {
                        Location old_part;
                        Location guess_part;

                        // base
                        old_part = old_solution.getSolutionPartLocation(BASE_VEL_VARIABLES_ID);
                        guess_part = solution_guess.getSolutionPartLocation(BASE_VEL_VARIABLES_ID);

                        guess_part.length_ -= 2;

                        old_part.offset_ +=2;
                        old_part.length_ -=2;

                        solution_guess.getData(guess_part) = old_solution.getData(old_part);


                        // body
                        old_part = old_solution.getSolutionPartLocation(BODY_JERK_VARIABLES_ID);
                        guess_part = solution_guess.getSolutionPartLocation(BODY_JERK_VARIABLES_ID);

                        guess_part.length_ -= 2;

                        old_part.offset_ +=2;
                        old_part.length_ -=2;

                        solution_guess.getData(guess_part) = old_solution.getData(old_part);
                    }
                }



                /**
                 * @brief Computes model state at the given time instant
                 *
                 * @param[in] model model
                 * @param[in] time_instant_ms time instant
                 *
                 * @return model state.
                 */
                humoto::pepper_mpc::ModelState  getModelState(const humoto::pepper_mpc::Model &model,
                                                          const std::size_t time_instant_ms)
                {
                    HUMOTO_ASSERT(solution_is_parsed_ == true, "This function can be called only after the solution is parsed.");


                    std::size_t time_offset_ms = time_instant_ms;
                    std::size_t interval_index = preview_horizon_.getIntervalIndexByTimeOffset(time_offset_ms);

                    return(getModelState(model, interval_index, convertMillisecondToSecond(time_offset_ms)));
                }


                /**
                 * @brief Computes model state at the given time instant
                 *
                 * @param[in] model model
                 * @param[in] time_instant double time instant
                 *
                 * @return model state.
                 */
                humoto::pepper_mpc::ModelState  getModelState(  const humoto::pepper_mpc::Model &model,
                                                                const double time_instant)
                {
                    HUMOTO_ASSERT(solution_is_parsed_ == true, "This function can be called only after the solution is parsed.");


                    double time_offset = time_instant;
                    std::size_t interval_index = preview_horizon_.getIntervalIndexByTimeOffset(time_offset);

                    return(getModelState(model, interval_index, time_offset));
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
                            const std::string & name = "mpcmg") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    preview_horizon_.log(logger, subname);


                    logger.log(LogEntryName(subname).add("R"), R_.getRaw());

                    logger.log(LogEntryName(subname).add("S") , S_);
                    logger.log(LogEntryName(subname).add("s") , s_);

                    logger.log(LogEntryName(subname).add("sp") , sp_);
                    logger.log(LogEntryName(subname).add("Ap") , Ap_);

                    logger.log(LogEntryName(subname).add("sas") , sas_);
                    logger.log(LogEntryName(subname).add("Aas") , Aas_);

                    logger.log(LogEntryName(subname).add("sps") , sps_);
                    logger.log(LogEntryName(subname).add("Aps") , Aps_);

                    logger.log(LogEntryName(subname).add("sjs") , sjs_);
                    logger.log(LogEntryName(subname).add("Ajs") , Ajs_);

                    logger.log(LogEntryName(subname).add("base_controls"), base_controls_);
                    logger.log(LogEntryName(subname).add("body_controls"), body_controls_);

                    logger.log(LogEntryName(subname).add("cstate_profile"), cstate_profile_);
                    logger.log(LogEntryName(subname).add("jerk_profile"), base_jerk_profile_);
                    logger.log(LogEntryName(subname).add("cop_profile"), cop_profile_);
                }
        };
    }
}

