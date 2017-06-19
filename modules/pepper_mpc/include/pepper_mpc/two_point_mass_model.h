/**
    @file
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
         * @brief Two Point Mass Model
         *
         * This class generates matrices of the following systems:
         *
         * 1) simple:
         *    c(k+1) = A c(k) + B cjerk(k+1)
         *    p(k)   = D c(k)
         *
         * 2) final:
         *    c(k+1)   = A c(k) + Bs us(k) + Bd RS(k+1) ud(k)
         *    p(k)     = Dps c(k)
         *    cjerk(k) = Dcjerk c(k) + Escjerk us(k) + Edcjerk RS(k+1) ud(k)
         */
        class HUMOTO_LOCAL TwoPointMassModel : protected humoto::rigidbody::TripleIntegrator
        {
            public:
                // number of state varaibles
                const std::size_t Ns_;
                // number of control varaibles
                const std::size_t Nu_;


            public:
                /**
                 * @brief Constructor
                 */
                TwoPointMassModel() : Ns_(12), Nu_(4)
                {
                }



                /**
                 * @brief Create intermediate As3 matrix
                 *
                 * @param[in] T timestep
                 * @return    3x3 matrix
                 */
                etools::Matrix3 getAs3(const double T) const
                {
                    return(getAVel<1>(T));
                }


                /**
                 * @brief Create intermediate Ad3 matrix
                 *
                 * @param[in] T timestep
                 * @return    3x3 matrix
                 */
                etools::Matrix3 getAd3(const double T) const
                {
                    return(getAJerk<1>(T));
                }


                /**
                 * @brief Create intermediate Bs3 matrix
                 *
                 * @param[in] T timestep
                 * @return    Bs6 matrix
                 */
                etools::Vector3 getBs3(const double T) const
                {
                    return(getBVel<1>(T));
                }


                /**
                 * @brief Create intermediate Bd3 matrix
                 *
                 * @param[in] T timestep
                 * @return    Bd6 matrix
                 */
                etools::Vector3 getBd3(const double T) const
                {
                    return(getBJerk<1>(T));
                }


                /**
                 * @brief Create Dps3 matrix
                 *
                 * @param[in] base_height CoM of base height
                 * @param[in] base_mass   base mass
                 * @param[in] body_mass   body mass
                 * @return    matrix
                 */
                etools::Matrix1x3 getDps3(  const double base_height,
                                                    const double base_mass,
                                                    const double body_mass) const
                {
                    etools::Matrix1x3 D;
                    D << -body_mass, 0., -base_mass * (base_height / humoto::g_gravitational_acceleration);

                    return(1./(base_mass + body_mass) * D);
                }


                /**
                 * @brief Create Dpd3 matrix
                 *
                 * @param[in] body_height CoM of body height
                 * @param[in] base_mass   base mass
                 * @param[in] body_mass   body mass
                 * @return    matrix
                 */
                etools::Matrix1x3 getDpd3(  const double body_height,
                                                    const double base_mass,
                                                    const double body_mass) const
                {
                    etools::Matrix1x3 D;
                    D << body_mass, 0., -body_mass * (body_height / humoto::g_gravitational_acceleration);

                    return(1./(base_mass + body_mass) * D);
                }



                /**
                 * @brief Create intermediate Dc6 matrix
                 *        of final model
                 *
                 * @param[in] T timestep
                 * @return    Dcjerk6 matrix
                 */
                etools::Matrix1x3 getDjs3(const double T) const
                {
                    return(getDVel<1>(T));
                }


                /**
                 * @brief Create intermediate Esc6 matrix
                 *        of final model
                 *
                 * @param[in] T timestep
                 * @return    Escjerk6 matrix
                 */
                double getEjs3(const double T) const
                {
                    return(getEVel<1>(T)(0,0));
                }



                // convert body state to mpc state


                /**
                 * @brief Get mpc base state
                 *
                 * @param[in] base_state base state
                 * @param[in] body_state body state
                 *
                 * @return mpc state
                 */
                EIGENTOOLS_CONSTANT_SIZE_VECTOR(6) convertBaseStateRBtoMPC(const humoto::rigidbody::RigidBodyState& base_state) const
                {
                    EIGENTOOLS_CONSTANT_SIZE_VECTOR(6) mpcstate;
                    mpcstate <<  base_state.position_.x(),
                                 base_state.velocity_.x(),
                                 base_state.acceleration_.x(),
                                 base_state.position_.y(),
                                 base_state.velocity_.y(),
                                 base_state.acceleration_.y();
                    return(mpcstate);
                }


                /**
                 * @brief Get mpc body state
                 *
                 * @param[in] base_state base state
                 * @param[in] body_state body state
                 *
                 * @return mpc state
                 */
                EIGENTOOLS_CONSTANT_SIZE_VECTOR(6) convertBodyStateRBtoMPC(const humoto::rigidbody::PointMassState& body_state) const
                {
                    EIGENTOOLS_CONSTANT_SIZE_VECTOR(6) mpcstate;
                    mpcstate <<  body_state.position_.x(),
                                 body_state.velocity_.x(),
                                 body_state.acceleration_.x(),
                                 body_state.position_.y(),
                                 body_state.velocity_.y(),
                                 body_state.acceleration_.y();
                    return(mpcstate);
                }



                /**
                 * @brief Get mpc state
                 *
                 * @param[in] base_state base state
                 * @param[in] body_state body state
                 *
                 * @return mpc state
                 */
                EIGENTOOLS_CONSTANT_SIZE_VECTOR(12) convertStateRBtoMPC(const humoto::rigidbody::RigidBodyState& base_state,
                                                                        const humoto::rigidbody::PointMassState& body_state) const
                {
                    EIGENTOOLS_CONSTANT_SIZE_VECTOR(12) mpcstate;
                    mpcstate << convertBaseStateRBtoMPC(base_state),
                                convertBodyStateRBtoMPC(body_state);
                    return(mpcstate);
                }



                /**
                 * @brief Converts given mpcstate vector to base state
                 *
                 * @param[in] mpc_state   mpc state vector
                 * @param[in] base_height body height
                 * @return    base state
                 */
                humoto::rigidbody::RigidBodyState convertBaseStateMPCtoRB(  const EIGENTOOLS_CONSTANT_SIZE_VECTOR(12) &mpcstate,
                                                                            const double base_height) const
                {
                    humoto::rigidbody::RigidBodyState base_state;

                    base_state.position_     << mpcstate(0), mpcstate(3), base_height;
                    base_state.velocity_     << mpcstate(1), mpcstate(4), 0.0;
                    base_state.acceleration_ << mpcstate(2), mpcstate(5), 0.0;

                    return(base_state);
                }


                /**
                 * @brief Converts given mpcstate vector to base and body states
                 *
                 * @param[in] mpc_state   mpc state vector
                 * @param[in] body_height body height
                 * @return    body state
                 */
                humoto::rigidbody::PointMassState convertBodyStateMPCtoRB(  const EIGENTOOLS_CONSTANT_SIZE_VECTOR(12) &mpcstate,
                                                                            const double body_height) const
                {
                    humoto::rigidbody::PointMassState body_state;

                    body_state.position_     << mpcstate(6), mpcstate(9),  body_height;
                    body_state.velocity_     << mpcstate(7), mpcstate(10), 0.0;
                    body_state.acceleration_ << mpcstate(8), mpcstate(11), 0.0;

                    return(body_state);
                }


                /**
                 * @brief Evaluate the state of the base
                 *
                 * @param[in] T                  timestep
                 * @param[in] Ts                 subsampling timestep (use Ts = T if not needed)
                 * @param[in] preceding_mpcstate mpc state
                 * @param[in] control            control vector
                 * @param[in] rotation           rotation matrix from base to global frame
                 *
                 * @return    base state
                 */
                 EIGENTOOLS_CONSTANT_SIZE_VECTOR(12) evaluate(
                                const double T,
                                const double Ts,
                                const EIGENTOOLS_CONSTANT_SIZE_VECTOR(12) & preceding_mpcstate,
                                const etools::Vector2 &base_control,
                                const etools::Vector2 &body_control) const
                {
                    etools::Vector4 control;

                    control << base_control, body_control;

                    return( getA12(T, Ts) * preceding_mpcstate
                            +
                            getB12(T, Ts) * control);
                }


                // final model


                /**
                 * @brief Create A matrix of final model
                 *
                 * @param[in] T timestep
                 * @param[in] Ts subsampling timestep (use Ts = T if not needed)
                 * @return    A matrix
                 */
                Eigen::MatrixXd getA12(const double T, const double Ts) const
                {
                    Eigen::MatrixXd A12;
                    etools::BlockMatrixInterface<3,3,etools::MatrixSparsityType::DIAGONAL> A12_bmi(A12);

                    A12_bmi.setZero(4,4);

                    if (Ts == T)
                    {
                        A12_bmi(0) = getAVel<1>(T);
                    }
                    else
                    {
                        A12_bmi(0) = getAsVel<1>(T, Ts);
                    }
                    A12_bmi(1) = A12_bmi(0);

                    A12_bmi(2) = getAJerk<1>(Ts);
                    A12_bmi(3) = A12_bmi(2);

                    return(A12);
                }



                /**
                 * @brief Create Bs matrix of final model
                 *
                 * @param[in] T  timestep
                 * @param[in] Ts subsampling timestep (use Ts = T if not needed)
                 * @return    Bs matrix
                 */
                Eigen::MatrixXd getB12(const double T, const double Ts) const
                {
                    Eigen::MatrixXd B12;
                    etools::BlockMatrixInterface<3,1,etools::MatrixSparsityType::DIAGONAL> B12_bmi(B12);

                    B12_bmi.setZero(4,4);

                    if (Ts == T)
                    {
                        B12_bmi(0) = getBVel<1>(T);
                    }
                    else
                    {
                        B12_bmi(0) = getBsVel<1>(T, Ts);
                    }
                    B12_bmi(1) = B12_bmi(0);

                    B12_bmi(2) = getBJerk<1>(Ts);
                    B12_bmi(3) = B12_bmi(2);

                    return(B12);
                }




                /**
                 * @brief Create Djs matrix of final model
                 *
                 * @param[in] T      timestep
                 * @return    Dcjerk matrix
                 */
                Eigen::MatrixXd getDj6(const double T) const
                {
                    Eigen::MatrixXd D;
                    etools::BlockMatrixInterface<1,3,etools::MatrixSparsityType::DIAGONAL> D_bmi(D);

                    D_bmi.setZero(2,2);
                    D_bmi(0) = getDVel<1>(T);
                    D_bmi(1) = D_bmi(0);

                    return(D);
                }


                /**
                 * @brief Create Ejs matrix of final model
                 *
                 * @param[in] T         timestep
                 * @return    Escjerk matrix
                 */
                Eigen::MatrixXd getEjs6(const double T) const
                {
                    etools::Matrix2 E;

                    E.setZero(2,2);
                    E(0,0) = getEVel<1>(T)(0,0);
                    E(1,1) = E(0,0);

                    return(E);
                }
        };
    } // pepper
} // humoto
