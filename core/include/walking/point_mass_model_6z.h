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
    namespace walking
    {
        /**
         * @brief Point Mass Model with piece-wise constant CoP velocity
         *
         * This class generates matrices of the following systems:
         *
         * 1) controlled by position of the CoP
         *   c(k+1)  = A c(k) + B z(k+1)
         *   zdot(k) = D c(k) + E z(k+1)
         *
         * 2) controlled by velocity of the CoP
         *   c(k+1)  = Adz c(k) + Bdz dz(k+1)
         *   z(k)    = Ddz c(k)
         *
         * @todo It can be faster if A,B are computed directly as derived in
         * documentation.
         */
        class HUMOTO_LOCAL PointMassModel6z
        {
            private:
                /**
                 * @brief Create A matrix of unrefined model
                 */
                static etools::Matrix3 getAdz3(const double T, const double omega)
                {
                    double Tw = T*omega;

                    etools::Matrix3 A3;

                    double ch = cosh(Tw);
                    double sh = sinh(Tw);

                    A3 << 1., sh/omega, (ch - 1)/(omega*omega),
                          0., ch      ,  sh/omega,
                          0., omega*sh,  ch;

                    return(A3);
                }


                /**
                 * @brief Create B matrix of unrefined model
                 */
                static etools::Vector3 getBdz3(const double T, const double omega)
                {
                    double Tw = T*omega;

                    etools::Vector3 B3;

                    double ch = cosh(Tw);
                    double sh = sinh(Tw);

                    B3 << -sh/omega + T,
                          -ch + 1,
                          -omega*sh;

                    return(B3);
                }


                /**
                 * @brief Create D matrix of unrefined model
                 */
                static etools::Matrix1x3 getDdz3(const double omega)
                {
                    etools::Matrix1x3 D3;

                    D3 << 1., 0., -1./(omega*omega);

                    return(D3);
                }


            public:
                /// Number of state variables
                const std::size_t Ns_;

                /// Number of control variables
                const int Nu_;


            public:
                /**
                 * @brief Constructor
                 */
                PointMassModel6z() : Ns_(6), Nu_(2)
                {
                }


                static double getOmega(const double com_height)
                {
                    return(std::sqrt(humoto::g_gravitational_acceleration / com_height));
                }


                /**
                 * @brief Converts given cstate vector to CoM state.
                 *
                 * @return CoM state
                 */
                static humoto::rigidbody::PointMassState   convertCoMState(
                        const etools::Vector6 &cstate,
                        const double com_height)
                {
                    humoto::rigidbody::PointMassState com_state;

                    com_state.position_     << cstate(0), cstate(3), com_height;
                    com_state.velocity_     << cstate(1), cstate(4), 0.0;
                    com_state.acceleration_ << cstate(2), cstate(5), 0.0;

                    return (com_state);
                }


                /**
                 * @brief Get cstate
                 */
                static etools::Vector6 convertCoMState(const humoto::rigidbody::PointMassState &com_state)
                {
                    etools::Vector6 cstate;
                    cstate <<  com_state.position_.x(),
                               com_state.velocity_.x(),
                               com_state.acceleration_.x(),
                               com_state.position_.y(),
                               com_state.velocity_.y(),
                               com_state.acceleration_.y();
                    return(cstate);
                }


                static humoto::rigidbody::PointMassState   evaluate(
                        const double Ts,
                        const double T,
                        const double com_height,
                        const etools::Vector6 & cstate,
                        const etools::Vector2 & control)
                {
                    humoto::rigidbody::PointMassState com_state;


                    etools::Matrix6    A = getA6(Ts, getOmega(com_height), T);
                    etools::Matrix6x2  B = getB6(Ts, getOmega(com_height), T);

                    com_state = convertCoMState(A * cstate + B * control, com_height);

                    return (com_state);
                }


                /**
                 * @brief Create A matrix of final model
                 */
                static etools::Matrix3 getA3(const double T, const double omega, const double Tsample)
                {
                    etools::Matrix3 A3 = getAdz3(T, omega);
                    etools::Vector3 B3 = getBdz3(T, omega);
                    etools::Matrix1x3 D3 = getDdz3(omega);

                    A3 = A3 - B3*D3/Tsample;
                    return A3;
                }


                /**
                 * @brief Create A matrix of final model
                 */
                static etools::Matrix3 getA3(const double T, const double omega)
                {
                    return getA3(T, omega, T);
                }


                /**
                 * @brief Create A matrix of final model
                 */
                static etools::Matrix6 getA6(const double T, const double omega, const double Tsample)
                {
                    etools::Matrix3 A3 = getA3(T, omega, Tsample);

                    etools::Matrix6     out;
                    out << A3, Eigen::Matrix3d::Zero(),
                           Eigen::Matrix3d::Zero(), A3;
                    return out;
                }


                /**
                 * @brief Create A matrix of final model
                 */
                static etools::Matrix6 getA6(const double T, const double omega)
                {
                    return ( getA6(T, omega, T) );
                }


                /**
                 * @brief Create B matrix of final model
                 */
                static etools::Vector3 getB3(const double T, const double omega, const double Tsample)
                {
                    etools::Vector3 B3 = getBdz3(T, omega);
                    B3 = B3/Tsample;

                    return B3;
                }


                /**
                 * @brief Create A matrix of final model
                 */
                static etools::Vector3 getB3(const double T, const double omega)
                {
                    return getB3(T, omega, T);
                }


                /**
                 * @brief Create B matrix of final model
                 */
                static etools::Matrix6x2 getB6(const double T, const double omega, const double Tsample)
                {
                    etools::Vector3 B3 = getB3(T, omega, Tsample);

                    etools::Matrix6x2 out;
                    out << B3, Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), B3;

                    return out;
                }


                /**
                 * @brief Create B matrix of final model
                 */
                static etools::Matrix6x2 getB6(const double T, const double omega)
                {
                    return ( getB6(T, omega, T) );
                }


                /**
                 * @brief Create D matrix of final model
                 */
                static etools::Matrix1x3 getD3(const double T, const double omega)
                {
                    etools::Matrix1x3 D3 = getDdz3(omega);

                    D3 = -D3/T;
                    return (D3);
                }


                /**
                 * @brief Create D matrix of final model
                 */
                static etools::Matrix2x6 getD6(const double T, const double omega)
                {
                    etools::Matrix1x3 D3 = getD3(T, omega);

                    etools::Matrix2x6 out(2,6);
                    out << D3, etools::Matrix1x3::Zero(),
                           etools::Matrix1x3::Zero(), D3;
                    return out;
                }


                /**
                 * @brief Create E matrix of final model
                 */
                static double getE3(const double T)
                {
                    return (1./T);
                }


                /**
                 * @brief Create E matrix of final model
                 */
                static etools::Matrix2 getE6(const double T, const double omega)
                {
                    return (getE3(T) * Eigen::Matrix2d::Identity());
                }


                /**
                 * @brief Create Ddz6 matrix
                 */
                static etools::Matrix2x6   getDdz6(const double com_height)
                {
                    etools::Matrix2x6 out;
                    double hg = com_height / humoto::g_gravitational_acceleration;
                    out <<  1.,  0.,  -hg,    0.,   0.,  0.,
                            0.,  0.,  0.,     1.,   0.,  -hg;
                    return out;
                }


                /**
                 * @brief Create ksi matrix
                 *
                 * @note  Used to generate matrix for terminal constraint
                 */
                static etools::Matrix2x6   getDcpv6(const double omega)
                {
                    etools::Matrix2x6 out;
                    out <<  0., 1., 1./omega, 0., 0., 0.,
                            0., 0., 0.,  0., 1., 1./omega;
                    return out;
                }
        };
    }
}
