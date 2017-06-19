/**
    @file
    @author  Jan Michalczyk
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @brief
*/

#pragma once

namespace humoto
{
    namespace rigidbody
    {
        /**
         * @brief Triple integrator class
         *        Class supports arbitrary number of integrators in the system
         *
         * This class generates matrices of the following systems:
         *
         * 1) controlled by acceleration
         *    x(k+1)  = A x(k) + B ddx(k+1)
         *    dddx(k) = D x(k) + E ddx(k+1)
         *
         * 2) controlled by velocity
         *    x(k+1)  = A x(k) + B dx(k+1)
         *    dddx(k) = D x(k) + E dx(k+1)
         *
         * 3) controlled by position
         *    x(k+1)  = A x(k) + B x(k+1)
         *    dddx(k) = D x(k) + E x(k+1)
         *
         * 4) controlled by jerk
         *    x(k+1)  = A x(k) + B dddx(k+1)
         */
        class HUMOTO_LOCAL TripleIntegrator
        {
            public:
                /**
                 * @brief Create A matrix of the model, suffix of the function
                 * name indicates the type of control [jerk, acc =
                 * acceleration, vel = velocity, pos = position].
                 *
                 * @tparam t_number_of_integrators number of integrators in the system
                 *
                 * @param[in] T sampling time
                 *
                 * @return    A matrix
                 */
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 3*t_number_of_integrators)
                        getAJerk(const double T)
                {
                    etools::Matrix3 A3;
                    A3 << 1., T,  T*T/2.,
                          0., 1., T,
                          0., 0., 1.;
                    return(etools::makeBlockDiagonal(A3, t_number_of_integrators));
                }

                //======================================================================

                /// @copydoc getAJerk()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 3*t_number_of_integrators)
                        getAAcc(const double T)
                {
                    etools::Matrix3 A3;
                    A3 << 1., T,  T*T/3.,
                          0., 1., T/2.,
                          0., 0., 0.;
                    return(etools::makeBlockDiagonal(A3, t_number_of_integrators));
                }


                /// @copydoc getAJerk()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 3*t_number_of_integrators)
                        getAVel(const double T)
                {
                    etools::Matrix3 A3;
                    A3 << 1.,  2.*T/3., T*T/6.,
                          0.,  0.,      0.,
                          0., -2./T,   -1.;
                    return(etools::makeBlockDiagonal(A3, t_number_of_integrators));
                }


                /// @copydoc getAJerk()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 3*t_number_of_integrators)
                        getAPos(const double T)
                {
                    etools::Matrix3 A3;
                    A3 << 0.,        0.,    0.,
                         -3./T,     -2.,   -T/2.,
                         -6./(T*T), -6./T, -2.;
                    return(etools::makeBlockDiagonal(A3, t_number_of_integrators));
                }

                //======================================================================

                /**
                 * @brief Create A matrix of the model, suffix of the function
                 * name indicates the type of control [jerk, acc =
                 * acceleration, vel = velocity, pos = position].
                 *
                 * @tparam t_number_of_integrators number of integrators in the system
                 *
                 * @param[in] T sampling time
                 * @param[in] Ts subsampling time
                 *
                 * @return    A matrix
                 */
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 3*t_number_of_integrators)
                        getAsAcc(const double T, const double Ts)
                {
                    etools::Matrix3 A3;
                    A3 <<
                        1 , Ts   ,  - (Ts*Ts*Ts - 3*T*Ts*Ts)/(6*T) ,
                        0 , 1    ,  - (Ts*Ts - 2*T*Ts)/(2*T) ,
                        0 , 0    ,  - (Ts - T)/T ;
                    return(etools::makeBlockDiagonal(A3, t_number_of_integrators));
                }


                /// @copydoc getAsAcc()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 3*t_number_of_integrators)
                        getAsVel(const double T, const double Ts)
                {
                    etools::Matrix3 A3;
                    A3 <<
                        1 ,  - (Ts*Ts*Ts - 3*T*T*Ts)/(3*T*T) ,  - (2*Ts*Ts*Ts - 3*T*Ts*Ts)/(6*T) ,
                        0 ,  - (Ts*Ts - T*T)/(T*T)        ,  - (Ts*Ts - T*Ts)/T ,
                        0 ,  - (2*Ts)/(T*T)              ,  - (2*Ts - T)/T ;
                    return(etools::makeBlockDiagonal(A3, t_number_of_integrators));
                }


                /// @copydoc getAsAcc()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 3*t_number_of_integrators)
                        getAsPos(const double T, const double Ts)
                {
                    etools::Matrix3 A3;
                    A3 <<
                        - (Ts*Ts*Ts - T*T*T)/(T*T*T) ,  - (Ts*Ts*Ts - T*T*Ts)/(T*T)    ,  - (Ts*Ts*Ts - T*Ts*Ts)/(2*T) ,
                        - (3*Ts*Ts)/(T*T*T)     ,  - (3*Ts*Ts - T*T)/(T*T)     ,  - (3*Ts*Ts - 2*T*Ts)/(2*T) ,
                        - (6*Ts)/(T*T*T)       ,  - (6*Ts)/(T*T)             ,  - (3*Ts - T)/T ;
                    return(etools::makeBlockDiagonal(A3, t_number_of_integrators));
                }


                //======================================================================
                //======================================================================
                //======================================================================


                /**
                 * @brief Create B matrix of the model, suffix of the function
                 * name indicates the type of control [jerk, acc =
                 * acceleration, vel = velocity, pos = position].
                 *
                 * @tparam t_number_of_integrators number of integrators in the system
                 *
                 * @param[in] T sampling time
                 * @return    B3 matrix
                 */
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 1*t_number_of_integrators)
                        getBJerk(const double T)
                {
                    etools::Vector3 B3;
                    B3 << T*T*T/6.,
                          T*T/2.,
                          T;
                    return(etools::makeBlockDiagonal(B3, t_number_of_integrators));
                }

                //======================================================================

                /// @copydoc getBJerk()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 1*t_number_of_integrators)
                        getBAcc(const double T)
                {
                    etools::Vector3 B3;
                    B3 << T*T/6.,
                          T/2.,
                          1.;
                    return(etools::makeBlockDiagonal(B3, t_number_of_integrators));
                }


                /// @copydoc getBJerk()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 1*t_number_of_integrators)
                        getBVel(const double T)
                {
                    etools::Vector3 B3;
                    B3 << T/3.,
                          1.,
                          2./T;
                    return(etools::makeBlockDiagonal(B3, t_number_of_integrators));
                }


                /// @copydoc getBJerk()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 1*t_number_of_integrators)
                        getBPos(const double T)
                {
                    etools::Vector3 B3;
                    B3 << 1.,
                          3./T,
                          6./(T*T);
                    return(etools::makeBlockDiagonal(B3, t_number_of_integrators));
                }

                //======================================================================

                /**
                 * @brief Create B matrix of the model, suffix of the function
                 * name indicates the type of control [jerk, acc =
                 * acceleration, vel = velocity, pos = position].
                 *
                 * @tparam t_number_of_integrators number of integrators in the system
                 *
                 * @param[in] T sampling time
                 * @param[in] Ts subsampling time
                 * @return    B3 matrix
                 */
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 1*t_number_of_integrators)
                        getBsAcc(const double T, const double Ts)
                {
                    etools::Vector3 B3;
                    B3 <<
                        (Ts*Ts*Ts)/(6*T) ,
                        (Ts*Ts)/(2*T) ,
                        (Ts)/(T) ;
                    return(etools::makeBlockDiagonal(B3, t_number_of_integrators));
                }


                /// @copydoc getBsAcc()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 1*t_number_of_integrators)
                        getBsVel(const double T, const double Ts)
                {
                    etools::Vector3 B3;
                    B3 <<
                        (Ts*Ts*Ts)/(3*T*T) ,
                        (Ts*Ts)/(T*T) ,
                        (2*Ts)/(T*T) ;
                    return(etools::makeBlockDiagonal(B3, t_number_of_integrators));
                }


                /// @copydoc getBsAcc()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(3*t_number_of_integrators, 1*t_number_of_integrators)
                        getBsPos(const double T, const double Ts)
                {
                    etools::Vector3 B3;
                    B3 <<
                        (Ts*Ts*Ts)/(T*T*T) ,
                        (3*Ts*Ts)/(T*T*T) ,
                        (6*Ts)/(T*T*T) ;
                    return(etools::makeBlockDiagonal(B3, t_number_of_integrators));
                }


                //======================================================================
                //======================================================================
                //======================================================================


                /**
                 * @brief Create D matrix of the model, suffix of the function
                 * name indicates the type of control [acc = acceleration, vel
                 * = velocity, pos = position].
                 *
                 * @tparam t_number_of_integrators number of integrators in the system
                 *
                 * @param[in] T sampling time
                 * @return    A matrix
                 */
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(1*t_number_of_integrators, 3*t_number_of_integrators)
                        getDAcc(const double T)
                {
                    etools::Matrix1x3 D3;
                    D3 << 0., 0., -1./T;
                    return(etools::makeBlockDiagonal(D3, t_number_of_integrators));
                }


                /// @copydoc getDAcc()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(1*t_number_of_integrators, 3*t_number_of_integrators)
                        getDVel(const double T)
                {
                    etools::Matrix1x3 D3;
                    D3 << 0., -2./(T*T), -2./T;
                    return(etools::makeBlockDiagonal(D3, t_number_of_integrators));
                }


                /// @copydoc getDAcc()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(1*t_number_of_integrators, 3*t_number_of_integrators)
                        getDPos(const double T)
                {
                    etools::Matrix1x3 D3;
                    D3 << -6./(T*T*T), -6./(T*T), -3./T;
                    return(etools::makeBlockDiagonal(D3, t_number_of_integrators));
                }

                //======================================================================
                //======================================================================
                //======================================================================


                /**
                 * @brief Create E matrix of the model, suffix of the function
                 * name indicates the type of control [acc = acceleration, vel
                 * = velocity, pos = position].
                 *
                 * @tparam t_number_of_integrators number of integrators in the system
                 *
                 * @param[in] T sampling time
                 * @return    A matrix
                 */
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(1*t_number_of_integrators, 1*t_number_of_integrators)
                        getEAcc(const double T)
                {
                    double E3;
                    E3 = 1./T;
                    return(E3*EIGENTOOLS_CONSTANT_SIZE_MATRIX(1*t_number_of_integrators, 1*t_number_of_integrators)::Identity(t_number_of_integrators, t_number_of_integrators));
                }


                /// @copydoc getEAcc()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(1*t_number_of_integrators, 1*t_number_of_integrators)
                        getEVel(const double T)
                {
                    double E3;
                    E3 = 2./(T*T);
                    return(E3*EIGENTOOLS_CONSTANT_SIZE_MATRIX(1*t_number_of_integrators, 1*t_number_of_integrators)::Identity(t_number_of_integrators, t_number_of_integrators));
                }


                /// @copydoc getEAcc()
                template <std::size_t t_number_of_integrators>
                    static EIGENTOOLS_CONSTANT_SIZE_MATRIX(1*t_number_of_integrators, 1*t_number_of_integrators)
                        getEPos(const double T)
                {
                    double E3;
                    E3 = 6./(T*T*T);
                    return(E3*EIGENTOOLS_CONSTANT_SIZE_MATRIX(1*t_number_of_integrators, 1*t_number_of_integrators)::Identity(t_number_of_integrators, t_number_of_integrators));
                }
        };
    }
}
