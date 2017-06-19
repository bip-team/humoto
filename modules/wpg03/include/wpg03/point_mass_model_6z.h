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
         * @brief Point Mass Model with piece-wise constant CoM jerk
         *
         *   c(k+1) = A c(k) + B u(k)
         *   z(k+1) = D c(k) + E u(k) + F f(k)
         */
        class PointMassModel6z
        {
            public:
                /// state defined by the center of mass
                Eigen::Matrix<double, 6, 1> cstate_;

                /// Number of state variables
                const unsigned int Ns_;

                /// Number of control variables
                const int Nu_;

                PointMassModel6z();
                Eigen::Matrix<double, 6, 6>     getA6(const double T) const;
                Eigen::Matrix<double, 6, 2>     getB6(const double T) const;
                Eigen::Matrix<double, 2, 6>     getD6(const double omega, const double mass, const double f_ext_z) const;
                Eigen::Matrix<double, 2, 2>     getE2() const; // zeros matrix, here for consistency with the math
                Eigen::Matrix<double, 2, 4>     getF4(const double com_height, const double mass, const double f_ext_z) const;
                Eigen::Matrix<double, 2, 6>     getDdz6(const double hg) const;

            private:
                /* These functions create the matrices of the unrefined model
                 * c(k+1) = A c(k) + B u(k)
                 * z(k)   = D c(k) + E u(k) + F f(k)
                 * Note that A, B, D, E here are used to create those of the final model but they are not equal
                 */
                Eigen::Matrix<double, 3, 3> formA3(const double T) const;
                Eigen::Matrix<double, 3, 1> formB3(const double T) const;
                Eigen::Matrix<double, 1, 3> formD3(const double omega, const double mass, const double f_ext_z) const;
                Eigen::Matrix<double, 1, 1> formE1() const; // zeros matrix, here for consistency with the math
        };

        /**
         * @brief Constructor
         */
        PointMassModel6z::PointMassModel6z() : Ns_(6), Nu_(2)
        {
            cstate_ << 0., 0., 0., 0., 0., 0;
        }


        /**
         * @brief Create A matrix of unrefined model
         */
        Eigen::Matrix<double, 3, 3> PointMassModel6z::formA3(const double T) const
        {
            Eigen::Matrix<double, 3, 3> A3;

            A3 << 1., T, pow(T,2.)/2.,
                  0., 1., T,
                  0., 0., 1.;

            return(A3);
        }


        /**
         * @brief Create B matrix of unrefined model
         */
        Eigen::Matrix<double, 3, 1> PointMassModel6z::formB3(const double T) const
        {
            Eigen::Matrix<double, 3, 1> B3;

            B3 << pow(T,3.)/6.,
                  pow(T,2.)/2.,
                  T;

            return(B3);
        }


        /**
         * @brief Create D matrix of unrefined model
         */
        Eigen::Matrix<double, 1, 3> PointMassModel6z::formD3(const double omega, const double mass, const double f_ext_z) const
        {
            Eigen::Matrix<double, 1, 3> D3;
            double mg = mass*humoto::g_gravitational_acceleration;
            D3 << 1., 0., -mg/((mg - f_ext_z)*pow(omega, 2.));

            return(D3);
        }


        /**
         * @brief Create E matrix of unrefined model
         */
        Eigen::Matrix<double, 1, 1> PointMassModel6z::formE1() const
        {
            Eigen::Matrix<double, 1, 1> E3;

            E3 << 0.;

            return(E3);
        }


        /**
         * @brief Create A matrix of final model
         */
        Eigen::Matrix<double, 6, 6> PointMassModel6z::getA6(const double T) const
        {
            Eigen::Matrix<double, 3, 3> A3 = formA3(T);

            Eigen::Matrix<double, 6, 6>     out;
            out << A3, Eigen::Matrix3d::Zero(),
                   Eigen::Matrix3d::Zero(), A3;
            return out;
        }


        /**
         * @brief Create B matrix of final model
         */
        Eigen::Matrix<double, 6, 2> PointMassModel6z::getB6(const double T) const
        {
            Eigen::Matrix<double, 3, 1> B3 = formB3(T);

            Eigen::Matrix<double, 6, 2> out;
            out << B3, etools::Vector3::Zero(),
                   etools::Vector3::Zero(), B3;

            return out;
        }


        /**
         * @brief Create D matrix of final model
         */
        Eigen::Matrix<double, 2, 6> PointMassModel6z::getD6(const double omega, const double mass, const double f_ext_z) const
        {
            Eigen::Matrix<double, 1, 3> D3 = formD3(omega, mass, f_ext_z);

            Eigen::Matrix<double, 2, 6> out;
            out << D3, Eigen::Matrix<double, 1, 3>::Zero(),
                   Eigen::Matrix<double, 1, 3>::Zero(), D3;
            return out;
        }


        /**
         * @brief Create E matrix of final model
         */
        Eigen::Matrix<double, 2, 2> PointMassModel6z::getE2() const
        {
            Eigen::Matrix<double, 1, 1> E1 = formE1();

            Eigen::Matrix<double, 2, 2> out;
            out << E1, 0.,
                   0., E1;
            return out;
        }


        /**
         * @brief Create F4 matrix of the final model
         */
        Eigen::Matrix<double, 2, 4>   PointMassModel6z::getF4(const double com_height, const double mass, const double f_ext_z) const
        {
            Eigen::Matrix<double, 2, 4> out;
            double factor = 1./(mass*humoto::g_gravitational_acceleration - f_ext_z);
            out << factor, com_height*factor, 0., 0.,
                   0., 0., -factor, com_height*factor;
            return out;
        }


        /**
         * @brief Create Ddz6 matrix
         */
        Eigen::Matrix<double, 2, 6>   PointMassModel6z::getDdz6(const double hg) const
        {
            Eigen::Matrix<double, 2, 6> out;
            out <<  1.,  0.,  -hg,    0.,   0.,  0.,
                    0.,  0.,  0.,     1.,   0.,  -hg;
            return out;
        }
    }
}
