/**
    @file
    @author  Alexander Sherikov
    @author Jan Michalczyk
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
         * @brief Type of interpolated trajectory
         */
        class TrajectoryEvaluationType
        {
            public:
                enum Type
                {
                    UNDEFINED    = 0,
                    POSITION     = 1,
                    VELOCITY     = 2,
                    ACCELERATION = 3
                };
        };


        /**
         * @brief Class interpolating simple 1D cubic polynomial
         *        between two 1D points.
         *        Use this class to build more complex trajectories.
         *
         *        f(t) = a0 + a1 t + a2 t^2 + a3 t^3
         */
        class HUMOTO_LOCAL CubicPolynomial1D
        {
            public:
                /**
                 * @brief Constructor.
                 */
                CubicPolynomial1D()
                {
                    initialize(0, 0);
                }


                /**
                 * @brief Builds cubic polynomial in t = [0, 1]
                 *        with first derivative equal to 0 at both ends
                 *
                 * @param[in] a initial position
                 * @param[in] b final   position
                 */
                void initialize(const double a, const double b)
                {
                    a0_ = a;
                    a1_ = 0.0;
                    a2_ = 3.0*(b - a);
                    a3_ = -2.0*(b - a);
                }


                /**
                 * @brief Builds cubic polynomial in t = [0, 1]
                 *        with first derivative not equal to 0 at ends
                 *
                 * @param[in] a    initial 1D position
                 * @param[in] adot initial 1D velocity
                 * @param[in] b    final   1D position
                 * @param[in] bdot final   1D velocity
                 */
                void initialize(const double a,
                                const double adot,
                                const double b,
                                const double bdot)
                {
                    a0_ = a;
                    a1_ = adot;
                    a2_ =  3.0*(b - a) - 2.0*(adot - bdot);
                    a3_ = -2.0*(b - a) + 1.0*(bdot - adot);
                }


                /**
                 * @brief Computes position at the given time point.
                 *
                 * @param[in] t time
                 *
                 * @return position at the given time point
                 */
                double getPosition(const double t) const
                {
                    HUMOTO_ASSERT((t >= 0.0) && (t <= 1.0), "time value not between 0 and 1." )
                    return (a0_ + a1_*t + a2_*t*t + a3_*t*t*t);
                }


                /**
                 * @brief Computes velocity at the given time point.
                 *
                 * @param[in] t time
                 *
                 * @return velocity at the given time point
                 */
                double getVelocity(const double t) const
                {
                    HUMOTO_ASSERT((t >= 0.0) && (t <= 1.0), "time value not between 0 and 1." )
                    return (a1_ + 2.0*a2_*t + 3.0*a3_*t*t);
                }


                /**
                 * @brief Computes acceleration at the given time point.
                 *
                 * @param[in] t time
                 *
                 * @return acceleration at the given time point
                 */
                double getAcceleration(const double t) const
                {
                    HUMOTO_ASSERT((t >= 0.0) && (t <= 1.0), "time value not between 0 and 1." )
                    return (2.0*a2_ + 6.0*a3_*t);
                }


                /**
                 * @brief Computes jerk.
                 *
                 * @return jerk.
                 */
                double getJerk() const
                {
                    return (6.0*a3_);
                }


                /**
                 * @brief Evaluate the spline at a point.
                 *
                 * @param[in] t time point
                 * @param[in] trajectory_type type of trajectory
                 *
                 * @return    evaluated sample
                 */
                double eval(const double t,
                            const TrajectoryEvaluationType::Type trajectory_type) const
                {
                    switch(trajectory_type)
                    {
                        case TrajectoryEvaluationType::POSITION:
                            return getPosition(t);
                        case TrajectoryEvaluationType::VELOCITY:
                            return getVelocity(t);
                        case TrajectoryEvaluationType::ACCELERATION:
                            return getAcceleration(t);
                        default:
                            HUMOTO_THROW_MSG("Unsupported trajectory type.");
                    }
                }


                /**
                 * @brief Evaluate the polynomial at time_interval points.
                 *
                 * @param[in] time_points eigen vector with time instants at which we evaluate
                 * @param[in] trajectory_type type of trajectory
                 *
                 * @return    std vector with evaluated samples
                 */
                std::vector<double> eval(   const Eigen::VectorXd& time_points,
                                            const TrajectoryEvaluationType::Type trajectory_type) const
                {
                    std::vector<double> result(time_points.size());

                    for(std::size_t i = 0; i < result.size(); ++i)
                    {
                        result[i] = eval(time_points(i), trajectory_type);
                    }

                    return result;
                }


                /**
                 * @brief Scale vector.
                 *
                 * @param[in] time_points eigen vector
                 *
                 * @return    eigen vector of scaled values
                 */
                static Eigen::VectorXd scale(const Eigen::VectorXd& time_points)
                {
                    Eigen::VectorXd result(time_points.size());

                    for(EigenIndex i = 0; i < result.size(); ++i)
                    {
                        result[i] = (time_points(i) - time_points.minCoeff()) / (time_points.maxCoeff() - time_points.minCoeff());
                    }

                    return result;
                }


            private:
                double a0_;
                double a1_;
                double a2_;
                double a3_;
        };
    }
}
