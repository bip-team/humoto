/**
    @file
    @author Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @brief
*/

#pragma once

namespace humoto
{
    namespace walking
    {
        /**
         * @brief Class implementing simple example foot trajectory
         *        between two 3D points.
         *
         *        trajectory along z is composed of two separate trajectories.
         */
        class HUMOTO_LOCAL Simple3DFootTrajectory
        {
            public:
                /**
                 * @brief Constructor.
                 */
                Simple3DFootTrajectory()
                {
                }


                /**
                 * @brief Constructor.
                 *        Velocities set to zero at both ends of the trajectory
                 *
                 * @param[in] a           initial 3D configuration [x, y, z]
                 * @param[in] b           final 3D configuration   [x, y, z]
                 * @param[in] step_height height of the step
                 */
                void initialize(const etools::Vector3& a,
                                const etools::Vector3& b,
                                const double step_height)
                {
                    HUMOTO_ASSERT(b(2) <= (a(2) + step_height), "final z position greater than step height.")
                    x_.initialize( a(0),               b(0));
                    y_.initialize( a(1),               b(1));
                    z1_.initialize(a(2),               a(2) + step_height);
                    z2_.initialize(step_height + a(2), b(2));
                }


                /**
                 * @brief Constructor.
                 *        Velocities set to non-zero at both ends of the trajectory
                 *
                 * @param[in] a           initial positions  [x, y, z]
                 * @param[in] adot        initial velocities [xdot, ydot, zdot]
                 * @param[in] b           final positions    [x, y, z]
                 * @param[in] bdot        final velocities   [xdot, ydot, zdot]
                 * @param[in] step_height height of the step
                 */
                void initialize(const etools::Vector3& a, const etools::Vector3& adot,
                                const etools::Vector3& b, const etools::Vector3& bdot,
                                const double step_height)
                {
                    HUMOTO_ASSERT(b(2) <= (a(2) + step_height), "final z position greater than step height.")
                    x_.initialize( a(0),               adot(0), b(0),               bdot(0));
                    y_.initialize( a(1),               adot(1), b(1),               bdot(1));
                    z1_.initialize(a(2),               adot(2), a(2) + step_height, 0.0);
                    z2_.initialize(step_height + a(2), 0.0,     b(2),               bdot(2));
                }


                /**
                 * @brief Constructor.
                 *        Velocities set to non-zero at both ends of the trajectory
                 *
                 * @param[in] a           initial point mass state
                 * @param[in] b           final point mass state
                 * @param[in] step_height height of the step
                 */
                void initialize(const humoto::rigidbody::RigidBodyState& a,
                                const humoto::rigidbody::RigidBodyState& b,
                                const double step_height)
                {
                    HUMOTO_ASSERT(b.position_(2) <= (a.position_(2) + step_height), "final z position greater than step height.")
                    x_.initialize( a.position_(0),               a.velocity_(0), b.position_(0),               b.velocity_(0));
                    y_.initialize( a.position_(1),               a.velocity_(1), b.position_(1),               b.velocity_(1));

                    theta_.initialize( a.rpy_(humoto::AngleIndex::YAW),
                                       a.angular_velocity_(humoto::AngleIndex::YAW), 
                                       b.rpy_(humoto::AngleIndex::YAW),
                                       b.angular_velocity_(humoto::AngleIndex::YAW));
                    
                    z1_.initialize(a.position_(2),               a.velocity_(2), a.position_(2) + step_height, 0.0);
                    z2_.initialize(step_height + a.position_(2), 0.0,            b.position_(2),               b.velocity_(2));
                }


                /**
                 * @brief Evaluate the foot trajectory along x/y/z coordinate.
                 *
                 * @param[in] t time instant
                 * @param[in] trajectory_type type of trajectory
                 * @return    value (position/velocity/acceleration)
                 */
                double evalx(   const double t,
                                const rigidbody::TrajectoryEvaluationType::Type trajectory_type) const
                {
                    return (x_.eval(t, trajectory_type));
                }


                /**
                 * @copydoc Simple3DFootTrajectory::evalx
                 */
                double evaly(   const double t,
                                const rigidbody::TrajectoryEvaluationType::Type trajectory_type) const
                {
                    return (y_.eval(t, trajectory_type));
                }


                /**
                 * @copydoc Simple3DFootTrajectory::evalx
                 */
                double evalz(   const double t,
                                const rigidbody::TrajectoryEvaluationType::Type trajectory_type) const
                {
                    if (t <= 0.5)
                    {
                        return (z1_.eval(t*2, trajectory_type));
                    }
                    else
                    {
                        return (z2_.eval((t-0.5)*2, trajectory_type));
                    }
                }
                
                
                /**
                 * @copydoc Simple3DFootTrajectory::evalx
                 */
                double evalTheta(const double t,
                                 const rigidbody::TrajectoryEvaluationType::Type trajectory_type) const
                {
                    return (theta_.eval(t, trajectory_type));
                }


                /**
                 * @brief Evaluate the trajectory
                 *
                 * @param[in] t time instant
                 * @param[in] trajectory_type type of trajectory
                 *
                 * @return    Eigen matrix with evaluated trajectory samples
                 */
                etools::Vector3 eval(   const double t,
                                        const rigidbody::TrajectoryEvaluationType::Type trajectory_type) const
                {
                    etools::Vector3 result;

                    result(0) = evalx(t,     trajectory_type);
                    result(1) = evaly(t,     trajectory_type);
                    result(2) = evalz(t,     trajectory_type);

                    return (result);
                }


                /**
                 * @brief Evaluate the trajectory
                 *
                 * @param[in,out] foot_state state of the foot at the given time instant
                 * @param[in] t time instant
                 */
                void eval(  humoto::rigidbody::RigidBodyState& foot_state,
                            const double t) const
                {
                    foot_state.position_      = eval(t, rigidbody::TrajectoryEvaluationType::POSITION);
                    foot_state.velocity_      = eval(t, rigidbody::TrajectoryEvaluationType::VELOCITY);
                    foot_state.acceleration_  = eval(t, rigidbody::TrajectoryEvaluationType::ACCELERATION);
                    
                    foot_state.rpy_(humoto::AngleIndex::YAW)                  = evalTheta(t, rigidbody::TrajectoryEvaluationType::POSITION);
                    foot_state.angular_velocity_(humoto::AngleIndex::YAW)     = evalTheta(t, rigidbody::TrajectoryEvaluationType::VELOCITY);
                    foot_state.angular_acceleration_(humoto::AngleIndex::YAW) = evalTheta(t, rigidbody::TrajectoryEvaluationType::ACCELERATION);
                }


            private:
                humoto::rigidbody::CubicPolynomial1D x_;
                humoto::rigidbody::CubicPolynomial1D y_;
                humoto::rigidbody::CubicPolynomial1D z1_;
                humoto::rigidbody::CubicPolynomial1D z2_;
                humoto::rigidbody::CubicPolynomial1D theta_;
        };
    }
}
