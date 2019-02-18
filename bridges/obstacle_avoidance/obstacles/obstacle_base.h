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
    namespace obstacle_avoidance
    {
        /**
         * @brief Abstract base class for obstacles.
         */
        class HUMOTO_LOCAL ObstacleBase : public humoto::rigidbody::RigidBodyState
        {

            #define HUMOTO_CONFIG_SECTION_ID "ObstacleBase"
            #define HUMOTO_CONFIG_CONSTRUCTOR ObstacleBase
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(humoto::rigidbody::RigidBodyState)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS

            public:
                /**
                 * @brief Constructor.
                 *
                 * @param[in] position
                 * @param[in] rpy
                 */
                ObstacleBase(const etools::Vector3& position, 
                             const etools::Vector3& rpy)
                {
                    updateState(position, rpy);
                }
                
                
                /**
                 * @brief Constructor.
                 */
                ObstacleBase()
                {
                }
                
                
                /**
                 * @brief Update state.
                 *
                 * @param[in] obstacle_state
                 */
                void updateState(const humoto::rigidbody::RigidBodyState& obstacle_state)
                {
                    position_     = obstacle_state.position_;
                    velocity_     = obstacle_state.velocity_;
                    acceleration_ = obstacle_state.acceleration_;

                    rpy_                  = obstacle_state.rpy_;
                    angular_velocity_     = obstacle_state.angular_velocity_;
                    angular_acceleration_ = obstacle_state.angular_acceleration_;
                }
                
                
                /**
                 * @brief Update state.
                 *
                 * @param[in] position
                 * @param[in] rpy
                 */
                void updateState(const etools::Vector3& position, 
                                 const etools::Vector3& rpy)
                {
                    humoto::rigidbody::RigidBodyState obstacle_state;
                    obstacle_state.position_ = position;
                    obstacle_state.rpy_      = rpy;

                    updateState(obstacle_state);
                }
                
                
                /**
                 * @brief Get A matrix of linear constraints.
                 *
                 * @return    A matrix
                 */
                const Eigen::MatrixXd& getA() const
                {
                    return(A_);
                }
                
                
                /**
                 * @brief Get bounds of linear constraints.
                 *
                 * @return    bounds
                 */
                const Eigen::VectorXd& getBounds() const 
                {
                    return(ub_);
                }
                
                
                /**
                 * @brief Update linear constraints.
                 *
                 * @param[in] control_problem
                 * @param[in] solution
                 * @param[in] safety_margin
                 */
                virtual void updateConstraints(const humoto::ControlProblem& control_problem,
                                               const humoto::Solution&       solution, 
                                               const double                  safety_margin) = 0;
                
                
                /**
                 * @brief Initialize linear constraints.
                 *
                 * @param[in] control_problem
                 */
                virtual void resetConstraints(const humoto::ControlProblem& control_problem) = 0;
                
                
                /**
                 * @brief Destructor.
                 */
                virtual ~ObstacleBase()
                {
                }


            protected:
                Eigen::MatrixXd A_;
                Eigen::VectorXd ub_;
        };
    }
}
