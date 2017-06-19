/**
    @file
    @author  Don Joven Agravante
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA, 2014-2015 CNRS. Licensed under the Apache
    License, Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once
#include <Eigen/Core>

namespace humoto
{
    namespace wpg03
    {
        /**
         * @brief Possible prediction models of the external wrench
         */
        enum ModelWrenchPrediction
        {
            /// Constant throughout the preview horizon
            MODEL_CONSTANT = 0,

            /// Linearly increasing
            MODEL_LINEAR = 1,

            /// NOT IMPLEMENTED: model for carrying an object solo
            MODEL_SINGLE_CARRY = 2
        };

        /**
         * @brief Reorder the wrench data for ease of use in the MPC
         *
         * @param[in] full_predicted
         *
         * @return n_stepx4 vector with the order: torque_y, force_x, torquex, force_y
         */
        Eigen::VectorXd getReorderedSet(const Eigen::VectorXd &full_predicted)
        {
            Eigen::VectorXd reordered;
            unsigned int n_steps = full_predicted.rows()/6;
            reordered.resize(4*n_steps);
            for(unsigned int i=0; i<n_steps; ++i)
            {
                reordered(i*4+0) = full_predicted(i*6+4); //torque_y
                reordered(i*4+1) = full_predicted(i*6+0); //force_x
                reordered(i*4+2) = full_predicted(i*6+3); //torque_x
                reordered(i*4+3) = full_predicted(i*6+1); //force_y
            }
            return reordered;
        }

        /**
         * @brief Return a prediction where the force remains constant for the preview horizon
         *
         * @param[in] f0        the wrench measured right now
         * @param[in] n_steps   number of preview steps
         *
         * @return n_stepx6 vector of predicted wrenches
         */
        Eigen::VectorXd predictWrenchConstant(const Eigen::VectorXd &f0,
                                              const unsigned int &n_steps)
        {
            Eigen::VectorXd predictedForce;
            predictedForce.resize(6*n_steps);
            for(unsigned int i=0; i<n_steps; ++i)
            {
                predictedForce.segment(i*6, 6) = f0;
            }
            return predictedForce;
        }

        /**
         * @brief Return a prediction of the force based on a linear model in time
         *
         * @param[in] f0         the wrench measured right now
         * @param[in] n_steps    number of preview steps
         * @param[in] slope      slope of the linear model
         * @param[in] time_step  preview window time in seconds
         *
         * @return n_stepx6 vector of predicted wrenches
         */
        Eigen::VectorXd predictWrenchLinear(const Eigen::VectorXd &f0,
                                            const unsigned int &n_steps,
                                            const Eigen::VectorXd &slope,
                                            const double &time_step)
        {
            Eigen::VectorXd predictedForce;
            predictedForce.resize(6*n_steps);
            for(unsigned int i=0; i<n_steps; ++i)
            {
                predictedForce.segment(i*6, 6) = (slope.cwiseProduct(time_step*Eigen::VectorXd::Constant(6, i+1))) + f0;
            }
            return predictedForce;
        }
    }
}
