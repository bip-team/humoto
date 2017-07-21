/**
    @file
    @author Alexander Sherikov
    @author Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace pepper_ik
    {
        /**
         * @brief Class representing the hierarchy of the problem
         */
        template <int t_features>
            class HUMOTO_LOCAL ConfigurableOptimizationProblem
                :   public humoto::ConfigurableOptimizationProblem
        {
            protected:
                /**
                 * @brief Fill map with all pointers to all tasks for given module
                 */
                humoto::TaskSharedPointer getTask(const std::string &string_id) const
                {
                    if (string_id == "TaskBaseCoMTracking")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_ik::TaskBaseCoMTracking<t_features>));
                    }
                    if (string_id == "TaskBaseOrientation")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_ik::TaskBaseOrientation<t_features>));
                    }
                    if (string_id == "TaskBodyCoMTracking")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_ik::TaskBodyCoMTracking<t_features>));
                    }
                    if (string_id == "TaskFixArms")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_ik::TaskFixArms<t_features>));
                    }
                    if (string_id == "TaskFixHead")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_ik::TaskFixHead<t_features>));
                    }
                    if (string_id == "TaskJointsBounds")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_ik::TaskJointsBounds<t_features>));
                    }
                    if (string_id == "TaskJointsReference")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_ik::TaskJointsReference<t_features>));
                    }
                    if (string_id == "TaskTagOrientation")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_ik::TaskTagOrientation<t_features>));
                    }
                    if (string_id == "TaskTagAngularVelocity")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_ik::TaskTagAngularVelocity<t_features>));
                    }
                    if (string_id == "TaskTagCompleteVelocity")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_ik::TaskTagCompleteVelocity<t_features>));
                    }

                    return(humoto::ConfigurableOptimizationProblem::getTask(string_id));
                }
        };
    }
}
