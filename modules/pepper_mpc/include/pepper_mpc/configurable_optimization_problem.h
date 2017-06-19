/**
    @file
    @author Alexander Sherikov
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
         * @brief Class representing the hierarchy of the problem
         */
        class HUMOTO_LOCAL ConfigurableOptimizationProblem
            :   public humoto::ConfigurableOptimizationProblem
        {
            protected:
                /**
                 * @brief Fill map with all pointers to all tasks for given module
                 */
                humoto::TaskSharedPointer getTask(const std::string &string_id) const
                {
                    if (string_id == "TaskBaseAccelerationBounds")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_mpc::TaskBaseAccelerationBounds));
                    }
                    if (string_id == "TaskBaseJerkMinimization")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_mpc::TaskBaseJerkMinimization));
                    }
                    if (string_id == "TaskBasePositionReference")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_mpc::TaskBasePositionReference));
                    }
                    if (string_id == "TaskBaseVelocityBounds")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_mpc::TaskBaseVelocityBounds));
                    }
                    if (string_id == "TaskBaseVelocityReference")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_mpc::TaskBaseVelocityReference));
                    }
                    if (string_id == "TaskBodyJerkMinimization")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_mpc::TaskBodyJerkMinimization));
                    }
                    if (string_id == "TaskBodyPositionBounds")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_mpc::TaskBodyPositionBounds));
                    }
                    if (string_id == "TaskBodyPositionReference")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_mpc::TaskBodyPositionReference));
                    }
                    if (string_id == "TaskCoPCentering")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_mpc::TaskCoPCentering));
                    }
                    if (string_id == "TaskCoPPositionBounds")
                    {
                        return (humoto::TaskSharedPointer(new humoto::pepper_mpc::TaskCoPPositionBounds));
                    }

                    return(humoto::ConfigurableOptimizationProblem::getTask(string_id));
                }
        };
    }
}
