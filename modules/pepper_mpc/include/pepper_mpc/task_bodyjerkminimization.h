/**
    @file
    @author  Alexander Sherikov
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
         * @brief [task_cop.m]
         */
        class HUMOTO_LOCAL TaskBodyJerkMinimization: public humoto::TaskZeroVariables
        {
            public:
                explicit TaskBodyJerkMinimization(const double gain = 0.707106781186548) 
                    : TaskZeroVariables(gain, "TaskBodyJerkMinimization", BODY_JERK_VARIABLES_ID)
                {
                    // nothing to do
                }
        };
    }
}
