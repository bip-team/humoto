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
         * @brief Minimize the CoM jerk
         */
        class TaskExtWrench : public humoto::TaskZeroVariables
        {
            public:
                TaskExtWrench (const double gain = 0.0707106781186548) 
                    : TaskZeroVariables(gain, "TaskExtWrench", EXTWRENCH_VARIABLES_ID)
                {
                    // nothing to do
                };
        };
    }
}
