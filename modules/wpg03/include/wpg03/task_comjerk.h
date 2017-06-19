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
        class TaskCoMJerk : public humoto::TaskZeroVariables
        {
            public:
                TaskCoMJerk(const double gain = 7.07106781186548e-04) 
                    : TaskZeroVariables(gain, "TaskCoMJerk", COMJERK_VARIABLES_ID)
                {
                    // nothing to do
                };
        };
    }
}
