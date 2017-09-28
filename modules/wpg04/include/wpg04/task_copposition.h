/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace wpg04
    {
        /**
         * @brief [task_cop.m]
         */
        class HUMOTO_LOCAL TaskCoPPosition : public humoto::TaskZeroVariables
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskZeroVariables)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            protected:
                void setDefaults()
                {
                    TaskZeroVariables::setDefaults();
                    TaskZeroVariables::setVariablesID(COP_VARIABLES_ID);
                    setGain(12.2474487139159);
                }


                void finalize()
                {
                    TaskZeroVariables::finalize();
                }


            public:
                explicit TaskCoPPosition(const double gain = 12.2474487139159)
                    : TaskZeroVariables(gain, "TaskCoPPosition", COP_VARIABLES_ID)
                {
                    // nothing to do
                };
        };
    }
}
