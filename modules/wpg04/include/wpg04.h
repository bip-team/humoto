/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @brief
*/


#pragma  once

namespace humoto
{
    /**
     * @brief WPG 04: basic walking pattern generator
     *
     * @ingroup Modules
     * @{
     * @defgroup wpg04 wpg04
     * @}
     *
     * @ingroup wpg04
     */
    namespace wpg04
    {
    }
}

// walking-related classes
#include "humoto/walking.h"

#include "wpg04/common.h"
#include "wpg04/model_state.h"
#include "wpg04/model.h"
#include "wpg04/preview_horizon.h"
#include "wpg04/mpc_wpg.h"

#include "wpg04/task_comvelocity.h"
#include "wpg04/task_copbounds.h"
#include "wpg04/task_copposition.h"
#include "wpg04/task_copvelocity.h"
#include "wpg04/task_footstepbounds.h"
#include "wpg04/task_terminalconstraint.h"


#ifdef HUMOTO_USE_CONFIG
#include "wpg04/configurable_optimization_problem.h"
#endif
