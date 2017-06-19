/**
    @file
    @author  Alexander Sherikov
    @author  Don Joven Agravante
    @copyright 2014-2017 INRIA, 2014-2015 CNRS. Licensed under the Apache
    License, Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#pragma once

namespace humoto
{
    /**
     * @brief WPG 03: walking pattern generator with external force
     *
     * @ingroup Modules
     * @{
     * @defgroup wpg03 wpg03
     * @}
     *
     * @ingroup wpg03
     */
    namespace wpg03
    {
    }
}

#include "humoto/walking.h"

#include "wpg03/common.h"

#include "wpg03/force_forecast.h"
#include "wpg03/model.h"
#include "wpg03/walk_finite_state_machine.h"
#include "wpg03/preview_horizon.h"
#include "wpg03/mpc_wpg.h"


#include "wpg03/task_comvelocity.h"
#include "wpg03/task_copbounds.h"
#include "wpg03/task_copposition.h"
#include "wpg03/task_comjerk.h"
#include "wpg03/task_footstepbounds.h"
#include "wpg03/task_comtrajectory.h"
#include "wpg03/task_comimpedance.h"
#include "wpg03/task_extwrench.h"
#include "wpg03/task_extwrenchbounds.h"
