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
    /**
     * @brief MPC controller for Pepper
     *
     * @ingroup Modules
     * @{
     * @defgroup pepper_mpc pepper_mpc
     * @}
     *
     * @ingroup pepper_mpc
     */
    namespace pepper_mpc
    {
    }
}

#include "pepper_mpc/common.h"
#include "pepper_mpc/two_point_mass_model.h"
#include "pepper_mpc/model_state.h"
#include "pepper_mpc/model.h"
#include "pepper_mpc/preview_horizon.h"
#include "pepper_mpc/mpc_mg.h"

#include "pepper_mpc/task_basevelocityref.h"
#include "pepper_mpc/task_bodypositionref.h"
#include "pepper_mpc/task_basevelocitybounds.h"
#include "pepper_mpc/task_bodypositionbounds.h"
#include "pepper_mpc/task_coppositionbounds.h"
#include "pepper_mpc/task_baseaccelerationbounds.h"
#include "pepper_mpc/task_basejerkminimization.h"
#include "pepper_mpc/task_bodyjerkminimization.h"
#include "pepper_mpc/task_copcentering.h"
#include "pepper_mpc/task_basepositionref.h"

#ifdef HUMOTO_USE_CONFIG
#include "pepper_mpc/configurable_optimization_problem.h"
#endif
