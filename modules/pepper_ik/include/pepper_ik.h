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
     * @brief Inverse kinematics controller for Pepper
     *
     * @ingroup Modules
     * @{
     * @defgroup pepper_ik pepper_ik
     * @}
     *
     * @ingroup pepper_ik
     */
    namespace pepper_ik
    {
    }
}

#include "humoto/rbdl.h"

#include "pepper_ik/common.h"
#include "pepper_ik/model_description.h"
#include "pepper_ik/generalized_coordinates.h"
#include "pepper_ik/model.h"
#include "pepper_ik/whole_body_controller.h"

#include "pepper_ik/task_basecom.h"
#include "pepper_ik/task_bodycom.h"
#include "pepper_ik/task_jointsref.h"
#include "pepper_ik/task_baseorient.h"
#include "pepper_ik/task_fixarms.h"
#include "pepper_ik/task_fixhead.h"
#include "pepper_ik/task_jointsbounds.h"
#include "pepper_ik/task_tagorient.h"
#include "pepper_ik/task_tagangularvel.h"
#include "pepper_ik/task_tagcompletevel.h"
#include "pepper_ik/task_tagpose.h"
#include "pepper_ik/task_tagpose3dof.h"

#ifdef HUMOTO_USE_CONFIG
#include "pepper_ik/configurable_optimization_problem.h"
#endif
