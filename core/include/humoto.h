/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "humoto_helpers.h"

#include "config.h"

#include "humoto/solution.h"
#include "humoto/model.h"
#include "humoto/control_problem.h"
#include "humoto/violations.h"
#include "humoto/active_set.h"
#include "humoto/constraints_mixins.h"
#include "humoto/constraints_base.h"
#include "humoto/constraints.h"
#include "humoto/task.h"
#include "humoto/task_generic.h"
#include "humoto/hierarchy_level.h"
#include "humoto/qp_problem.h"
#include "humoto/hierarchy.h"

#include "humoto/solver.h"

#ifdef HUMOTO_USE_CONFIG
#include "humoto/configurable_optimization_problem.h"
#endif

#include "rigid_body.h"
