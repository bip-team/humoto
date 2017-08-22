/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "build_config.h"
#include "export_import.h"

#include <algorithm>
#include <vector>
#include <map>
#include <stack>
#include <queue>
#include <list>
#include <string>
#include <string>
#include <cmath>
#include <utility>
#include <limits>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sys/time.h>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

#ifdef HUMOTO_USE_THREADS_FOR_LOGGING
#include <boost/thread.hpp>
#endif

#ifdef DNDEBUG
#else
#define EIGEN_INITIALIZE_MATRICES_BY_NAN
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>


//=============================================================================
// Configuration
//=============================================================================

// The defines, which are disabled by default should be kept here.
#ifdef HUMOTO_DOXYGEN_PROCESSING

/// Define this to enable logging
#define HUMOTO_GLOBAL_LOGGER_ENABLED

#endif // HUMOTO_DOXYGEN_PROCESSING

//=============================================================================


/**
 * @brief HUMOTO_THROW_MSG throws an error message concatenated with the name
 * of the function (if supported).
 */
#ifdef HUMOTO_COMPILER_SUPPORTS_FUNC_
    #define HUMOTO_THROW_MSG(s) throw std::runtime_error(std::string("In ") + __func__ + "() // " + (s))
#else
    #ifdef HUMOTO_COMPILER_SUPPORTS_FUNCTION_
        #define HUMOTO_THROW_MSG(s) throw std::runtime_error(std::string("In ") + __FUNCTION__ + "() // " + (s))
    #else
        #define HUMOTO_THROW_MSG(s) throw std::runtime_error(s)
    #endif
#endif // HUMOTO_COMPILER_SUPPORTS_FUNC_



#ifdef DNDEBUG
#define HUMOTO_ASSERT(condition, message)
#else
#define HUMOTO_ASSERT(condition, message) if (!(condition)) {HUMOTO_THROW_MSG(message);};
#endif


#define HUMOTO_MACRO_SUBSTITUTE(arg) arg


/**
 * @brief The root namespace of HuMoTo.
 */
namespace humoto
{
}

#include "eigentools.h"

#include "humoto/logger.h"

#include "humoto/constants.h"
#include "humoto/utility.h"
#include "humoto/leftright.h"

#include "humoto/time.h"

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
#include "humoto/optimization_problem.h"

#include "humoto/solver.h"

#ifdef HUMOTO_USE_CONFIG
#include "humoto/configurable_optimization_problem.h"
#endif

#include "rigid_body.h"
