/**
    @file
    @author  Jan Michalczyk
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <iostream>
#include <limits>
#include <iomanip>

#define HUMOTO_GLOBAL_LOGGER_ENABLED

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
// common & abstract classes
#include "humoto/humoto.h"
// walking-related classes
#include "humoto/walking.h"
// specific control problem (many can be included simultaneously)
#include "humoto/pepper_mpc.h"

//testing
#include "gtest/gtest.h"
#include "utilities_regression.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);


//===================================================
// qpOASES
//===================================================
#ifdef HUMOTO_BRIDGE_qpOASES

#include "humoto/qpoases.h"
#define HUMOTO_TEST_SOLVER_NAMESPACE            qpoases
#include "regression_test_000_body.h"
#endif
//===================================================


//===================================================
// QuadProgpp
//===================================================
#ifdef HUMOTO_BRIDGE_QuadProgpp

#include "humoto/quadprogpp.h"
#define HUMOTO_TEST_SOLVER_NAMESPACE            quadprogpp
#include "regression_test_000_body.h"
#endif
//===================================================

HUMOTO_DEFINE_REGRESSION_TEST_MAIN()
