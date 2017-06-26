/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
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
#include "humoto/humoto.h"
#include "humoto/wpg04.h"

//testing
#include "gtest/gtest.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);
//#undef HUMOTO_GLOBAL_LOGGER_ENABLED


#include "utilities_regression.h"

#define HUMOTO_TEST_HIERARCHY_ID    "Hierarchy00"

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
// LexLS
//===================================================
#ifdef HUMOTO_BRIDGE_lexls

#include "humoto/lexls.h"
#define HUMOTO_TEST_SOLVER_NAMESPACE            lexls
#include "regression_test_000_body.h"
#endif
//===================================================


//===================================================
// eiQuadProg
//===================================================
#ifdef HUMOTO_BRIDGE_eiquadprog

#include "humoto/eiquadprog.h"
#define HUMOTO_TEST_SOLVER_NAMESPACE            eiquadprog
#define HUMOTO_TEST_DISABLE_HOT_STARTING        1
#include "regression_test_000_body.h"
#endif
//===================================================

//===================================================
// QuadProgpp
//===================================================
#ifdef HUMOTO_BRIDGE_QuadProgpp

#include "humoto/quadprogpp.h"
#define HUMOTO_TEST_SOLVER_NAMESPACE            quadprogpp
#define HUMOTO_TEST_DISABLE_HOT_STARTING        1
#include "regression_test_000_body.h"
#endif
//===================================================

//===================================================
// qpmad
//===================================================
#ifdef HUMOTO_BRIDGE_qpmad

#include "humoto/qpmad.h"
#define HUMOTO_TEST_SOLVER_NAMESPACE            qpmad
#define HUMOTO_TEST_DISABLE_HOT_STARTING        1
#include "regression_test_000_body.h"
#endif
//===================================================


HUMOTO_DEFINE_REGRESSION_TEST_MAIN()
