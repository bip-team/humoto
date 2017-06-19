/**
    @file
    @author Jan Michalczyk
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <iostream>
#include <limits>
#include <iomanip>

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
// common & abstract classes
#include "humoto/humoto.h"
// specific control problem (many can be included simultaneously)
#include "humoto/pepper_mpc.h"


#include "utilities.h"

#define HUMOTO_TEST_HIERARCHY_SETUP_FUNCTION    setupHierarchy_v0


//===================================================
// qpOASES
//===================================================
#ifdef HUMOTO_BRIDGE_qpOASES

#include "humoto/qpoases.h"
#define HUMOTO_TEST_SOLVER_NAMESPACE qpoases
#include "test_001_body.h"

#endif
//===================================================


//===================================================
// LexLS
//===================================================
#ifdef HUMOTO_BRIDGE_lexls

#include "humoto/lexls.h"
#define HUMOTO_TEST_SOLVER_NAMESPACE            lexls
#include "test_001_body.h"

#endif
//===================================================


//===================================================
// eiQuadProg
//===================================================
#ifdef HUMOTO_BRIDGE_eiquadprog
// specific solver (many can be included simultaneously)
#include "humoto/eiquadprog.h"

#define HUMOTO_TEST_SOLVER_NAMESPACE            eiquadprog
#define HUMOTO_TEST_DISABLE_HOT_STARTING        1
#include "test_001_body.h"

#endif
//===================================================


//===================================================
// QuadProgpp
//===================================================
#ifdef HUMOTO_BRIDGE_QuadProgpp
// specific solver (many can be included simultaneously)
#include "humoto/quadprogpp.h"

#define HUMOTO_TEST_SOLVER_NAMESPACE            quadprogpp
#define HUMOTO_TEST_DISABLE_HOT_STARTING        1
#include "test_001_body.h"

#endif
//===================================================



/**
 * @brief main
 *
 * @param[in] argc number of args
 * @param[in] argv args
 *
 * @return status
 */
int main(int argc, char **argv)
{
    std::size_t n_of_simulations = 20;

    std::string config_path = humoto_tests::getConfigPath(argc, argv, 2);

    if(argc == 2)
    {
        std::stringstream sstream(argv[1]);
        sstream >> n_of_simulations;

        HUMOTO_ASSERT(n_of_simulations > 0, "Number of simulation must be greater than zero.");
    }

    try
    {
        #ifdef HUMOTO_BRIDGE_qpOASES
        humoto_tests::pepper_mpc::qpoases::run(n_of_simulations, config_path);
        #endif

        #ifdef HUMOTO_BRIDGE_lexls
        humoto_tests::pepper_mpc::lexls::run(n_of_simulations, config_path);
        #endif

        #ifdef HUMOTO_BRIDGE_eiquadprog
        humoto_tests::pepper_mpc::eiquadprog::run(n_of_simulations, config_path);
        #endif

        #ifdef HUMOTO_BRIDGE_QuadProgpp
        humoto_tests::pepper_mpc::quadprogpp::run(n_of_simulations, config_path);
        #endif
    }
    catch (std::exception & e)
    {
        std::cout << e.what() << std::endl;
    }
}
