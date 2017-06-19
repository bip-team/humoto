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

// common & abstract classes (must be first)
#include "humoto/humoto.h"
// specific control problem (many can be included simultaneously)
#include "humoto/wpg04.h"


#include "utilities.h"


#define HUMOTO_TEST_HIERARCHY_SETUP_FUNCTION    setupHierarchy_v0


//===================================================
// qpOASES
//===================================================
#ifdef HUMOTO_BRIDGE_qpOASES

#include "humoto/qpoases.h"
#define HUMOTO_TEST_SOLVER_NAMESPACE            qpoases
#include "test_006_body.h"

#endif
//===================================================


//===================================================
// LexLS
//===================================================
#ifdef HUMOTO_BRIDGE_lexls

#include "humoto/lexls.h"
#define HUMOTO_TEST_SOLVER_NAMESPACE            lexls
#include "test_006_body.h"

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

    if(argc == 2)
    {
        std::stringstream sstream(argv[1]);
        sstream >> n_of_simulations;

        HUMOTO_ASSERT(n_of_simulations > 0, "Number of simulation must be greater than zero.");
    }

#ifdef HUMOTO_BRIDGE_qpOASES
    humoto_tests::wpg04::qpoases::run(n_of_simulations, "");
#endif

#ifdef HUMOTO_BRIDGE_lexls
    humoto_tests::wpg04::lexls::run(n_of_simulations, "");
#endif
}
