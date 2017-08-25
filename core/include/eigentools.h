/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

/**
 * @brief This namespace contains various functions operating on Eigen
 * matrices and vectors.
 */
namespace etools
{
}

#define EIGENTOOLS_ASSERT(condition, message)   HUMOTO_ASSERT(condition, message)
#define EIGENTOOLS_VISIBILITY_ATTRIBUTE         HUMOTO_LOCAL
#define EIGENTOOLS_ENABLE_EIGENTYPE_DETECTION

#include "eigentools/eigentools.h"
#include "eigentools/cross_product.h"
#include "eigentools/blockmatrix_base.h"
#include "eigentools/blockmatrix_kronecker.h"
#include "eigentools/blockmatrix.h"
