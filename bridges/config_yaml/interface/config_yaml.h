/**
    @file
    @author Alexander Sherikov
    @author Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief

    @note Headers with the implementation are included in 'config.h'.
*/

#pragma once

#ifdef HUMOTO_DEFINE_CONFIG_CONSTRUCTORS
    #error "This header must be included before humoto.h."
#else
    #define HUMOTO_USE_CONFIG
    #define HUMOTO_USE_YAML_CONFIG
#endif
