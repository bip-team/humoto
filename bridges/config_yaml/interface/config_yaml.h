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

#ifdef HUMOTO_CONFIG_DISABLED
    #error "This header must be included before humoto.h."
#else
    #ifdef HUMOTO_USE_CONFIG_YAML
        #error "YAML config is already in use."
    #else
        #define HUMOTO_USE_CONFIG_YAML
    #endif


    #ifndef HUMOTO_USE_CONFIG
        #define HUMOTO_USE_CONFIG
    #endif


    // We do not inlude headers here since they depend on humoto stuff, which
    // is not included yet.
    #define HUMOTO_CONFIG_YAML_HEADER   "config_yaml/all.h"
#endif
