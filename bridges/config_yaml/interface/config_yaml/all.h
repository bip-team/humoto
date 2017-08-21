/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "yaml-cpp/yaml.h"

#include "reader.h"
#include "writer.h"


#define HUMOTO_CONFIG_YAML_METHOD_DECLARATION \
    virtual void writeConfigEntries(humoto::config::yaml::Writer &) const = 0; \
    virtual void readConfigEntries(humoto::config::yaml::Reader &, const bool) = 0;


#define HUMOTO_CONFIG_YAML_METHOD_DEFINITION \
    void writeConfigEntries(humoto::config::yaml::Writer & writer) const \
    { \
        writeConfigEntriesTemplate(writer); \
    } \
    void readConfigEntries(humoto::config::yaml::Reader & reader, const bool crash_flag) \
    {\
        readConfigEntriesTemplate(reader, crash_flag); \
    }
