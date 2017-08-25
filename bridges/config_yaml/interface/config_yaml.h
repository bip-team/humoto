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
    #error "This header must be included before humoto.h"
#else

    #include "humoto_helpers.h"

    #include "yaml-cpp/yaml.h"

    #include "config_yaml/reader.h"
    #include "config_yaml/writer.h"


    namespace humoto
    {
        namespace config
        {
            /**
             * @brief YAML bridge namespace.
             */
            namespace yaml
            {
                class ConfigurableBase
                #ifdef HUMOTO_CONFIG_CONFIGURABLE_BASE_PARENT
                    : public HUMOTO_CONFIG_CONFIGURABLE_BASE_PARENT
                #endif
                {
                    protected:
                        #ifdef HUMOTO_CONFIG_CONFIGURABLE_BASE_PARENT
                            using HUMOTO_CONFIG_CONFIGURABLE_BASE_PARENT::writeConfigEntries;
                            using HUMOTO_CONFIG_CONFIGURABLE_BASE_PARENT::readConfigEntries;
                        #endif

                        ///@{
                        /**
                         * @attention Implementations of these methods are added
                         * automatically upon inclusion of define_accessors.h.
                         */
                        virtual void writeConfigEntries(humoto::config::yaml::Writer &) const = 0;
                        virtual void readConfigEntries(humoto::config::yaml::Reader &, const bool) = 0;
                        ///@}
                };

                #ifdef HUMOTO_CONFIG_CONFIGURABLE_BASE_PARENT
                    #undef HUMOTO_CONFIG_CONFIGURABLE_BASE_PARENT
                #endif
                #define HUMOTO_CONFIG_CONFIGURABLE_BASE_PARENT humoto::config::yaml::ConfigurableBase
            }
        }
    }

    #define HUMOTO_BRIDGE_config_yaml_DEFINITIONS \
        protected: \
            void writeConfigEntries(humoto::config::yaml::Writer & writer) const \
            { \
                writeConfigEntriesTemplate(writer); \
            }\
            void readConfigEntries(humoto::config::yaml::Reader & reader, const bool crash_flag)\
            {\
                readConfigEntriesTemplate(reader, crash_flag);\
            }


    #ifndef HUMOTO_USE_CONFIG
        #define HUMOTO_USE_CONFIG
    #endif
#endif
