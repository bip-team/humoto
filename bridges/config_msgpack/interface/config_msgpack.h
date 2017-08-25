/**
    @file
    @author Alexander Sherikov
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

    #include "msgpack.hpp"

    #include "config_msgpack/reader.h"
    #include "config_msgpack/writer.h"


    namespace humoto
    {
        namespace config
        {
            /**
             * @brief MessagePack bridge namespace.
             */
            namespace msgpack
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
                        virtual void writeConfigEntries(humoto::config::msgpack::Writer &) const = 0;
                        virtual void readConfigEntries(humoto::config::msgpack::Reader &, const bool) = 0;
                        ///@}
                };

                #ifdef HUMOTO_CONFIG_CONFIGURABLE_BASE_PARENT
                    #undef HUMOTO_CONFIG_CONFIGURABLE_BASE_PARENT
                #endif
                #define HUMOTO_CONFIG_CONFIGURABLE_BASE_PARENT humoto::config::msgpack::ConfigurableBase
            }
        }
    }

    #define HUMOTO_BRIDGE_config_msgpack_DEFINITIONS \
        protected: \
            void writeConfigEntries(humoto::config::msgpack::Writer & writer) const \
            { \
                writeConfigEntriesTemplate(writer); \
            }\
            void readConfigEntries(humoto::config::msgpack::Reader & reader, const bool crash_flag)\
            {\
                readConfigEntriesTemplate(reader, crash_flag);\
            }

    #ifndef HUMOTO_USE_CONFIG
        #define HUMOTO_USE_CONFIG
    #endif

#endif
