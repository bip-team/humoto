/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "msgpack.hpp"

#define HUMOTO_CONFIG_MSGPACK_NAMESPACE     humoto::config::msgpack

namespace humoto
{
    namespace config
    {
        namespace msgpack
        {
            class ReaderData
            {
                public:
                    std::vector< boost::shared_ptr< ::msgpack::object_handle > > handles_;
            };
        }
    }
}

#define HUMOTO_CONFIG_MSGPACK_PRIVATE_DATA  HUMOTO_CONFIG_MSGPACK_NAMESPACE::ReaderData     msgpack_reader_data_;

#include "reader.h"
#include "writer.h"
