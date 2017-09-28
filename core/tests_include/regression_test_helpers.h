/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "test_helpers.h"


namespace
{
    std::string     g_ref_filename;
    std::string     g_config_path;
}


namespace humoto_tests
{
    namespace HUMOTO_TEST_MODULE_NAMESPACE
    {
        class TestEnvironment : public testing::Environment
        {
            public:
                TestEnvironment(const std::string & config_path,
                                const std::string & ref_filename)
                {
                    g_ref_filename = ref_filename;
                    g_config_path = config_path;
                }
        };
    }
}


#define HUMOTO_DEFINE_REGRESSION_TEST_MAIN()  \
    int main(int argc, char **argv)\
    {\
        std::string config_path = humoto_tests::getConfigPath(argc, argv, 1);\
        std::string ref_filename = humoto_tests::getReferenceFileName(argc, argv, 2);\
        ::testing::InitGoogleTest(&argc, argv);\
        testing::AddGlobalTestEnvironment(new humoto_tests::HUMOTO_TEST_MODULE_NAMESPACE::TestEnvironment(config_path, ref_filename));\
        return RUN_ALL_TESTS();\
    }
