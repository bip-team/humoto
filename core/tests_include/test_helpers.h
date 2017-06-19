/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#ifndef HUMOTO_TEST_MODULE_NAME
#error "You must define HUMOTO_TEST_MODULE_NAME before including this header."
#endif

#ifndef HUMOTO_TEST_MODULE_NAMESPACE
#define HUMOTO_TEST_MODULE_NAMESPACE    HUMOTO_TEST_MODULE_NAME
#endif


namespace humoto_tests
{

#define HUMOTO_STRING_NAME_CATENATOR(prefix, id, suffix)   prefix #id suffix
#define HUMOTO_STRING_NAME_GENERATOR(prefix, id, suffix)   HUMOTO_STRING_NAME_CATENATOR(prefix, id, suffix)

#define HUMOTO_NAME_CATENATOR(prefix, suffix)   prefix##suffix
#define HUMOTO_NAME_GENERATOR(prefix, suffix)   HUMOTO_NAME_CATENATOR(prefix, suffix)


    std::string getConfigPath(  const int argc,
                                char ** const argv,
                                const int parameter_num = 1,
                                const char *default_path =
                                    HUMOTO_STRING_NAME_GENERATOR("../../config/", HUMOTO_TEST_MODULE_NAME, "/") )
    {
        std::string     config_path;

        if ((parameter_num < argc) && (parameter_num > 0))
        {
            //path to search for config files
            config_path = std::string(argv[parameter_num]);
        }
        else
        {
            //default config path
            config_path = default_path;
        }

        return (config_path);
    }


    std::string getReferenceFileName(   const int argc,
                                        char ** const argv,
                                        const int parameter_num = 1)
    {
        std::string     log_file_name;

        if ((parameter_num < argc) && (parameter_num > 0))
        {
            // Name of the output file is provided, running in batch mode.
            log_file_name = argv[parameter_num];
        }
        else
        {
            log_file_name = std::string(argv[0]) + ".ref";
        }

        return (log_file_name);
    }


    std::string getLogFileName( const int argc,
                                char ** const argv,
                                const int parameter_num = 1)
    {
        std::string     log_file_name;

        if ((parameter_num < argc) && (parameter_num > 0))
        {
            // Name of the output file is provided, running in batch mode.
            log_file_name = argv[parameter_num];
        }
        else
        {
            log_file_name = std::string(argv[0]) + ".m";
        }

        return (log_file_name);
    }
}
