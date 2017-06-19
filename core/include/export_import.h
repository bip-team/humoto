/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief based on the example taken from https://gcc.gnu.org/wiki/Visibility
*/


#ifdef HUMOTO_DOXYGEN_PROCESSING
    // enter this branch only during the documentation generation

    /**
     * This macro sets visibility parameters of the classes that should be
     * exported from the compiled library.
     * This macro is defined automatically.
     */
    #define HUMOTO_API

    /**
     * This macro sets visibility parameters of the classes that should NOT be
     * exported from the compiled library.
     * This macro is defined automatically.
     */
    #define HUMOTO_LOCAL

    /**
     * This should be defined if humoto is compiled in a shared library to hide
     * classes defied with HUMOTO_LOCAL.
     */
    #define HUMOTO_COMPILE_SHARED_LIB

    /**
     * This should be defined if some sources are compiled using a library
     * containing humoto. This should be neccessary on WIN platforms, which we
     * do not support currently.
     */
    #define HUMOTO_IMPORT_LIB

#else

    // helper macro depending on the compiler
    #if defined _WIN32 || defined __CYGWIN__
        #define HUMOTO_LIB_IMPORT __declspec(dllimport)
        #define HUMOTO_LIB_EXPORT __declspec(dllexport)
        #define HUMOTO_LIB_LOCAL
    #else
        #if __GNUC__ >= 4
            #define HUMOTO_LIB_IMPORT __attribute__ ((visibility ("default")))
            #define HUMOTO_LIB_EXPORT __attribute__ ((visibility ("default")))
            #define HUMOTO_LIB_LOCAL  __attribute__ ((visibility ("hidden")))
        #else
            #define HUMOTO_LIB_IMPORT
            #define HUMOTO_LIB_EXPORT
            #define HUMOTO_LIB_LOCAL
        #endif
    #endif

    #ifdef HUMOTO_COMPILE_SHARED_LIB
        // compiled as a shared library (the default)
        #define HUMOTO_LOCAL HUMOTO_LIB_LOCAL

        #ifdef HUMOTO_IMPORT_LIB
            // this apparently makes sense only in WIN
            #define HUMOTO_API HUMOTO_LIB_IMPORT
        #else
            #define HUMOTO_API HUMOTO_LIB_EXPORT
        #endif
    #else
        // compiled as a static library
        #define HUMOTO_API
        #define HUMOTO_LOCAL
    #endif

#endif // HUMOTO_DOXYGEN_PROCESSING
