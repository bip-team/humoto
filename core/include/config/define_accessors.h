/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Inclusion of this file results in generation of functions which
    read and write entries 'HUMOTO_CONFIG_ENTRIES' defined in the including
    header from / to a configuration file.
*/

#ifndef HUMOTO_DOXYGEN_PROCESSING

#ifdef HUMOTO_USE_CONFIG
    #ifndef HUMOTO_CONFIG_ENTRIES
        #error "HUMOTO_CONFIG_ENTRIES is not defined."
    #endif


    #ifdef HUMOTO_CONFIG_SECTION_ID
        HUMOTO_DEFINE_CONFIG_SECTION_ID(HUMOTO_CONFIG_SECTION_ID);
    #endif


    #define HUMOTO_CONFIG_COMPOUND_(entry)       HUMOTO_CONFIG_WRITE_COMPOUND_(entry)
    #define HUMOTO_CONFIG_COMPOUND(entry)        HUMOTO_CONFIG_WRITE_COMPOUND(entry)

    #define HUMOTO_CONFIG_SCALAR_(entry)       HUMOTO_CONFIG_WRITE_SCALAR_(entry)
    #define HUMOTO_CONFIG_SCALAR(entry)        HUMOTO_CONFIG_WRITE_SCALAR(entry)

    #define HUMOTO_CONFIG_ENUM_(entry)         HUMOTO_CONFIG_WRITE_ENUM_(entry)
    #define HUMOTO_CONFIG_ENUM(entry)          HUMOTO_CONFIG_WRITE_ENUM(entry)

    #define HUMOTO_CONFIG_PARENT_CLASS(entry)          HUMOTO_CONFIG_WRITE_PARENT_CLASS(entry)
    #define HUMOTO_CONFIG_MEMBER_CLASS(entry, name)    HUMOTO_CONFIG_WRITE_MEMBER_CLASS(entry, name)

    HUMOTO_DEFINE_CONFIG_WRITER(HUMOTO_CONFIG_ENTRIES);

    #undef HUMOTO_CONFIG_COMPOUND_
    #undef HUMOTO_CONFIG_COMPOUND

    #undef HUMOTO_CONFIG_SCALAR_
    #undef HUMOTO_CONFIG_SCALAR

    #undef HUMOTO_CONFIG_ENUM_
    #undef HUMOTO_CONFIG_ENUM

    #undef HUMOTO_CONFIG_PARENT_CLASS
    #undef HUMOTO_CONFIG_MEMBER_CLASS



    #define HUMOTO_CONFIG_COMPOUND_(entry)       HUMOTO_CONFIG_READ_COMPOUND_(entry)
    #define HUMOTO_CONFIG_COMPOUND(entry)        HUMOTO_CONFIG_READ_COMPOUND(entry)

    #define HUMOTO_CONFIG_SCALAR_(entry)       HUMOTO_CONFIG_READ_SCALAR_(entry)
    #define HUMOTO_CONFIG_SCALAR(entry)        HUMOTO_CONFIG_READ_SCALAR(entry)

    #define HUMOTO_CONFIG_ENUM_(entry)         HUMOTO_CONFIG_READ_ENUM_(entry)
    #define HUMOTO_CONFIG_ENUM(entry)          HUMOTO_CONFIG_READ_ENUM(entry)

    #define HUMOTO_CONFIG_PARENT_CLASS(entry)          HUMOTO_CONFIG_READ_PARENT_CLASS(entry)
    #define HUMOTO_CONFIG_MEMBER_CLASS(entry, name)    HUMOTO_CONFIG_READ_MEMBER_CLASS(entry, name)

    HUMOTO_DEFINE_CONFIG_READER(HUMOTO_CONFIG_ENTRIES);

    #undef HUMOTO_CONFIG_COMPOUND_
    #undef HUMOTO_CONFIG_COMPOUND

    #undef HUMOTO_CONFIG_SCALAR_
    #undef HUMOTO_CONFIG_SCALAR

    #undef HUMOTO_CONFIG_ENUM_
    #undef HUMOTO_CONFIG_ENUM

    #undef HUMOTO_CONFIG_PARENT_CLASS
    #undef HUMOTO_CONFIG_MEMBER_CLASS
#endif //HUMOTO_USE_CONFIG

#undef HUMOTO_CONFIG_SECTION_ID
#undef HUMOTO_CONFIG_ENTRIES
#endif
