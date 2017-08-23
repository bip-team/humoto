# General parameters
cmake_minimum_required (VERSION 2.6.3)
project (humoto CXX)

set (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
include(ExternalProject)
include(CMakeParseArguments)
include(humoto_copy_dir_if_exists)
include(humoto_copy_file_if_exists)
include(humoto_add_external_git_project)
include(humoto_add_external_svn_project)
include(humoto_add_external_hg_project)
include(humoto_identify_git_commit)
include(humoto_list_filenames)
include(humoto_dump_variables)



#####################################
## Options (definitions & defaults)
#####################################

# bridges
option (HUMOTO_ENABLE_BRIDGES_BY_DEFAULT    "If true all modules are enabled by default." OFF)
option (HUMOTO_INSTALL_BRIDGES              "Install bridges on the system." OFF)
set    (HUMOTO_BRIDGES_DIRS                 "${PROJECT_SOURCE_DIR}/bridges/" CACHE STRING "Directories to search for bridges.")

# these bridges are distributed with humoto and enabled by default.
option (HUMOTO_ENABLE_QuadProgpp           "Enable support for QuadProgpp."    ON)
option (HUMOTO_ENABLE_qpOASES              "Enable support for qpOASES."       ON)
option (HUMOTO_ENABLE_config_yaml          "Enable YAML config files."         ON)


# modules
option (HUMOTO_ENABLE_MODULES_BY_DEFAULT    "If true all modules are enabled by default." OFF)
set    (HUMOTO_MODULES_DIRS                 "${PROJECT_SOURCE_DIR}/modules/" CACHE STRING "Directories to search for modules.")


# extra dependencies
option (HUMOTO_ENABLE_THREADS_FOR_LOGGING  "Enable threads for logging (Boost threads)."  ON)

# build parameters
option (HUMOTO_VERBOSE                  "Verbose build"                     ON)
option (HUMOTO_BUILD_TESTS              "Build tests."                      ON)
option (HUMOTO_BUILD_REGRESSION_TESTS   "Build regression tests."           ON)
option (HUMOTO_ENABLE_CCACHE            "Try to use ccache"                 ON)
option (HUMOTO_ENABLE_OPENMP            "Try to use OpenMP"                 ON)

option (HUMOTO_BUILD_WITH_QI            "Build is performed using NaoQI"    OFF)
set    (HUMOTO_QI_TOOLCHAIN_DIR         "" CACHE STRING "QI toolchain directory")

#####################################

# Gather all cached options controlling compilation of the modules to unset
# those that are not valid anymore.
get_cmake_property(CACHED_VARIABLES CACHE_VARIABLES)
set(HUMOTO_CACHED_BRIDGES "")
set(HUMOTO_CACHED_MODULES "")
foreach (CACHED_VARIABLE ${CACHED_VARIABLES})
    if (CACHED_VARIABLE MATCHES "HUMOTO_MODULE_.*")
        list(APPEND HUMOTO_CACHED_MODULES ${CACHED_VARIABLE})
    else()
        if (CACHED_VARIABLE MATCHES "HUMOTO_BRIDGE_.*")
            list(APPEND HUMOTO_CACHED_BRIDGES ${CACHED_VARIABLE})
        endif()
    endif()
endforeach(CACHED_VARIABLE)


# Bridges
if(HUMOTO_BUILD_REGRESSION_TESTS)
    set(HUMOTO_BRIDGE_googletest ON)
endif()


set(HUMOTO_BRIDGES "")
foreach (HUMOTO_BRIDGES_DIR ${HUMOTO_BRIDGES_DIRS})
    humoto_list_filenames("${HUMOTO_BRIDGES_DIR}"  "HUMOTO_BRIDGES_TMP" DIRS_WITH_CMAKELISTS)
    list(APPEND HUMOTO_BRIDGES ${HUMOTO_BRIDGES_TMP})

    foreach(HUMOTO_BRIDGE ${HUMOTO_BRIDGES_TMP})
        set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_DIR  "${HUMOTO_BRIDGES_DIR}/${HUMOTO_BRIDGE}")

        list(REMOVE_ITEM HUMOTO_CACHED_BRIDGES "HUMOTO_BRIDGE_${HUMOTO_BRIDGE}")
        if (HUMOTO_BRIDGE_${HUMOTO_BRIDGE})
            option ("HUMOTO_BRIDGE_${HUMOTO_BRIDGE}"
                    "Enable bridge '${HUMOTO_BRIDGE}'" ${HUMOTO_BRIDGE_${HUMOTO_BRIDGE}})
        else()
            option ("HUMOTO_BRIDGE_${HUMOTO_BRIDGE}"
                    "Enable bridge '${HUMOTO_BRIDGE}'" ${HUMOTO_ENABLE_BRIDGES_BY_DEFAULT})
        endif()


        if (HUMOTO_VERBOSE)
            message("----------------------BRIDGE----------------------")
            message("Name: ${HUMOTO_BRIDGE}")
            message("Path: ${HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_DIR}")
            message("Enabled: ${HUMOTO_BRIDGE_${HUMOTO_BRIDGE}}")
            message("Enable/disable option name: HUMOTO_BRIDGE_${HUMOTO_BRIDGE}")
            message("--------------------------------------------------")
        endif(HUMOTO_VERBOSE)
    endforeach(HUMOTO_BRIDGE)
endforeach(HUMOTO_BRIDGES_DIR)


# Modules
set(HUMOTO_MODULES "")
foreach (HUMOTO_MODULES_DIR ${HUMOTO_MODULES_DIRS})
    humoto_list_filenames("${HUMOTO_MODULES_DIR}"  "HUMOTO_MODULES_TMP")
    list(APPEND HUMOTO_MODULES ${HUMOTO_MODULES_TMP})

    foreach(HUMOTO_MODULE ${HUMOTO_MODULES_TMP})
        set(HUMOTO_MODULE_${HUMOTO_MODULE}_DIR  "${HUMOTO_MODULES_DIR}/${HUMOTO_MODULE}")

        list(REMOVE_ITEM HUMOTO_CACHED_MODULES "HUMOTO_MODULE_${HUMOTO_MODULE}")
        if (HUMOTO_MODULE_${HUMOTO_MODULE})
            option ("HUMOTO_MODULE_${HUMOTO_MODULE}"
                    "Enable module '${HUMOTO_MODULE}'" ${HUMOTO_MODULE_${HUMOTO_MODULE}})
        else()
            option ("HUMOTO_MODULE_${HUMOTO_MODULE}"
                    "Enable module '${HUMOTO_MODULE}'" ${HUMOTO_ENABLE_MODULES_BY_DEFAULT})
        endif()


        if (HUMOTO_VERBOSE)
            message("======================MODULE======================")
            message("Name: ${HUMOTO_MODULE}")
            message("Path: ${HUMOTO_MODULE_${HUMOTO_MODULE}_DIR}")
            message("Enabled: ${HUMOTO_MODULE_${HUMOTO_MODULE}}")
            message("Enable/disable option name: HUMOTO_MODULE_${HUMOTO_MODULE}")
            message("==================================================")
        endif(HUMOTO_VERBOSE)
    endforeach(HUMOTO_MODULE)
endforeach(HUMOTO_MODULES_DIR)


foreach (CACHED_VARIABLE "${HUMOTO_CACHED_MODULES}")
    unset ("${CACHED_VARIABLE}" CACHE)
endforeach(CACHED_VARIABLE)



#####################################
## Settings
#####################################

##
## default options
##
if (HUMOTO_VERBOSE)
    set (CMAKE_VERBOSE_MAKEFILE     ON)
endif(HUMOTO_VERBOSE)
set (CMAKE_RULE_MESSAGES        OFF CACHE BOOL "")


##
## paths
##
set (HUMOTO_CORE_DIR "${PROJECT_SOURCE_DIR}/core/")
set (HUMOTO_CONFIG_DIR "${PROJECT_BINARY_DIR}/config/")
set (HUMOTO_INCLUDE_DIR "${PROJECT_BINARY_DIR}/include/humoto")


##
## compilation flags
##

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
if(NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

if (HUMOTO_ENABLE_CCACHE)
    find_program(CCACHE_FOUND ccache)
    if(CCACHE_FOUND)
        set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
        set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    endif(CCACHE_FOUND)
endif(HUMOTO_ENABLE_CCACHE)


set (RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set (LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
set (ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

# AS. 'CMAKE_' prefix makes it default for all targets.
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

# This will go to the configuration file for projects, which use humoto.
set (HUMOTO_OPTIONAL_LIBRARIES "")
set (HUMOTO_OPTIONAL_INCLUDES "")
set (HUMOTO_MANDATORY_LIBRARIES "")
set (HUMOTO_MANDATORY_INCLUDES "")
set (HUMOTO_DEFINES   "")


#####################################
## Dependencies
#####################################

add_custom_target( fetch-bridges )
add_custom_target( TGT_humoto_copy_headers ALL )
add_custom_target( TGT_humoto_copy_config ALL )

##
## Mandatory dependencies
##
if(HUMOTO_BUILD_WITH_QI)
    if (NOT HUMOTO_QI_TOOLCHAIN_DIR)
        message(FATAL_ERROR, "HUMOTO_QI_TOOLCHAIN_DIR must be set.")
    endif()


    # this variable is overriden in external projects
    #set(CMAKE_FIND_ROOT_PATH "${CMAKE_FIND_ROOT_PATH};${HUMOTO_QI_TOOLCHAIN_DIR}/boost/")
    #set(CMAKE_FIND_ROOT_PATH "${CMAKE_FIND_ROOT_PATH};${HUMOTO_QI_TOOLCHAIN_DIR}/eigen3/")
    #        "-DCMAKE_FIND_ROOT_PATH=${CMAKE_FIND_ROOT_PATH}")


    # provide hints for find_package()
    set(Boost_NO_BOOST_CMAKE    ON) # XXX ignore buggy boost-config.cmake
    #set(Boost_DEBUG             ON)
    set(BOOST_ROOT          "${HUMOTO_QI_TOOLCHAIN_DIR}/boost/")
    # XXX all boost libraries have version suffix and cannot be found
    set(CMAKE_FIND_LIBRARY_SUFFIXES .so.1.55.0 ${CMAKE_FIND_LIBRARY_SUFFIXES})

    set(EIGEN3_INCLUDE_DIR  "${HUMOTO_QI_TOOLCHAIN_DIR}/eigen3/include/eigen3/")


    # required for processing of "*config.cmake" corresponding to packages
    find_package(qibuild)


    # Search for Eigen
    find_package(Eigen3 REQUIRED)
    include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

    # Boost
    if(HUMOTO_ENABLE_THREADS_FOR_LOGGING)
        find_package(Boost REQUIRED thread system)
        set(HUMOTO_USE_THREADS_FOR_LOGGING true)
    else()
        find_package(Boost REQUIRED)
    endif(HUMOTO_ENABLE_THREADS_FOR_LOGGING)
    include_directories(SYSTEM ${Boost_INCLUDE_DIRS})


    # XXX Workaround for a bug: crosscompilation toolchain uses C flags instead
    # of CXX.
    set(HUMOTO_COPY_CXX_FLAGS_TO_C_FLAGS true)
else(HUMOTO_BUILD_WITH_QI)
    # Search for Eigen
    find_package(Eigen3 REQUIRED)
    include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

    # Boost
    if(HUMOTO_ENABLE_THREADS_FOR_LOGGING)
        find_package(Boost REQUIRED thread system)
        set(HUMOTO_USE_THREADS_FOR_LOGGING true)
    else()
        find_package(Boost REQUIRED)
    endif(HUMOTO_ENABLE_THREADS_FOR_LOGGING)
    include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
endif(HUMOTO_BUILD_WITH_QI)

list(APPEND HUMOTO_MANDATORY_INCLUDES "${EIGEN3_INCLUDE_DIR}")
list(APPEND HUMOTO_MANDATORY_INCLUDES "${Boost_INCLUDE_DIRS}")

if(HUMOTO_USE_THREADS_FOR_LOGGING)
    list(APPEND HUMOTO_MANDATORY_LIBRARIES  "${Boost_THREAD_LIBRARY}")
    list(APPEND HUMOTO_MANDATORY_LIBRARIES  "${Boost_SYSTEM_LIBRARIES}")
endif(HUMOTO_USE_THREADS_FOR_LOGGING)


##
## Optional dependencies
##

set(BRIDGE_TARGET_PREFIX TGT_bridge)


set (HUMOTO_BRIDGES_BINARY_DIR "${PROJECT_BINARY_DIR}/bridges/")
if (HUMOTO_INSTALL_BRIDGES)
    set(HUMOTO_BRIDGES_INSTALL_PREFIX   "${CMAKE_INSTALL_PREFIX}")
else()
    set(HUMOTO_BRIDGES_INSTALL_PREFIX   "${PROJECT_BINARY_DIR}/bridges_install/")
endif (HUMOTO_INSTALL_BRIDGES)


# TODO We use this to set CMAKE_INSTALL_LIBDIR in order to prevent creation of
# separate directories for different architectures. This is potentially unsafe
# if HUMOTO_INSTALL_BRIDGES = ON.
#
# It seems that the only robust solution is to work with humoto as an external
# project, where we would be able to use find_library().
set(HUMOTO_BRIDGES_LIBDIR       "${HUMOTO_BRIDGES_INSTALL_PREFIX}/lib")

set(HUMOTO_BRIDGES_INCLUDE_DIR  "${HUMOTO_BRIDGES_INSTALL_PREFIX}/include")
include_directories (BEFORE SYSTEM     "${HUMOTO_BRIDGES_INCLUDE_DIR}")


set(BRIDGES_ARGS
    --no-warn-unused-cli
    -DCMAKE_INSTALL_PREFIX=${HUMOTO_BRIDGES_INSTALL_PREFIX}
    -DCMAKE_INSTALL_LIBDIR=${HUMOTO_BRIDGES_LIBDIR}
    -DCMAKE_VERBOSE_MAKEFILE=${CMAKE_VERBOSE_MAKEFILE}
    -DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}
    -DCMAKE_BUILD_TYPE=Release)


if(CMAKE_TOOLCHAIN_FILE)
    if(NOT IS_ABSOLUTE ${CMAKE_TOOLCHAIN_FILE})
        set(CMAKE_TOOLCHAIN_FILE "${PROJECT_SOURCE_DIR}/${CMAKE_TOOLCHAIN_FILE}")
    endif()

    set(BRIDGES_ARGS
        ${BRIDGES_ARGS}
        "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
endif(CMAKE_TOOLCHAIN_FILE)

# naoqi workarounds
if(HUMOTO_BUILD_WITH_QI)
    set(BRIDGES_ARGS
        ${BRIDGES_ARGS}
        "-DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}")
endif(HUMOTO_BUILD_WITH_QI)


# OpenMP
if(HUMOTO_ENABLE_OPENMP)
    IF(POLICY CMP0012)
        # Some versions of FindOpenMP rely on deprecated behavior.
        cmake_policy(SET CMP0012 OLD)
    ENDIF(POLICY CMP0012)

    find_package(OpenMP QUIET)
    if(OPENMP_FOUND)
        set (CMAKE_CXX_FLAGS    "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    else()
        message("OpenMP support was not detected. Proceeding without it.")
    endif(OPENMP_FOUND)

    IF(POLICY CMP0012)
        cmake_policy(SET CMP0012 NEW)
    ENDIF(POLICY CMP0012)
endif(HUMOTO_ENABLE_OPENMP)



# bridges
foreach(HUMOTO_BRIDGES_DIR ${HUMOTO_BRIDGES_DIRS})
    humoto_list_filenames("${HUMOTO_BRIDGES_DIR}"   "HUMOTO_BRIDGES_TMP" DIRS_WITH_CMAKELISTS)

    foreach(HUMOTO_BRIDGE ${HUMOTO_BRIDGES_TMP})
        if (HUMOTO_BRIDGE_${HUMOTO_BRIDGE})
            add_subdirectory("${HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_DIR}" "${HUMOTO_BRIDGES_BINARY_DIR}/${HUMOTO_BRIDGE}")

            if (HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_LIBS)
                list(APPEND HUMOTO_OPTIONAL_LIBRARIES  "${HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_LIBS}")
            endif()

            if (HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_INCLUDES)
                list(APPEND HUMOTO_OPTIONAL_INCLUDES    "${HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_INCLUDES}")
                include_directories("${HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_INCLUDES}")
            endif()
        endif()
    endforeach(HUMOTO_BRIDGE)
endforeach(HUMOTO_BRIDGES_DIR)



#####################################
## Variables / options for humoto
#####################################


set (CMAKE_CXX_FLAGS            "${CMAKE_CXX_FLAGS} -fPIC -pedantic -Wall")
# extra warnings
set (CMAKE_CXX_FLAGS            "${CMAKE_CXX_FLAGS} -Wempty-body -Wignored-qualifiers -Wmissing-field-initializers")
set (CMAKE_CXX_FLAGS            "${CMAKE_CXX_FLAGS} -Wsign-compare -Wtype-limits -Wuninitialized -Woverloaded-virtual")

set (CMAKE_CXX_FLAGS            "${CMAKE_CXX_FLAGS} -std=c++98")


include(TargetArch)
target_architecture(HUMOTO_TARGET_ARCHITECTURE)

if(NOT "${HUMOTO_TARGET_ARCHITECTURE}" STREQUAL "x86_64")
    # Disable alignment of static Eigen matrices on all architectures except
    # x86_64
    set(EIGENTOOLS_DISABLE_ALIGNMENT_OF_STATIC_MATRICES true)
endif()


include (humoto_detect_func_macro)
humoto_detect_func_macro()


if (HUMOTO_USE_CONFIG)
    set(HUMOTO_DEFAULT_CONFIG_PREFIX "${CMAKE_INSTALL_PREFIX}/share/humoto/config/")
endif(HUMOTO_USE_CONFIG)


#####################################
## Humoto headers
#####################################

# Core

humoto_copy_dir_if_exists(TGT_humoto_copy_headers       "core"
                          "${HUMOTO_CORE_DIR}/include"  "${HUMOTO_INCLUDE_DIR}")
humoto_copy_dir_if_exists(TGT_humoto_copy_config        "core"
                          "${HUMOTO_CORE_DIR}/config"   "${HUMOTO_CONFIG_DIR}/core")


# Modules
foreach(HUMOTO_MODULE ${HUMOTO_MODULES})
    if (HUMOTO_MODULE_${HUMOTO_MODULE})
        set(MODULE_DIR "${HUMOTO_MODULE_${HUMOTO_MODULE}_DIR}")

        humoto_copy_dir_if_exists(TGT_humoto_copy_headers       "${HUMOTO_MODULE}"
                                  "${MODULE_DIR}/include"       "${HUMOTO_INCLUDE_DIR}")
        humoto_copy_dir_if_exists(TGT_humoto_copy_config        "${HUMOTO_MODULE}"
                                  "${MODULE_DIR}/config"        "${HUMOTO_CONFIG_DIR}/${HUMOTO_MODULE}")
    endif (HUMOTO_MODULE_${HUMOTO_MODULE})
endforeach(HUMOTO_MODULE)


# All headers are collected in one directory
include_directories ("${PROJECT_BINARY_DIR}/include/")


#####################################
## Tests
#####################################
if (HUMOTO_BUILD_TESTS OR HUMOTO_BUILD_REGRESSION_TESTS)
    enable_testing()

    include(humoto_copy_extra_test_files)
    include(humoto_define_test)
    include(humoto_define_regression_test)
    include(humoto_parse_test_dependencies)

    set (HUMOTO_MODULES_BINARY_DIR "${PROJECT_BINARY_DIR}/modules/")
    if (HUMOTO_BUILD_WITH_QI)
        set (HUMOTO_RUNTIME_OUTPUT_PREFIX   "${CMAKE_BINARY_DIR}/sdk/bin/")
    else(HUMOTO_BUILD_WITH_QI)
        set (HUMOTO_RUNTIME_OUTPUT_PREFIX   "${PROJECT_BINARY_DIR}/tests/")
    endif(HUMOTO_BUILD_WITH_QI)


    if (HUMOTO_ENABLE_MODULES_BY_DEFAULT)
        set (CMAKE_RUNTIME_OUTPUT_DIRECTORY     "${HUMOTO_RUNTIME_OUTPUT_PREFIX}/core/")
        set (HUMOTO_MODULE "core")
        add_custom_target( "${HUMOTO_MODULE}" )
        add_subdirectory ("${HUMOTO_CORE_DIR}/tests")
    endif(HUMOTO_ENABLE_MODULES_BY_DEFAULT)


    foreach(HUMOTO_MODULE ${HUMOTO_MODULES})
        if (HUMOTO_MODULE_${HUMOTO_MODULE})
            if (EXISTS "${HUMOTO_MODULE_${HUMOTO_MODULE}_DIR}/tests")
                set (CMAKE_RUNTIME_OUTPUT_DIRECTORY     "${HUMOTO_RUNTIME_OUTPUT_PREFIX}/${HUMOTO_MODULE}/")
                add_custom_target( "${HUMOTO_MODULE}" )
                add_subdirectory (  "${HUMOTO_MODULE_${HUMOTO_MODULE}_DIR}/tests"
                                    "${HUMOTO_MODULES_BINARY_DIR}/${HUMOTO_MODULE}")
            endif()
        endif (HUMOTO_MODULE_${HUMOTO_MODULE})
    endforeach(HUMOTO_MODULE)
endif()


#####################################
## Configuration files
#####################################

list(APPEND HUMOTO_LIBRARIES "${HUMOTO_OPTIONAL_LIBRARIES}" "${HUMOTO_MANDATORY_LIBRARIES}")
list(APPEND HUMOTO_INCLUDES "${HUMOTO_OPTIONAL_INCLUDES}" "${HUMOTO_MANDATORY_INCLUDES}")
if (HUMOTO_LIBRARIES)
    set (HUMOTO_SHARED_GNU_LINKER_FLAGS "-Wl,--exclude-libs")

    foreach(HUMOTO_LIBRARY ${HUMOTO_LIBRARIES})
        set(HUMOTO_SHARED_GNU_LINKER_FLAGS "${HUMOTO_SHARED_GNU_LINKER_FLAGS},${HUMOTO_LIBRARY}")
    endforeach(HUMOTO_LIBRARY)
endif (HUMOTO_LIBRARIES)

set (HUMOTO_SHARED_GNU_LINKER_FLAGS
    "${HUMOTO_SHARED_GNU_LINKER_FLAGS} -Wl,--version-script=${CMAKE_INSTALL_PREFIX}/share/humoto/gnu_ld_version_script.map")

humoto_identify_git_commit()

configure_file("cmake/HumotoConfig.cmake.in"    "${PROJECT_BINARY_DIR}/HumotoConfig.cmake")


# The following is similar to
#   configure_file("cmake/build_config.h.in"        "${HUMOTO_INCLUDE_DIR}/build_config.h")
# but more flexible -- no need to know all possible variables in advance
#
set(HUMOTO_BUILD_CONFIG
"// __func__ ?
#cmakedefine HUMOTO_COMPILER_SUPPORTS_FUNC_
// __FUNCTION__ ?
#cmakedefine HUMOTO_COMPILER_SUPPORTS_FUNCTION_
//
//#define humoto HUMOTO_CUSTOM_NAMESPACE
//
#cmakedefine HUMOTO_DEFAULT_CONFIG_PREFIX \"${HUMOTO_DEFAULT_CONFIG_PREFIX}\"
//
#cmakedefine HUMOTO_USE_THREADS_FOR_LOGGING
//
#cmakedefine EIGENTOOLS_DISABLE_ALIGNMENT_OF_STATIC_MATRICES
//
#cmakedefine HUMOTO_GIT_COMMIT_INFO \"${HUMOTO_GIT_COMMIT_INFO}\"
//
//
#cmakedefine HUMOTO_BUILD_TESTS
#cmakedefine HUMOTO_BUILD_REGRESSION_TESTS\n")

foreach(HUMOTO_BRIDGE ${HUMOTO_BRIDGES})
    set(HUMOTO_BUILD_CONFIG "${HUMOTO_BUILD_CONFIG}#cmakedefine HUMOTO_BRIDGE_${HUMOTO_BRIDGE}\n")
endforeach(HUMOTO_BRIDGE)

foreach(HUMOTO_MODULE ${HUMOTO_MODULES})
    set(HUMOTO_BUILD_CONFIG "${HUMOTO_BUILD_CONFIG}#cmakedefine HUMOTO_MODULE_${HUMOTO_MODULE}\n")
endforeach(HUMOTO_MODULE)

string(CONFIGURE "${HUMOTO_BUILD_CONFIG}" HUMOTO_BUILD_CONFIG)
file(WRITE "${HUMOTO_INCLUDE_DIR}/build_config.h" "${HUMOTO_BUILD_CONFIG}")



#####################################
## Installation
#####################################

install (DIRECTORY "${HUMOTO_INCLUDE_DIR}"
         DESTINATION "${CMAKE_INSTALL_PREFIX}/include/")

if (HUMOTO_USE_CONFIG)
    install (DIRECTORY "${HUMOTO_CONFIG_DIR}"
             DESTINATION "${CMAKE_INSTALL_PREFIX}/share/humoto/config")
endif(HUMOTO_USE_CONFIG)

install (FILES "${PROJECT_BINARY_DIR}/HumotoConfig.cmake"
         DESTINATION "${CMAKE_INSTALL_PREFIX}/share/humoto/")

install (FILES "${HUMOTO_CORE_DIR}/share/gnu_ld_version_script.map"
         DESTINATION "${CMAKE_INSTALL_PREFIX}/share/humoto/")
