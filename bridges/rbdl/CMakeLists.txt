set (TGT_RBDL            "${BRIDGE_TARGET_PREFIX}_${HUMOTO_BRIDGE}")
set (RBDL_SOURCE_DIR     "${CMAKE_CURRENT_LIST_DIR}/rbdl")
set (RBDL_INTERFACE_DIR  "${CMAKE_CURRENT_LIST_DIR}/interface")
set (RBDL_CXX_FLAGS      "-fPIC")

set(RBDL_ARGS
    ${BRIDGES_ARGS}
    "-DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}"
    "-DCMAKE_CXX_FLAGS_DEBUG=${RBDL_CXX_FLAGS}"
    "-DRBDL_BUILD_STATIC=ON"
    "-DRBDL_BUILD_ADDON_URDFREADER=ON"
    "-DRBDL_USE_ROS_URDF_LIBRARY=OFF"
    "-DRBDL_BUILD_ADDON_GEOMETRY=OFF"
    "-DRBDL_BUILD_ADDON_MUSCLE=OFF")

#    humoto_add_external_hg_project( TARGET_NAME "${TGT_RBDL}"
#                                    PROJECT_DIR "${RBDL_SOURCE_DIR}"
#                                    CMAKE_ARGS   ${RBDL_ARGS}
#                                    REPOSITORY  "https://bitbucket.org/rbdl/rbdl"
#                                    TAG         "d5550334c3d1")
humoto_add_external_git_project(TARGET_NAME "${TGT_RBDL}"
                                PROJECT_DIR "${RBDL_SOURCE_DIR}"
                                CMAKE_ARGS   ${RBDL_ARGS})

add_dependencies(fetch-bridges ${TGT_RBDL}-download)

humoto_copy_file_if_exists( TGT_humoto_copy_headers         "rbdl_h"
                            "${RBDL_INTERFACE_DIR}/rbdl.h"  "${HUMOTO_INCLUDE_DIR}")
humoto_copy_dir_if_exists(  TGT_humoto_copy_headers         "rbdl_dir"
                            "${RBDL_INTERFACE_DIR}/rbdl"    "${HUMOTO_INCLUDE_DIR}/rbdl")


set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_INCLUDES "" PARENT_SCOPE)
set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_LIBS    
    "${HUMOTO_BRIDGES_LIBDIR}/${CMAKE_SHARED_MODULE_PREFIX}rbdl${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${HUMOTO_BRIDGES_LIBDIR}/${CMAKE_SHARED_MODULE_PREFIX}rbdl_urdfreader${CMAKE_STATIC_LIBRARY_SUFFIX}" PARENT_SCOPE)
