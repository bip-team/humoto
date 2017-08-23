set (TGT_QUADPROGPP             "${BRIDGE_TARGET_PREFIX}_${HUMOTO_BRIDGE}")
set (QUADPROGPP_SOURCE_DIR      "${CMAKE_CURRENT_LIST_DIR}/QuadProgpp")
set (QUADPROGPP_INTERFACE_DIR   "${CMAKE_CURRENT_LIST_DIR}/interface")
set (QUADPROGPP_CXX_FLAGS       "-fPIC")


set(QUADPROGPP_ARGS
    ${BRIDGES_ARGS}
    "-DEIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}"
    "-DQUADPROGPP_BUILD_EXAMPLE=OFF"
    "-DQUADPROGPP_ENABLE_TRACING=OFF"
    "-DQUADPROGPP_ENABLE_EIGEN=ON"
    "-DCMAKE_CXX_FLAGS=${QUADPROGPP_CXX_FLAGS}")

if(HUMOTO_COPY_CXX_FLAGS_TO_C_FLAGS)
    set(QUADPROGPP_ARGS
        ${QUADPROGPP_ARGS}
        "-DCMAKE_C_FLAGS=${QUADPROGPP_CXX_FLAGS}")
endif(HUMOTO_COPY_CXX_FLAGS_TO_C_FLAGS)

humoto_add_external_git_project(TARGET_NAME "${TGT_QUADPROGPP}"
                                PROJECT_DIR "${QUADPROGPP_SOURCE_DIR}"
                                CMAKE_ARGS   ${QUADPROGPP_ARGS})

add_dependencies(fetch-bridges ${TGT_QUADPROGPP}-download)


humoto_copy_file_if_exists( TGT_humoto_copy_headers                     "quadprogpp_h"
                            "${QUADPROGPP_INTERFACE_DIR}/quadprogpp.h"  "${HUMOTO_INCLUDE_DIR}")
humoto_copy_dir_if_exists(  TGT_humoto_copy_headers                     "quadprogpp_dir"
                            "${QUADPROGPP_INTERFACE_DIR}/quadprogpp"    "${HUMOTO_INCLUDE_DIR}/quadprogpp")


set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_INCLUDES "" PARENT_SCOPE)
set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_LIBS
    "${HUMOTO_BRIDGES_LIBDIR}/${CMAKE_SHARED_MODULE_PREFIX}quadprog${CMAKE_STATIC_LIBRARY_SUFFIX}" PARENT_SCOPE)
