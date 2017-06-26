set (TGT_QPMAD             "${BRIDGE_TARGET_PREFIX}_${HUMOTO_BRIDGE}")
set (QPMAD_SOURCE_DIR      "${CMAKE_CURRENT_LIST_DIR}/qpmad")
set (QPMAD_INTERFACE_DIR   "${CMAKE_CURRENT_LIST_DIR}/interface")
set (QPMAD_CXX_FLAGS       "-fPIC")


set(QPMAD_ARGS
    ${BRIDGES_ARGS}
    "-DQPMAD_BUILD_TESTS=OFF"
    "-DQPMAD_ENABLE_TRACING=OFF"
    "-DQPMAD_USE_HOUSEHOLDER=ON"
    "-DCMAKE_CXX_FLAGS=${QPMAD_CXX_FLAGS}")

if(HUMOTO_COPY_CXX_FLAGS_TO_C_FLAGS)
    set(QPMAD_ARGS
        ${QPMAD_ARGS}
        "-DCMAKE_C_FLAGS=${QPMAD_CXX_FLAGS}")
endif(HUMOTO_COPY_CXX_FLAGS_TO_C_FLAGS)

humoto_add_external_git_project(TARGET_NAME "${TGT_QPMAD}"
                                PROJECT_DIR "${QPMAD_SOURCE_DIR}"
                                CMAKE_ARGS   ${QPMAD_ARGS})

add_dependencies(fetch-bridges ${TGT_QPMAD}-download)


humoto_copy_file_if_exists( TGT_humoto_copy_headers                     "qpmad.h"
                            "${QPMAD_INTERFACE_DIR}/qpmad.h"  "${HUMOTO_INCLUDE_DIR}")
humoto_copy_dir_if_exists(  TGT_humoto_copy_headers                     "qpmad_dir"
                            "${QPMAD_INTERFACE_DIR}/qpmad"    "${HUMOTO_INCLUDE_DIR}/qpmad")


set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_INCLUDES "" PARENT_SCOPE)
set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_LIBS "" PARENT_SCOPE)
