set (TGT_QPOASES            "${BRIDGE_TARGET_PREFIX}_${HUMOTO_BRIDGE}")
set (QPOASES_SOURCE_DIR     "${CMAKE_CURRENT_LIST_DIR}/qpOASES")
set (QPOASES_INTERFACE_DIR  "${CMAKE_CURRENT_LIST_DIR}/interface")
set (QPOASES_CXX_FLAGS      "-fPIC -D__SUPPRESSANYOUTPUT__")

set(QPOASES_ARGS
    ${BRIDGES_ARGS}
    "-DQPOASES_BUILD_EXAMPLES=OFF"
    "-DCMAKE_CXX_FLAGS=${QPOASES_CXX_FLAGS}"
    "-DCMAKE_CXX_FLAGS_DEBUG=-fPIC")

if(HUMOTO_COPY_CXX_FLAGS_TO_C_FLAGS)
    set(QPOASES_ARGS
        ${QPOASES_ARGS}
        "-DCMAKE_C_FLAGS=${QPOASES_CXX_FLAGS}")
endif(HUMOTO_COPY_CXX_FLAGS_TO_C_FLAGS)

humoto_add_external_svn_project(TARGET_NAME "${TGT_QPOASES}"
                                PROJECT_DIR "${QPOASES_SOURCE_DIR}"
                                CMAKE_ARGS   ${QPOASES_ARGS}
                                REPOSITORY  "https://projects.coin-or.org/svn/qpOASES/stable/3.2")

add_dependencies(fetch-bridges ${TGT_QPOASES}-download)

humoto_copy_file_if_exists( TGT_humoto_copy_headers                 "qpoases_h"
                            "${QPOASES_INTERFACE_DIR}/qpoases.h"    "${HUMOTO_INCLUDE_DIR}")
humoto_copy_dir_if_exists(  TGT_humoto_copy_headers                 "qpoases_dir"
                            "${QPOASES_INTERFACE_DIR}/qpoases"      "${HUMOTO_INCLUDE_DIR}/qpoases")


set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_INCLUDES "" PARENT_SCOPE)
set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_LIBS "${HUMOTO_BRIDGES_LIBDIR}/${CMAKE_SHARED_MODULE_PREFIX}qpOASES${CMAKE_STATIC_LIBRARY_SUFFIX}" PARENT_SCOPE)
