set (TGT_YAMLCPP            "${BRIDGE_TARGET_PREFIX}_${HUMOTO_BRIDGE}")
set (YAMLCPP_SOURCE_DIR     "${CMAKE_CURRENT_LIST_DIR}/yaml-cpp")
set (YAMLCPP_INTERFACE_DIR  "${CMAKE_CURRENT_LIST_DIR}/interface")
set (YAMLCPP_CXX_FLAGS      "-fPIC")

set(YAMLCPP_ARGS
    ${BRIDGES_ARGS}
    -DYAML_CPP_BUILD_TOOLS=OFF
    -DYAML_CPP_BUILD_CONTRIB=OFF
    "-DCMAKE_CXX_FLAGS=${YAMLCPP_CXX_FLAGS}")

if(HUMOTO_COPY_CXX_FLAGS_TO_C_FLAGS)
    set(YAMLCPP_ARGS
        ${YAMLCPP_ARGS}
        "-DCMAKE_C_FLAGS=${YAMLCPP_CXX_FLAGS}")
endif(HUMOTO_COPY_CXX_FLAGS_TO_C_FLAGS)

humoto_add_external_git_project(TARGET_NAME "${TGT_YAMLCPP}"
                                PROJECT_DIR "${YAMLCPP_SOURCE_DIR}"
                                CMAKE_ARGS   ${YAMLCPP_ARGS})

add_dependencies(fetch-bridges ${TGT_YAMLCPP}-download)

humoto_copy_file_if_exists( TGT_humoto_copy_headers                     "config_yaml_h"
                            "${YAMLCPP_INTERFACE_DIR}/config_yaml.h"    "${HUMOTO_INCLUDE_DIR}")
humoto_copy_dir_if_exists(  TGT_humoto_copy_headers                     "config_yaml_dir"
                            "${YAMLCPP_INTERFACE_DIR}/config_yaml"      "${HUMOTO_INCLUDE_DIR}/config_yaml")


set(HUMOTO_USE_CONFIG ON PARENT_SCOPE)
set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_INCLUDES "" PARENT_SCOPE)
set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_LIBS
    "${HUMOTO_BRIDGES_LIBDIR}/${CMAKE_SHARED_MODULE_PREFIX}yaml-cpp${CMAKE_STATIC_LIBRARY_SUFFIX}" PARENT_SCOPE)
