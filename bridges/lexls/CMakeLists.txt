set (TGT_LEXLS            "${BRIDGE_TARGET_PREFIX}_${HUMOTO_BRIDGE}")
set (LEXLS_SOURCE_DIR     "${CMAKE_CURRENT_LIST_DIR}/lexls")
set (LEXLS_INTERFACE_DIR  "${CMAKE_CURRENT_LIST_DIR}/interface")


humoto_add_external_git_project(TARGET_NAME "${TGT_LEXLS}"
                                PROJECT_DIR "${LEXLS_SOURCE_DIR}"
                                NOBUILD)

add_custom_target(  TGT_copy_lexls_headers COMMAND ${CMAKE_COMMAND} -E
                    copy_directory "${LEXLS_SOURCE_DIR}/include" "${HUMOTO_BRIDGES_INCLUDE_DIR}/lexls")
add_dependencies(TGT_copy_lexls_headers    ${TGT_LEXLS})
add_dependencies(TGT_humoto_copy_headers   TGT_copy_lexls_headers)

add_dependencies(fetch-bridges ${TGT_LEXLS}-download)

humoto_copy_file_if_exists( TGT_humoto_copy_headers             "lexls_h"
                            "${LEXLS_INTERFACE_DIR}/lexls.h"    "${HUMOTO_INCLUDE_DIR}")
humoto_copy_dir_if_exists(  TGT_humoto_copy_headers             "lexls_dir"
                            "${LEXLS_INTERFACE_DIR}/lexls"      "${HUMOTO_INCLUDE_DIR}/lexls")


set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_LIBS "" PARENT_SCOPE)
set(HUMOTO_BRIDGE_${HUMOTO_BRIDGE}_INCLUDES "${HUMOTO_BRIDGES_INCLUDE_DIR}/lexls" PARENT_SCOPE)
