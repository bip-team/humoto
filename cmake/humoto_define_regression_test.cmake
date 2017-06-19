# parameters
#   HUMOTO_MODULE               module
#   REGRESSION_TEST_ID          test id, number starting from 000
#   DEPENDENCIES                list of dependencies
#   [OPTIONAL_DEPENDENCIES]     list of optional dependencies, can be omitted
#
function(humoto_define_regression_test HUMOTO_MODULE REGRESSION_TEST_ID DEPENDENCIES)
    set(TEST_NAME        "regression_test_${REGRESSION_TEST_ID}")

    include_directories("${HUMOTO_CORE_DIR}/tests_include/")

    set(OPTIONAL_DEPENDENCIES   "${ARGV3}")
    set(LINK_TO_LIBRARIES       "${HUMOTO_MANDATORY_LIBRARIES}")


    humoto_parse_test_dependencies("${DEPENDENCIES};googletest" "${LINK_TO_LIBRARIES}" "${TGT_DEPENDS}")
    if (MISSING_DEPENDENCY)
        return()
    endif()
    humoto_parse_test_dependencies("${OPTIONAL_DEPENDENCIES}" "${LINK_TO_LIBRARIES}" "${TGT_DEPENDS}")

    list (FIND DEPENDENCIES "DIFF_WITH_REFERENCE" INDEX)
    if (${INDEX} GREATER -1)
        if (HUMOTO_BUILD_WITH_QI)
            return()
        else()
            set(DIFF_WITH_REFERENCE ON)
        endif()
    endif()


    set(TGT_NAME "${HUMOTO_MODULE}_${TEST_NAME}")

    if(HUMOTO_BUILD_WITH_QI)
        #qibuild modif
        qi_create_bin(${TGT_NAME} "${TEST_NAME}.cpp" SUBFOLDER ${HUMOTO_MODULE})
    else()
        add_executable(${TGT_NAME} "${TEST_NAME}.cpp")
    endif()
    add_dependencies("${HUMOTO_MODULE}" "${TGT_NAME}")

    set_target_properties(${TGT_NAME} PROPERTIES OUTPUT_NAME "${TEST_NAME}")
    # TODO this is a workaround to suppress warnings due to googletest headers
    set_target_properties(${TGT_NAME} PROPERTIES COMPILE_FLAGS "-Wno-pedantic")

    add_dependencies(${TGT_NAME} TGT_humoto_copy_headers ${TGT_DEPENDS})

    target_link_libraries(${TGT_NAME} ${LINK_TO_LIBRARIES})

    #if(HUMOTO_BUILD_WITH_QI)
    #    qi_use_lib(${TGT_NAME} BOOST EIGEN3)
    #endif()


    humoto_copy_extra_test_files(${TGT_NAME} "copy_cfg" "${CMAKE_CURRENT_LIST_DIR}" "${TEST_NAME}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

    if(DIFF_WITH_REFERENCE)
        # never reach this branch if HUMOTO_BUILD_WITH_QI is set
        set(REF_FILENAME "${CMAKE_CURRENT_LIST_DIR}/${TEST_NAME}.ref")
        add_test(NAME ${TGT_NAME} COMMAND sh -c "./${TEST_NAME} | grep -o --color=never \"^humoto.*\ =\\|^%.*\" | diff ${REF_FILENAME} -" WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    else()
        if(HUMOTO_BUILD_WITH_QI)
            qi_add_test(${TGT_NAME} "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TEST_NAME}" ARGUMENTS "${HUMOTO_CONFIG_DIR}/${HUMOTO_MODULE}/" "${TEST_NAME}.ref" WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
        else()
            add_test(NAME ${TGT_NAME} COMMAND ${TEST_NAME} "${HUMOTO_CONFIG_DIR}/${HUMOTO_MODULE}/" "${TEST_NAME}.ref" WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
        endif()
    endif()
endfunction()
