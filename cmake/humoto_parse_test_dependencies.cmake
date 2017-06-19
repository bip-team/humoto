function(humoto_parse_test_dependencies DEPENDENCIES LINK_TO_LIBRARIES TGT_DEPENDS)
    set(MISSING_DEPENDENCY OFF)

    list(APPEND TGT_DEPENDS TGT_humoto_copy_config)

    foreach(DEPENDENCY ${DEPENDENCIES})
        list (FIND HUMOTO_BRIDGES "${DEPENDENCY}" INDEX)

        if (${INDEX} GREATER -1)
            if (HUMOTO_BRIDGE_${DEPENDENCY})
                list(APPEND TGT_DEPENDS "${BRIDGE_TARGET_PREFIX}_${DEPENDENCY}")
                list(APPEND LINK_TO_LIBRARIES "${HUMOTO_BRIDGE_${DEPENDENCY}_LIBS}")
            else()
                set(MISSING_DEPENDENCY ON)
            endif()
        else()
            string(COMPARE NOTEQUAL "${DEPENDENCY}" "DIFF_WITH_REFERENCE" COMPARE_RESULT)
            if (COMPARE_RESULT)
                set(MISSING_DEPENDENCY ON)
            endif(COMPARE_RESULT)
        endif()
    endforeach(DEPENDENCY)


    set(LINK_TO_LIBRARIES   "${LINK_TO_LIBRARIES}"  PARENT_SCOPE)
    set(TGT_DEPENDS         "${TGT_DEPENDS}"        PARENT_SCOPE)
    set(MISSING_DEPENDENCY  "${MISSING_DEPENDENCY}" PARENT_SCOPE)
endfunction()
