function(humoto_identify_git_commit)
    set(HUMOTO_GIT_COMMIT_INFO  "<not available>")
    if(EXISTS ${PROJECT_SOURCE_DIR}/.git)
        find_package(Git)
        if(GIT_FOUND)
            execute_process(
                COMMAND ${GIT_EXECUTABLE} log --pretty=%H -1
                WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
                OUTPUT_VARIABLE HUMOTO_GIT_COMMIT_ID
                OUTPUT_STRIP_TRAILING_WHITESPACE)
            execute_process(
                COMMAND ${GIT_EXECUTABLE} log --pretty=%ai -1
                WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
                OUTPUT_VARIABLE HUMOTO_GIT_COMMIT_DATE
                OUTPUT_STRIP_TRAILING_WHITESPACE)

            if (HUMOTO_GIT_COMMIT_ID AND HUMOTO_GIT_COMMIT_DATE)
                set(HUMOTO_GIT_COMMIT_INFO
                    "date ${HUMOTO_GIT_COMMIT_DATE} | hash ${HUMOTO_GIT_COMMIT_ID}"
                    PARENT_SCOPE)
            endif()
        endif(GIT_FOUND)
    endif()
endfunction(humoto_identify_git_commit)
