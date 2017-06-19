function(humoto_add_external_svn_project)
    set(options NOBUILD)
    set(oneValueArgs TARGET_NAME PROJECT_DIR REPOSITORY)
    set(multiValueArgs CMAKE_ARGS)
    cmake_parse_arguments("HUMOTO" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )


    if (EXISTS "${HUMOTO_PROJECT_DIR}/.svn")
        message("Found '.svn' subdirectory in project '${HUMOTO_PROJECT_DIR}': skipping download step.")

        if (HUMOTO_NOBUILD)
            ExternalProject_Add(
                ${HUMOTO_TARGET_NAME}
                DOWNLOAD_DIR    "${HUMOTO_PROJECT_DIR}"
                SOURCE_DIR      "${HUMOTO_PROJECT_DIR}"
                DOWNLOAD_COMMAND    ""
                BUILD_COMMAND       ""
                INSTALL_COMMAND     ""
                CONFIGURE_COMMAND   ""
                STEP_TARGETS    download
            )
        else (HUMOTO_NOBUILD)
            ExternalProject_Add(
                ${HUMOTO_TARGET_NAME}
                DOWNLOAD_DIR    "${HUMOTO_PROJECT_DIR}"
                SOURCE_DIR      "${HUMOTO_PROJECT_DIR}"
                CMAKE_ARGS      ${HUMOTO_CMAKE_ARGS}
                DOWNLOAD_COMMAND    ""
                STEP_TARGETS    download
            )
        endif (HUMOTO_NOBUILD)
    else()
        find_package(Subversion)

        if (SUBVERSION_FOUND)
            if (HUMOTO_NOBUILD)
                ExternalProject_Add(
                    ${HUMOTO_TARGET_NAME}
                    DOWNLOAD_DIR    "${HUMOTO_PROJECT_DIR}"
                    SOURCE_DIR      "${HUMOTO_PROJECT_DIR}"
                    SVN_REPOSITORY  "${HUMOTO_REPOSITORY}"
                    SVN_TRUST_CERT  1
                    CMAKE_ARGS      ${HUMOTO_CMAKE_ARGS}
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    CONFIGURE_COMMAND   ""
                    STEP_TARGETS    download
                )
            else (HUMOTO_NOBUILD)
                ExternalProject_Add(
                    ${HUMOTO_TARGET_NAME}
                    DOWNLOAD_DIR    "${HUMOTO_PROJECT_DIR}"
                    SOURCE_DIR      "${HUMOTO_PROJECT_DIR}"
                    SVN_REPOSITORY  "${HUMOTO_REPOSITORY}"
                    SVN_TRUST_CERT  1
                    CMAKE_ARGS      ${HUMOTO_CMAKE_ARGS}
                    STEP_TARGETS    download
                )
            endif (HUMOTO_NOBUILD)
        else(SUBVERSION_FOUND)
            message( SEND_ERROR "Cannot download project '${HUMOTO_PROJECT_DIR}' due to missing svn. Checking if the files are alredy there..." )
            file(GLOB RESULT ${HUMOTO_PROJECT_DIR})
            list(LENGTH RESULT RES_LEN)
            if(RES_LEN EQUAL 0)
                message(FATAL_ERROR "Project directory '${HUMOTO_PROJECT_DIR}' is empty, compilation is impossible." )
            endif()
        endif(SUBVERSION_FOUND)
    endif()
endfunction(humoto_add_external_svn_project)
