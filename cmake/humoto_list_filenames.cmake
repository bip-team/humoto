# List filenames (all intermediate directories are stripped) in the given
# directory.
# This is equivalent to 'file(GLOB ... RELATIVE ... ...)', but behaves more
# consistently with different versions of cmake.
#
function(humoto_list_filenames  DIR     LISTNAME)
    file(GLOB FILENAMES_TMP "${DIR}/*")

    set (FILENAMES "")
    foreach(FILENAME_TMP ${FILENAMES_TMP})
        get_filename_component(FILENAME_TMP ${FILENAME_TMP} NAME)
        list(APPEND FILENAMES ${FILENAME_TMP})
    endforeach(FILENAME_TMP)

    set(${LISTNAME}   "${FILENAMES}"  PARENT_SCOPE)
endfunction()
