# Finding a settings header by name, shared by pack profiles (profiles/) and
# boards (boards/). Each is a <name>.h in one of a list of directories, searched
# in order; the same name in two of them is an error.

function(bms_list_headers dirs out)
    list(REMOVE_ITEM dirs "")
    set(names)
    foreach(dir IN LISTS dirs)
        file(GLOB headers ${dir}/*.h)
        foreach(h IN LISTS headers)
            get_filename_component(n ${h} NAME_WE)
            list(APPEND names ${n})
        endforeach()
    endforeach()
    list(REMOVE_DUPLICATES names)
    list(SORT names)
    string(REPLACE ";" ", " names "${names}")
    set(${out} "${names}" PARENT_SCOPE)
endfunction()

# Resolve <name>.h in dirs into out_header. what is the kind of header ("pack
# profile", "board") and var its cache variable, for the messages; hint is
# added when the name isn't found.
function(bms_select_header what var name dirs hint out_header)
    list(REMOVE_ITEM dirs "") # an unset optional directory
    if(NOT name MATCHES "^[A-Za-z0-9_-]+$")
        bms_list_headers("${dirs}" available)
        message(FATAL_ERROR "Invalid ${var} '${name}': a ${what} is named by its file, "
            "e.g. -D${var}=<name>. Available: ${available}\n${hint}")
    endif()

    set(found)
    foreach(dir IN LISTS dirs)
        if(EXISTS ${dir}/${name}.h)
            list(APPEND found ${dir}/${name}.h)
        endif()
    endforeach()

    list(LENGTH found count)
    if(count EQUAL 0)
        bms_list_headers("${dirs}" available)
        message(FATAL_ERROR "No ${what} '${name}'. Available: ${available}\n${hint}")
    elseif(count GREATER 1)
        string(REPLACE ";" "\n  " found "${found}")
        message(FATAL_ERROR "${what} '${name}' is defined more than once:\n  ${found}\n"
            "Give the custom one a different name.")
    endif()

    set(${out_header} ${found} PARENT_SCOPE)
endfunction()
