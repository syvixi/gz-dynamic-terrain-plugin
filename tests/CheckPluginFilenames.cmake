file(GLOB terrain_example_files
    "${EXAMPLES_DIR}/*.sdf"
    "${EXAMPLES_DIR}/*.config")

set(found_plugin FALSE)
foreach(example_file IN LISTS terrain_example_files)
    file(READ "${example_file}" contents)
    if(contents MATCHES "libgz-dynamic-terrain-system-v[0-9_]+\\.so")
        message(FATAL_ERROR
            "${example_file} still references a stale versioned terrain plugin")
    endif()
    if(contents MATCHES "libgz-dynamic-terrain-system\\.so")
        set(found_plugin TRUE)
    endif()
endforeach()

if(NOT found_plugin)
    message(FATAL_ERROR
        "No example references libgz-dynamic-terrain-system.so")
endif()
