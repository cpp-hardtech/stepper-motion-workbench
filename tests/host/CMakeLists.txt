set(motion_src "${PROJECT_SOURCE_DIR}/src")
option(STEPPER_MOTION_HOST_TRACES "Generate FasNAxis HTML traces in host tests" OFF)

add_library(stepper_motion_host STATIC
    "${motion_src}/StepperMotion.cpp"
    "${motion_src}/StepperMotionEngine.cpp"
    "${motion_src}/fas_queue/queue_add_entry.cpp"
    "${motion_src}/fas_queue/queue_get_position.cpp"
    "${motion_src}/fas_queue/queue_init.cpp"
    "${motion_src}/fas_queue/queue_utils.cpp"
    "${motion_src}/fas_ramp/RampCalculator.cpp"
    "${motion_src}/fas_ramp/RampControl.cpp"
    "${motion_src}/fas_ramp/RampGenerator.cpp"
    "${motion_src}/log2/Log2Representation.cpp"
    StepperISR_test.cpp
)
target_include_directories(stepper_motion_host PRIVATE "${motion_src}")
target_compile_definitions(stepper_motion_host PRIVATE TEST F_CPU=16000000)
target_compile_features(stepper_motion_host PRIVATE cxx_std_17)

file(GLOB test_sources CONFIGURE_DEPENDS "test_[0-9][0-9].cpp")
list(APPEND test_sources log2_test.cpp rmc_test.cpp)

foreach(source IN LISTS test_sources)
    get_filename_component(test_name "${source}" NAME_WE)
    add_executable("${test_name}" "${source}")
    target_include_directories("${test_name}" PRIVATE "${motion_src}")
    target_compile_definitions("${test_name}" PRIVATE TEST F_CPU=16000000)
    target_compile_features("${test_name}" PRIVATE cxx_std_17)
    target_compile_options("${test_name}" PRIVATE -UNDEBUG)
    if(NOT test_name MATCHES "^test_(21|22|24|25|28)$")
        target_link_libraries("${test_name}" PRIVATE stepper_motion_host)
    endif()
    add_test(NAME "${test_name}" COMMAND "${test_name}")
    if(STEPPER_MOTION_HOST_TRACES AND test_name STREQUAL "test_26")
        set(naxis_trace_dir "${CMAKE_CURRENT_BINARY_DIR}/naxis-traces")
        configure_file(viewer_template.html "${naxis_trace_dir}/viewer_template.html" COPYONLY)
        target_compile_definitions("${test_name}" PRIVATE FAS_NAXIS_TRACE NAXIS_HTML_ROOT=".")
        set_tests_properties("${test_name}" PROPERTIES WORKING_DIRECTORY "${naxis_trace_dir}")
    endif()
endforeach()

set(naxis_html_dump_dir "${CMAKE_CURRENT_BINARY_DIR}/naxis_html_dump")
configure_file(viewer_template.html "${naxis_html_dump_dir}/viewer_template.html" COPYONLY)
add_executable(naxis_html_dump_test naxis_html_dump_test.cpp)
target_compile_features(naxis_html_dump_test PRIVATE cxx_std_17)
add_test(NAME naxis_html_dump COMMAND naxis_html_dump_test)
set_tests_properties(naxis_html_dump PROPERTIES WORKING_DIRECTORY "${naxis_html_dump_dir}")
