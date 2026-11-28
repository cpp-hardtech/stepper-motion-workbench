cmake_minimum_required(VERSION 3.20)

file(MAKE_DIRECTORY "${TRACE_DIR}")
set(csv "${TRACE_DIR}/test_29.commands.csv")
file(REMOVE "${csv}")
execute_process(
    COMMAND "${TRACE_PROGRAM}" --command-csv
    WORKING_DIRECTORY "${TRACE_DIR}"
    RESULT_VARIABLE result
)
if(NOT result STREQUAL "0" OR NOT EXISTS "${csv}")
    message(FATAL_ERROR "Command trace export failed (exit ${result})")
endif()

file(STRINGS "${csv}" rows)
list(POP_FRONT rows header)
if(NOT header STREQUAL "start_tick,end_tick,x_delta,y_delta,x_position,y_position")
    message(FATAL_ERROR "Unexpected command trace header: ${header}")
endif()

set(tick 0)
set(x 0)
set(y 0)
set(has_submicrosecond_tick OFF)
set(remaining_corners "30635200,3032,0" "61270400,3032,3032" "91905600,0,3032")
foreach(row IN LISTS rows)
    if(NOT row MATCHES "^([0-9]+),([0-9]+),(-?[0-9]+),(-?[0-9]+),(-?[0-9]+),(-?[0-9]+)$")
        message(FATAL_ERROR "Invalid command row: ${row}")
    endif()
    math(EXPR x "${x} + ${CMAKE_MATCH_3}")
    math(EXPR y "${y} + ${CMAKE_MATCH_4}")
    if(NOT CMAKE_MATCH_1 EQUAL tick OR NOT CMAKE_MATCH_2 GREATER tick OR
       NOT CMAKE_MATCH_5 EQUAL x OR NOT CMAKE_MATCH_6 EQUAL y)
        message(FATAL_ERROR "Inconsistent command interval or position: ${row}")
    endif()
    set(tick "${CMAKE_MATCH_2}")
    list(REMOVE_ITEM remaining_corners "${tick},${x},${y}")
    math(EXPR remainder "${tick} % 16")
    if(remainder)
        set(has_submicrosecond_tick ON)
    endif()
endforeach()

if(remaining_corners)
    message(FATAL_ERROR "Missing square trace corners: ${remaining_corners}")
endif()
if(NOT has_submicrosecond_tick OR NOT tick EQUAL 122540800 OR NOT x EQUAL 0 OR NOT y EQUAL 0)
    message(FATAL_ERROR "Unexpected square trace endpoint: tick=${tick}, position=(${x}, ${y})")
endif()
