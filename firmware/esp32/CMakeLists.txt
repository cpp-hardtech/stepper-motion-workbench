cmake_minimum_required(VERSION 3.20)

get_filename_component(STEPPER_MOTION_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
list(APPEND EXTRA_COMPONENT_DIRS "${STEPPER_MOTION_DIR}")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stepper_motion_example)
