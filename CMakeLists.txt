cmake_minimum_required(VERSION 3.20)

if(ESP_PLATFORM)
    idf_component_register(
        SRC_DIRS src src/pd_esp32 src/fas_ramp src/log2 src/fas_queue
        INCLUDE_DIRS src
        REQUIRES esp_driver_gpio esp_driver_rmt esp_driver_mcpwm esp_driver_pcnt esp_driver_i2s
    )
    return()
endif()

project(StepperMotionWorkbench LANGUAGES CXX)

include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests/host)
endif()
