# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS
    ~/Developer/esp/esp-zigbee-sdk/examples/common/light_driver
    ~/Developer/esp/esp-zigbee-sdk/examples/common/switch_driver
    )
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bathroom_thermostat_controller)
