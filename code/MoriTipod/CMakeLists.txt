cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME TeleoperationDemoTripod)
set(PROJECT_DESCRIPTION "TeleoperationDemoTripod")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

find_package(mc_mori REQUIRED)
find_package(PahoMqttCpp REQUIRED)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/TeleoperationDemoTripod.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/TeleoperationDemoTripod.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/TeleoperationDemoTripod.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
