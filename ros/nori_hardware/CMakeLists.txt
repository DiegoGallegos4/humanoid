cmake_minimum_required(VERSION 3.16)
project(nori_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# --- locate the MuJoCo C library shipped by the pip wheel (see docker/Dockerfile)
execute_process(
  COMMAND python3 -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))"
  OUTPUT_VARIABLE MUJOCO_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE MUJOCO_FIND_RESULT)
if(NOT MUJOCO_FIND_RESULT EQUAL 0 OR NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
  message(FATAL_ERROR "MuJoCo not found (expected the pip wheel). Got: '${MUJOCO_DIR}'")
endif()
file(GLOB MUJOCO_LIB "${MUJOCO_DIR}/libmujoco.so*")
if(NOT MUJOCO_LIB)
  message(FATAL_ERROR "libmujoco.so not found under ${MUJOCO_DIR}")
endif()
message(STATUS "MuJoCo: ${MUJOCO_LIB} (headers: ${MUJOCO_DIR}/include)")

# shared firmware<->ROS contract (header-only): firmware/protocol
set(FIRMWARE_PROTOCOL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/protocol/include")

add_library(nori_hardware SHARED src/nori_mujoco_system.cpp)
target_include_directories(nori_hardware PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${MUJOCO_DIR}/include
  ${FIRMWARE_PROTOCOL_INCLUDE})
target_link_libraries(nori_hardware ${MUJOCO_LIB})
ament_target_dependencies(nori_hardware
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

# find libmujoco.so.X at runtime without polluting global LD_LIBRARY_PATH
set_target_properties(nori_hardware PROPERTIES
  BUILD_RPATH "${MUJOCO_DIR}"
  INSTALL_RPATH "${MUJOCO_DIR}")

pluginlib_export_plugin_description_file(hardware_interface nori_hardware.xml)

install(TARGETS nori_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_libraries(nori_hardware)
ament_package()
