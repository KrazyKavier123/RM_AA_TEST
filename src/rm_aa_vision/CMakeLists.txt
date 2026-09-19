cmake_minimum_required(VERSION 3.16)
project(rm_aa_vision LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d dnn video)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)

add_library(perception_core
  src/preprocessor.cpp src/detector.cpp src/classifier.cpp
  src/pose_solver.cpp src/image_tracker.cpp src/source.cpp)
target_include_directories(perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${OpenCV_INCLUDE_DIRS})
target_link_libraries(perception_core PUBLIC ${OpenCV_LIBS} Threads::Threads)
include(cmake/Hikrobot.cmake)
rm_aa_enable_hikrobot(perception_core)
target_compile_options(perception_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(perception_node src/perception_node.cpp)
target_link_libraries(perception_node perception_core nlohmann_json::nlohmann_json)
ament_target_dependencies(perception_node rclcpp sensor_msgs geometry_msgs std_msgs rcl_interfaces cv_bridge ament_index_cpp)
target_compile_options(perception_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(perception_cli src/perception_cli.cpp)
target_link_libraries(perception_cli perception_core nlohmann_json::nlohmann_json)

install(TARGETS perception_core EXPORT perception_core_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS perception_node perception_cli DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../licenses/rm_auto_aim-MIT.txt
  DESTINATION share/${PROJECT_NAME}/models RENAME LICENSE.txt)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception test/test_perception.cpp)
  target_link_libraries(test_perception perception_core)
  target_compile_definitions(test_perception PRIVATE MODEL_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/models")
endif()
ament_export_targets(perception_core_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV Threads)
ament_package()
