# TEST_CNN/files 独立测试，与主工程构建无关
cmake_minimum_required(VERSION 3.16)
project(test_onnx CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV 4 REQUIRED)
add_executable(test_onnx test_onnx.cpp)
target_link_libraries(test_onnx PRIVATE ${OpenCV_LIBS})
