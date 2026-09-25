cmake_minimum_required(VERSION 3.16)
project(RMVTraining LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_WEB_DEBUG "Build the scalar Web Debug dashboard" ON)

# 海康 MVS SDK 取流（可选）。装有 SDK 时默认开启，可用 -DENABLE_HIKROBOT=OFF 关闭。
set(MVS_ROOT "/opt/MVS" CACHE PATH "海康 MVS SDK 安装根目录")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND EXISTS "${MVS_ROOT}/include/MvCameraControl.h")
    set(_hikrobot_default ON)
else()
    set(_hikrobot_default OFF)
endif()
option(ENABLE_HIKROBOT "Use the Hikrobot MVS SDK for camera capture" ${_hikrobot_default})
option(BUILD_TESTING "Build the starter-project tests" ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc videoio calib3d dnn)
find_package(Threads REQUIRED)

add_subdirectory(serialPort)
if(ENABLE_WEB_DEBUG)
    add_subdirectory(web_debug)
endif()

# 视觉算法模块，移植自 src/rm_aa_vision。
add_library(vision STATIC
    vision/src/Source.cpp
    vision/src/Preprocessor.cpp
    vision/src/Detector.cpp
    vision/src/Classifier.cpp
    vision/src/PoseSolver.cpp
)
target_include_directories(vision PUBLIC vision/include include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision PUBLIC ${OpenCV_LIBS})

if(ENABLE_HIKROBOT)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(_mvs_arch 64)
    else()
        set(_mvs_arch 32)
    endif()
    find_path(MVS_INCLUDE_DIR MvCameraControl.h HINTS "${MVS_ROOT}/include")
    find_library(MVS_LIBRARY NAMES MvCameraControl HINTS "${MVS_ROOT}/lib/${_mvs_arch}" "${MVS_ROOT}/lib")
    if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
        message(FATAL_ERROR "未找到 MVS 头文件或库，请安装完整 MVS 开发包并设置 MVS_ROOT")
    endif()
    target_compile_definitions(vision PRIVATE ENABLE_HIKROBOT=1)
    target_include_directories(vision SYSTEM PRIVATE "${MVS_INCLUDE_DIR}")
    target_link_libraries(vision PUBLIC "${MVS_LIBRARY}")
    message(STATUS "海康取流已启用: ${MVS_LIBRARY}")
else()
    message(STATUS "海康取流未启用，仅支持 UVC/V4L2 与视频输入")
endif()

# 跟踪与弹道模块，移植自 src/rm_aa_control。
add_library(tracking STATIC
    tracking/src/Tracker.cpp
    tracking/src/Mpc.cpp
)
target_include_directories(tracking PUBLIC tracking/include)
target_link_libraries(tracking PUBLIC Eigen3::Eigen)

add_library(training_core STATIC
    src/AppSupport.cpp
    src/AutoAimTypes.cpp
    student/src/StudentTasks.cpp
)
target_include_directories(training_core
    PUBLIC
        include
        student/include
        serialPort/include
        vision/include
        tracking/include
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(training_core PUBLIC serialPort vision tracking ${OpenCV_LIBS})

add_executable(rmv_training main.cpp)
target_include_directories(rmv_training PRIVATE web_debug/include)
target_link_libraries(rmv_training
    PRIVATE
        training_core
        vision
        tracking
        Threads::Threads
)
if(ENABLE_WEB_DEBUG)
    target_link_libraries(rmv_training PRIVATE web_debug)
endif()

if(BUILD_TESTING)
    enable_testing()
    add_executable(test_scaffold tests/test_scaffold.cpp)
    target_link_libraries(test_scaffold PRIVATE training_core)
    add_test(NAME scaffold_safety COMMAND test_scaffold)
    add_test(NAME cli_help COMMAND rmv_training --help)
endif()
