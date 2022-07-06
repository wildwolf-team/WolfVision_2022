# Create project
cmake_minimum_required(VERSION 3.14.0)
project(WolfVision VERSION 2021.7.9 LANGUAGES CXX)

# Check system platform
if(NOT CMAKE_SYSTEM_NAME STREQUAL Linux)
    message(FATAL_ERROR "Not supported platform: ${CMAKE_SYSTEM_NAME}")
endif()

# Specialize CMake build flags for different build types
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-deprecated")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Use standard c++20
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add DEBUG/RELEASE marco to define if debug/release configured
add_compile_options("$<$<CONFIG:DEBUG>:-DDEBUG>")
add_compile_options("$<$<CONFIG:RELEASE>:-DRELEASE>")

# Don't build test by default
set(BUILD_TEST False)

# Prefer posix threads
set(THREADS_PREFER_PTHREAD_FLAG True)
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
find_package(Threads REQUIRED)

include_directories(
    #OpenVINO推理引擎的头文件
    ~/intel/openvino_2021/deployment_tools/inference_engine/include/
    ~/intel/openvino_2021/deployment_tools/ngraph/include/
    .
)

set(InferenceEngine_DIR "~/intel/openvino_2021/deployment_tools/inference_engine/share")
find_package(InferenceEngine)

# new serial
set(serial_SRCS
    devices/serial/src/serial.cc
    devices/serial/src/impl/unix.cc
    devices/serial/src/impl/list_ports/list_ports_linux.cc
    devices/serial/serial/serial.h
    devices/serial/serial/v8stdint.h
)

# Find and add opencv as required
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

# Add Fetch Content module
include(FetchContent)

# Add fmtlib directory
find_package(fmt QUIET)
if (NOT fmt_FOUND)
    FetchContent_Declare(
        fmt
        GIT_REPOSITORY https://hub.fastgit.xyz/fmtlib/fmt.git
        GIT_TAG 9.0.0
    )
    FetchContent_MakeAvailable(fmt)
    list(APPEND EXTRA_INCLUDES ${fmt_SOURCE_DIR}/include)
endif()

# Add fmtlib include to EXTRA_INCLUDES

#add eigen
find_package(Eigen3 QUIET)
if (NOT Eigen3_FOUND)
    FetchContent_Declare(
        Eigen3
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    )
    FetchContent_MakeAvailable(Eigen3)
    list(APPEND EXTRA_INCLUDES ${Eigen3_SOURCE_DIR}/Eigen)
else()
    list(APPEND EXTRA_INCLUDES ${EIGEN3_INCLUDE_DIRS})
endif()

# Add mindvision-sdk directory and include path for target
add_library(MVSDK SHARED IMPORTED)
get_filename_component(MVSDK_PATH 3rdparty/mindvision/linux REALPATH)
# Check system architecture for mindvision-sdk library
execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "Checking platform architecture for MVSDK: ${ARCHITECTURE}")
if(${ARCHITECTURE} STREQUAL x86_64)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/x64/libMVSDK.so)
elseif(${ARCHITECTURE} STREQUAL x86)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/x86/libMVSDK.so)
elseif(${ARCHITECTURE} STREQUAL aarch64)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/arm64/libMVSDK.so)
elseif(${ARCHITECTURE} STREQUAL armv7)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/arm/libMVSDK.so)
else()
    message(FATAL_ERROR "Not supported architecture for MVSDK: ${ARCHITECTURE}")
endif()
set_property(TARGET MVSDK PROPERTY IMPORTED_LOCATION ${MIMDVISION_SDK_LIBRARY})
# Add mindvision lib to build target
list(APPEND EXTRA_LIBS MVSDK)
# Add mindvision include path
list(APPEND EXTRA_INCLUDES 3rdparty/mindvision/linux/include)

# Add project source dir to include
list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR})

# Add base src and header
list(APPEND EXTRA_INCLUDES base)

# Add device as shared library to build target
list(APPEND EXTRA_INCLUDES devices)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/devices/camera)
add_library(wv-mv-video-capture SHARED devices/camera/mv_video_capture.cpp)
target_link_libraries(wv-mv-video-capture
    PRIVATE
    MVSDK
)
list(APPEND WV_EXTRA_LIBS wv-mv-video-capture)

# Add module as shared library to build target
list(APPEND EXTRA_INCLUDES module)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/net)
add_library(wv-basic-net SHARED module/net/basic_net.cpp)
list(APPEND WV_EXTRA_LIBS wv-basic-net)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/armor)
add_library(wv-basic-armor SHARED module/armor/basic_armor.cpp)
list(APPEND WV_EXTRA_LIBS wv-basic-armor)
add_library(wv-fan-armor SHARED module/armor/fan_armor.cpp)
list(APPEND WV_EXTRA_LIBS wv-fan-armor)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/buff)
add_library(wv-basic-buff SHARED module/buff/basic_buff.cpp)
target_link_libraries(wv-basic-buff
    PRIVATE
    wv-fan-armor
)
list(APPEND WV_EXTRA_LIBS wv-basic-buff)

# Add utils header
list(APPEND EXTRA_INCLUDES utils)

# Add EXTRA_INCLUDES to project includes
include_directories(${EXTRA_INCLUDES})

# Add base/wolfvision to executable
add_executable(WolfVision base/wolfvision.cpp)

include_directories(devices/serial)
add_library(serial ${serial_SRCS})

# Add shared library link to WolfVision
target_link_libraries(WolfVision
    PUBLIC
    Threads::Threads
    fmt::fmt
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${EXTRA_LIBS}
    PRIVATE
    ${WV_EXTRA_LIBS}
    serial
    ${InferenceEngine_LIBRARIES}
)

# Set WolfVision compile cxx_std features
target_compile_features(WolfVision PUBLIC cxx_std_${CMAKE_CXX_STANDARD})

# Set WolfVision linking time optimization to true
set_target_properties(WolfVision PROPERTIES INTERPROCEDURAL_OPTIMIZATION True)

# Set built binary to ~/bin
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

# Set built library to ~/lib
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib")

# Set configs folder absolute path
set(CONFIG_FILE_PATH ${PROJECT_SOURCE_DIR}/configs)
set(SOURCE_PATH ${PROJECT_SOURCE_DIR})
target_compile_definitions(WolfVision PRIVATE "CONFIG_FILE_PATH=\"${CONFIG_FILE_PATH}\"")
target_compile_definitions(WolfVision PRIVATE "SOURCE_PATH=\"${SOURCE_PATH}\"")
target_compile_definitions(wv-basic-buff PRIVATE "CONFIG_FILE_PATH=\"${CONFIG_FILE_PATH}\"")


# Add test_camera directory
if(BUILD_TEST)
    add_subdirectory(test/test_camera)
endif()
