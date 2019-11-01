cmake_minimum_required(VERSION 3.1)
project(SofaHapticAvatar VERSION 0.1)

find_package(SofaGeneral REQUIRED)
find_package(SofaHaptics REQUIRED)

set(SOFAHAPTICAVATAR_SRC_DIR src/SofaHapticAvatar)
set(HEADER_FILES
    ${SOFAHAPTICAVATAR_SRC_DIR}/config.h.in
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatarDefines.h 
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatarAPI.h
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatarDriver.h
)

set(SOURCE_FILES
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatarAPI.cpp
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatarDriver.cpp
    ${SOFAHAPTICAVATAR_SRC_DIR}/initSofaHapticAvatarPlugin.cpp
)

set(README_FILES Readme.txt)

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Set define dllimport/dllexport mechanism on Windows.
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_SOFAHAPTICAVATAR")

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} SofaComponentGeneral SofaHaptics SofaOpenglVisual)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

## Install rules for the library; CMake package configurations files
sofa_generate_package(
    NAME ${PROJECT_NAME}
    TARGETS ${PROJECT_NAME}
    VERSION ${PROJECT_VERSION}
    INCLUDE_SOURCE_DIR ${SOFAHAPTICAVATAR_SRC_DIR}
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )