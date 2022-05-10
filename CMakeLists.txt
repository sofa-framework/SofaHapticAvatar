cmake_minimum_required(VERSION 3.1)
project(SofaHapticAvatar VERSION 0.1)

find_package(Sofa.GL REQUIRED)
find_package(SofaConstraint REQUIRED)
find_package(SofaHaptics REQUIRED)
sofa_find_package(TinyXML REQUIRED)

set(SOFAHAPTICAVATAR_SRC_DIR src/SofaHapticAvatar)
set(HEADER_FILES
    ${SOFAHAPTICAVATAR_SRC_DIR}/config.h.in
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_Defines.h
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_DriverBase.h
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_DriverPort.h
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_DriverIbox.h
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_DriverScope.h
	${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_PortalManager.h
	${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_Portal.h    
    
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_HapticThreadManager.h    

    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_IBoxController.h    
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_BaseDeviceController.h
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_ArticulatedDeviceController.h
    
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_GrasperDeviceController.h
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_ArticulatedDeviceEmulator.h 
)

set(SOURCE_FILES
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_DriverBase.cpp
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_DriverPort.cpp
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_DriverIbox.cpp
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_DriverScope.cpp
	${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_PortalManager.cpp
	${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_Portal.cpp        
    
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_HapticThreadManager.cpp
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_IBoxController.cpp
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_BaseDeviceController.cpp
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_ArticulatedDeviceController.cpp
    
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_GrasperDeviceController.cpp
    ${SOFAHAPTICAVATAR_SRC_DIR}/HapticAvatar_ArticulatedDeviceEmulator.cpp
    
    ${SOFAHAPTICAVATAR_SRC_DIR}/initSofaHapticAvatarPlugin.cpp
)

set(README_FILES Readme.txt)

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Set define dllimport/dllexport mechanism on Windows.
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_SOFAHAPTICAVATAR")

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} PUBLIC SofaConstraint SofaHaptics Sofa.GL)
target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml) # Private because not exported in API

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

## Install rules for the library; CMake package configurations files
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )    