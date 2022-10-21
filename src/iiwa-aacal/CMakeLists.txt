cmake_minimum_required(VERSION 3.10.0)
project(iiwa-aacal VERSION 0.0.1 LANGUAGES C CXX)

# Get the project name in capital letters
# for the lib namespace
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_CAPS)

# Options. Turn on with 'cmake -DBUILD_TESTING=ON'.
option(BUILD_TESTING "Build all tests." OFF)
option(BUILD_EXAMPLES "Build all examples." OFF)

set(CMAKE_BUILD_TYPE Debug)

###########
## Build ##
###########

# option(STAND_ALONE "Compile this package outside a higher lever repository" OFF)
option(STAND_ALONE "Compile this package outside a higher lever repository" ON)

message(STATUS ${CMAKE_CURRENT_SOURCE_DIR}) 
# Rebuild the lidar each time cmake is invoked

if(STAND_ALONE)
    set(UTHashIncludeDir ${CMAKE_SOURCE_DIR}/external/uthash/include)
    set(CJSONLibIncludeDir ${CMAKE_SOURCE_DIR}/external/cJSON)
    set(ReadParameterLibIncludeDir ${CMAKE_SOURCE_DIR}/external/read_parameter_struct/include)
    set(CoordLibIncludeDir ${CMAKE_SOURCE_DIR}/external/coordination_libraries/include)
    set(AACALibIncludeDir ${CMAKE_SOURCE_DIR}/external/algorithm_and_activity_component_architecture/include)
    set(DataStructLibIncludeDir ${CMAKE_SOURCE_DIR}/external/data-structure-aacal/include)
    ## KUKA SDK specific
    set(KUKA_FRI_SDK ${CMAKE_SOURCE_DIR}/external/kuka_fri_sdk/)
    set(FRISDK_INCLUDE ${CMAKE_SOURCE_DIR}/external/kuka_fri_sdk/include)
    set(FRISDK_LIB ${CMAKE_SOURCE_DIR}/external/kuka_fri_sdk/lib)

    add_subdirectory(external/cJSON)
    add_subdirectory(external/read_parameter_struct)
    add_subdirectory(external/coordination_libraries)
    add_subdirectory(external/algorithm_and_activity_component_architecture)
    add_subdirectory(external/data-structure-aacal)
endif()

include_directories(${FRISDK_INCLUDE})
link_directories(${FRISDK_LIB})

add_library(${PROJECT_NAME} SHARED
    src/iiwa_client.cpp
    src/iiwa_interface.cpp
    src/iiwa_activity.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${AACALibIncludeDir}>
        $<BUILD_INTERFACE:${DataStructLibIncludeDir}>
        $<BUILD_INTERFACE:${ReadParameterLibIncludeDir}>
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    aaca-lib
    data-structure-aacal
    /home/fulloa/AACAL/iiwa-aacal/external/kuka_fri_sdk/lib/libFRIClient.so
)

if(STAND_ALONE)
    add_executable(iiwa_demo src/iiwa_demo.cpp)
    target_link_libraries(iiwa_demo PRIVATE
        ${PROJECT_NAME}
        pthread
        aaca-lib
        data-structure-aacal
        /home/fulloa/AACAL/iiwa-aacal/external/kuka_fri_sdk/lib/libFRIClient.so

        )    

endif()


#############
## Install ##
#############
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
