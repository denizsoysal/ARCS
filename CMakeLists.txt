cmake_minimum_required(VERSION 3.13.0)
project(iiwase VERSION 0.0.1 LANGUAGES C CXX)

message(STATUS ${CMAKE_CURRENT_SOURCE_DIR}) ##This is to print the status during the cmake

set(CMAKE_BUILD_TYPE Debug)

############
## Option ##
############
option(STAND_ALONE "Compile this package outside a higher lever repository" ON)
option(BUILD_OFFLINE "Build offline" OFF)

if (${BUILD_OFFLINE})
    set_property(DIRECTORY ${${PROJECT_NAME}_SOURCE_DIR}
                PROPERTY EP_UPDATE_DISCONNECTED 1)
endif()


###########
## Build ##
###########

# Eigen requirement for KDL
find_package(Eigen3 REQUIRED)
message(STATUS ${EIGEN3_INCLUDE_DIR})

# KDL and KDL Models
set(BUILD_MODELS ON CACHE BOOL "Build kdl models")
add_subdirectory(external/orocos_kdl)

include(FetchContent)

# SPDLOG (other way to do it, but not working consistently)
# FetchContent_Declare(spdlog
#     GIT_REPOSITORY https://github.com/gabime/spdlog.git
#     GIT_TAG v1.x
#     TIMEOUT 5
#     CMAKE_ARGS
#         -DSPDLOG_BUILD_SHARED=ON
#     BUILD_COMMAND make
#     INSTALL_COMMAND ""
# )
# FetchContent_MakeAvailable(spdlog)
# FetchContent_GetProperties(spdlog
#     SOURCE_DIR SPDLOG_SOURCE_DIR
#     BINARY_DIR SPDLOG_BINARY_DIR
# )
# set(SPDLOG_INCLUDE_DIR ${SPDLOG_SOURCE_DIR}/include)

# ROB-KUL libraries
# FIVE C
FetchContent_Declare(five-c
    GIT_REPOSITORY git@gitlab.kuleuven.be:u0144428/algorithm_and_activity_component_architecture.git
    GIT_TAG repo/organization
    TIMEOUT 5
    INSTALL_COMMAND ""
)
FetchContent_MakeAvailable(five-c)
FetchContent_GetProperties(five-c
    SOURCE_DIR FIVE_C_SOURCE_DIR
    BINARY_DIR FIVE_C_BUILD_DIR
)
set(FIVE_C_INCLUDE_DIR ${FIVE_C_SOURCE_DIR}/include)

# Coordination libraries
FetchContent_Declare(coordination-libraries
    GIT_REPOSITORY git@gitlab.kuleuven.be:u0144428/coordination_libraries.git
    GIT_TAG repo/organization
    TIMEOUT 5
    INSTALL_COMMAND ""
)
FetchContent_MakeAvailable(coordination-libraries)

FetchContent_GetProperties(coordination-libraries
    SOURCE_DIR COORDINATON_LIBRARIES_SOURCE_DIR
    BINARY_DIR COORDINATON_LIBRARIES_BUILD_DIR
)
set(COORDINATON_LIBRARIES_INCLUDE_DIR ${COORDINATON_LIBRARIES_SOURCE_DIR}/include)

# Including iiwa iiwa_activity
FetchContent_Declare(iiwa-aacal
    GIT_REPOSITORY git@gitlab.kuleuven.be:u0141400/iiwa_aacal.git
    GIT_TAG repo/organization
    TIMEOUT 5
    INSTALL_COMMAND ""
)
FetchContent_MakeAvailable(iiwa-aacal)

FetchContent_GetProperties(iiwa-aacal
    SOURCE_DIR IIWA_AACAL_SOURCE_DIR
    BINARY_DIR IIWA_AACAL_BUILD_DIR
)
set(IIWA_AACAL_INCLUDE_DIR ${IIWA_AACAL_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} SHARED
    src/navigation.cpp
    src/iiwa/iiwa_controller.cpp
    src/iiwa/iiwa_virtual.cpp
    src/iiwa/iiwa_state_estimation.cpp
    src/task_mediator/task_mediator.cpp
    src/task_mediator/task_mediator_petrinet.cpp
)

add_dependencies(${PROJECT_NAME} five_c coordination_libraries iiwa_aacal orocos-kdl orocos-kdl-models)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${FIVE_C_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${COORDINATON_LIBRARIES_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${IIWA_AACAL_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/external/orocos_kdl/models>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/external/orocos_kdl/src>
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
        ${${PROJECT_NAME}_SOURCE_DIR}/external/spdlog/
)

target_link_directories(${PROJECT_NAME}
    PUBLIC
        ${COORDINATON_LIBRARIES_BUILD_DIR}
        ${IIWA_AACAL_BUILD_DIR}
        ${CMAKE_SOURCE_DIR}/external/spdlog/
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    five_c
    coordination_libraries
    iiwa_aacal
    pthread
    spdlog
    PUBLIC orocos-kdl orocos-kdl-models
)

add_executable(iiwase_demo src/demo/iiwase_demo.cpp)
target_link_libraries(iiwase_demo PRIVATE
    ${PROJECT_NAME}
    five_c
    coordination_libraries
    iiwa_aacal
    pthread
    spdlog
    PUBLIC orocos-kdl orocos-kdl-models
)    


#############
## Install ##
#############
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
