cmake_minimum_required(VERSION 3.10.0)
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

# ROB-KUL libraries
# FIVE C
include(FetchContent)
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
    src/iiwa/iiwa_controller.cpp
    src/iiwa/iiwa_virtual.cpp
    src/task_mediator/task_mediator.cpp
    src/task_mediator/task_mediator_petrinet.cpp
)

add_dependencies(${PROJECT_NAME} five_c coordination_libraries iiwa_aacal)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${FIVE_C_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${COORDINATON_LIBRARIES_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${IIWA_AACAL_INCLUDE_DIR}>
)

target_link_directories(${PROJECT_NAME}
    PUBLIC
        ${COORDINATON_LIBRARIES_BUILD_DIR}
        ${IIWA_AACAL_BUILD_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    five_c
    coordination_libraries
    iiwa_aacal
)

if(STAND_ALONE)
    add_executable(iiwa_demo_mediated src/demo/iiwa_demo_mediated.cpp)
    target_link_libraries(iiwa_demo_mediated PRIVATE
        ${PROJECT_NAME}
        pthread
        five_c
        coordination_libraries
        iiwa_aacal
        )    
endif()


#############
## Install ##
#############
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
