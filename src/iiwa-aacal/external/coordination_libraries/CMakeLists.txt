cmake_minimum_required(VERSION 3.10.0)
project(coordination-libs VERSION 0.0.1 LANGUAGES C CXX)

# Get the project name in capital letters
# for the lib namespace
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_CAPS)

# Options. Turn on with 'cmake -DBUILD_TESTING=ON'.
option(BUILD_TESTING "Build all tests." OFF)
option(BUILD_EXAMPLES "Build all examples." OFF)

# CMake doesn't provide a GraphVIZ module so we add it locally
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
find_package(Graphviz REQUIRED)

###########
## Build ##
###########
message(STATUS "CoordLibIncludeDir (inside) ${CMAKE_SOURCE_DIR}/coordination_libraries/include")
if(NOT DEFINED UTHashIncludeDir)
    set(UTHashIncludeDir ${CMAKE_SOURCE_DIR}/external/uthash/include)
endif()

add_library(${PROJECT_NAME} SHARED
    src/petrinet.c
    src/DAG.c
    src/lcsm.c
    src/FSM.c
    src/top-level_4_state_lcsm.c
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${GRAPHVIZ_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${UTHashIncludeDir}>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        ${GRAPHVIZ_CDT_LIBRARY}
        ${GRAPHVIZ_CGRAPH_LIBRARY}
)

##############
## Examples ##
##############

#if(BUILD_EXAMPLES)
#
#  add_subdirectory(examples)
#
#endif(BUILD_EXAMPLES)

#############
## Testing ##
#############

# DAG test executable
add_executable(test_DAG examples/test_DAG.cpp)
target_link_libraries(test_DAG PRIVATE ${PROJECT_NAME})

# DAG test executable
add_executable(test_fsm examples/test_FSM.c)
target_link_libraries(test_fsm PUBLIC ${PROJECT_NAME})


#############
## Install ##
#############

install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS ${PROJECT_NAME}  LIBRARY DESTINATION lib )

