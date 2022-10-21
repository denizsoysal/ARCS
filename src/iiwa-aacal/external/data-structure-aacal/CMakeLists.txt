cmake_minimum_required(VERSION 3.10.0)
project(data-structure-aacal VERSION 0.0.1 LANGUAGES C CXX)

# Get the project name in capital letters
# for the lib namespace
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_CAPS)

# Locate system library graphviz
find_path(GRAPHVIZ_INCLUDE_DIR NAMES graphviz/cgraph.h)
message("")
if (GRAPHVIZ_INCLUDE_DIR)
    set(GRAPHVIZ_FOUND TRUE)
    message("Graphviz found")
else ()
    set(GRAPHVIZ_FOUND FALSE)
    message("Graphviz not found")
endif ()
set(GRAPHVIZ_INCLUDE_DIRS ${GRAPHVIZ_INCLUDE_DIR} ${GRAPHVIZ_INCLUDE_DIR}/graphviz)

###########
## Build ##
###########
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
include(ExternalProject)

# AACAL
ExternalProject_Add(
  aacalib
  GIT_REPOSITORY https://kelo:giixwXQQAz_vFkKtx8Cs@gitlab.kuleuven.be/u0144428/algorithm_and_activity_component_architecture.git
  GIT_TAG feature/configuration_file
  TIMEOUT 5
  PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
  INSTALL_COMMAND ""
)

ExternalProject_Get_Property(aacalib source_dir)
ExternalProject_Get_Property(aacalib binary_dir)
set(ReadFileIncludeDir ${source_dir}/external/read_parameter_struct/include)
set(ReadFileLibBuildDir ${binary_dir}/external/read_parameter_struct) # Need to be able to link
endif()

add_library(data-structure-aacal SHARED
    src/kelo_wheel.c
    src/kelo_tricycle.c)
     
target_include_directories(data-structure-aacal
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${ReadFileIncludeDir}>
)

target_link_directories(data-structure-aacal
    PUBLIC
        ${ReadFileLibBuildDir}
)


target_link_libraries(data-structure-aacal 
    readfile
)

#############
## Install ##
#############
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX})


