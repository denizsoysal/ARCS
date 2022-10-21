cmake_minimum_required(VERSION 3.1)
project(readfile)

if (CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DDEBUG)
endif (CMAKE_BUILD_TYPE MATCHES Debug)

if(NOT DEFINED CJSONLibIncludeDir)
    add_subdirectory(external/cJSON)
    set(CJSONLibIncludeDir ${CMAKE_SOURCE_DIR}/external/cJSON)
endif()

add_library(readfile STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/read_file.c 
)
  
target_include_directories(readfile PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CJSONLibIncludeDir}
) 

target_link_libraries(readfile 
    cjson
)

# Executable with all subcomponents
add_executable(read_test 
    example/main.c 
    example/test_struct.c   
) 

target_link_libraries(read_test 
    readfile
)

set(EXECUTABLE_OUTPUT_PATH     
    ${CMAKE_CURRENT_SOURCE_DIR}/example)

install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS readfile LIBRARY DESTINATION lib )
    