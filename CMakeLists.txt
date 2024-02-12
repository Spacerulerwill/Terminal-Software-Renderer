cmake_minimum_required(VERSION 3.22.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(TerminalSoftwareRenderer VERSION 1.0)

add_executable(${PROJECT_NAME} 
    src/main.cpp
)

add_subdirectory(submodules/fmt)

target_include_directories(${PROJECT_NAME} 
    SYSTEM PRIVATE submodules/fmt/include
    PUBLIC include
)

target_link_directories(${PROJECT_NAME}
    PRIVATE submodules/fmt/src
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC fmt
)

# copy our resources to executable location
add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/res $<TARGET_FILE_DIR:${PROJECT_NAME}>)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /wd4996 /external:W0)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()
