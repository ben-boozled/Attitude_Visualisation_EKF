cmake_minimum_required(VERSION 3.16.0)
project(Attitude_Visualiser VERSION 0.1.0 LANGUAGES C CXX)

# ==================== Dependencies ====================
include(FetchContent)

# 1. SDL3 - for attitude visualisation
set(SDL3_VERSION 3.4.16)

FetchContent_Declare(SDL3
    URL https://github.com/libsdl-org/SDL/releases/download/release-${SDL3_VERSION}/SDL3-${SDL3_VERSION}.tar.gz
    FIND_PACKAGE_ARGS ${SDL3_VERSION}
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/SDL3
)

# 2. SDL3_ttf - SDL3 extension library to render fonts
set(SDL3_ttf_VERSION 3.2.2)

FetchContent_Declare(SDL3_ttf
    GIT_REPOSITORY https://github.com/libsdl-org/SDL_ttf.git
    GIT_TAG release-${SDL3_ttf_VERSION}
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/SDL3_ttf
    GIT_SHALLOW TRUE # download only the latest commits instead of the entire repo history
    GIT_SUBMODULES_RECURSIVE TRUE # explicitly pull freetype/harbuzz/plutosvg under the "external" subdirectory
)

# tells SDL3_ttf to compile freetype/harbuzz/plutosvg from its own "external" subdirectory
# instead of looking for these dependencies on the current device's system
set(SDLTTF_VENDORED ON CACHE BOOL "" FORCE) 

# 3. Download dependencies based on FetchContent_Declare() if dependencies are not found
FetchContent_MakeAvailable(SDL3 SDL3_ttf) 

# 4. Ensure that other required packages can be found
find_package(Threads REQUIRED)

# ==================== Project Configurations ====================
set(SOURCE_FILES
    src/main.cpp
    src/text_renderer.cpp
    src/graph_renderer.cpp
    src/serial_port.cpp
    src/utils.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE
                      SDL3::SDL3
                      SDL3_ttf::SDL3_ttf
                      Threads::Threads
)                       

target_include_directories(${PROJECT_NAME} PRIVATE include) # add the include folder to the search path

# ===== Post-build configurations =====

# 1. Copies SDL3.dll and SDL3_ttf.dll to the same directory as the project executable (Window only)
if(WIN32)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy_if_different
                       $<TARGET_FILE:SDL3::SDL3> # generator expression that resolves at build time to the actual path of SDL3.dll
                       $<TARGET_FILE:SDL3_ttf::SDL3_ttf> # generator expression that resolves at build time to the actual path of SDL3_ttf.dll
                       $<TARGET_FILE_DIR:${PROJECT_NAME}>   # generator expression that resolves at build time to the actual directory of Attitude_Visualiser.exe
                       COMMENT "Copying SDL3.dll and SDL3_ttf.dll to the same directory as Attitude_Visualiser.exe")
endif()

# 2. Copies font assets folder to be beside the project executable
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                   ${CMAKE_CURRENT_SOURCE_DIR}/assets
                   $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets
                   COMMENT "Copying assets/fonts to the same directory as Attitude_Visualiser.exe")