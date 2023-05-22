#==============================================================================
# starling CMake file
#
# Hanno Hildenbrandt 2020
#============================================================================== 
cmake_minimum_required(VERSION 3.16)

project(starling)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})

set(GLSL_VERSION /DGLSL_OPENGL_MAJOR_VERSION=4 /DGLSL_OPENGL_MINOR_VERSION=5)

if (WIN32)
    add_definitions(/DNOMINMAX /DWIN32_LEAN_AND_MEAN /D_CRT_SECURE_NO_WARNINGS ${GLSL_VERSION})
else()
    add_definitions(${GLSL_VERSION})
endif()

# tbb
if (WIN32)
    set_property(TARGET ${TARGET_NAME} PROPERTY VS_PACKAGE_REFERENCES "tbb_4.2.3.1")
endif()

find_package(TBB REQUIRED)


file(GLOB all_SRCS
     "${PROJECT_SOURCE_DIR}/*.cpp"
     "${PROJECT_SOURCE_DIR}/libs/*.hpp"
     "${PROJECT_SOURCE_DIR}/actions/*.hpp"
     "${PROJECT_SOURCE_DIR}/actions/*.cpp"
     "${PROJECT_SOURCE_DIR}/agents/*.hpp"
     "${PROJECT_SOURCE_DIR}/agents/*.cpp"
     "${PROJECT_SOURCE_DIR}/analysis/*.hpp"
     "${PROJECT_SOURCE_DIR}/analysis/*.cpp"
     "${PROJECT_SOURCE_DIR}/species/*.hpp"
     "${PROJECT_SOURCE_DIR}/species/*.cpp"
     "${PROJECT_SOURCE_DIR}/states/*.hpp"
     "${PROJECT_SOURCE_DIR}/states/*.cpp"
     "${PROJECT_SOURCE_DIR}/stress/*.hpp"
     "${PROJECT_SOURCE_DIR}/stress/*.cpp"
     "${PROJECT_SOURCE_DIR}/model/*.hpp"
     "${PROJECT_SOURCE_DIR}/model/*.cpp"
     "${PROJECT_SOURCE_DIR}/simgl/*.cpp"
     "${PROJECT_SOURCE_DIR}/libs/glad/*.c"
     "${PROJECT_SOURCE_DIR}/libs/glsl/*.cpp"
     "${PROJECT_SOURCE_DIR}/starling_model.cpp"
)

add_executable(starling ${all_SRCS})
target_include_directories(starling PRIVATE
     "${PROJECT_SOURCE_DIR}"
     "${PROJECT_SOURCE_DIR}/libs"
     "${PROJECT_SOURCE_DIR}/libs/glad/4_5"
     "${PROJECT_SOURCE_DIR}/model"
     "${CURRENT_SOURCE_DIR}"
     "${TBB}"
)

target_link_libraries(starling ${CMAKE_DL_LIBS} PUBLIC TBB::tbb)
install(TARGETS starling 
        CONFIGURATIONS Release
        RUNTIME DESTINATION bin/Release)
install(TARGETS starling 
        CONFIGURATIONS Debug
        RUNTIME DESTINATION bin/Debug)
