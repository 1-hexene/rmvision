# basic set
cmake_minimum_required(VERSION 3.10)
project(RM_Vision)
set(CMAKE_CXX_STANDARD 11)

# find opencv package
find_package(OpenCV REQUIRED)

# add new source file
set(SOURCES Src/main.cpp Src/ArmorParam.cpp Src/armorDescriptor.cpp)

#include head file
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(./Inc)
include_directories(/opt/MVS/include)
LINK_DIRECTORIES(/opt/MVS/lib/aarch64/)
add_executable(RM_Project ${SOURCES})

target_link_libraries(RM_Project ${OpenCV_LIBS})
