cmake_minimum_required(VERSION 2.8 FATAL_ERROR)



set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(final)

find_package(PCL 1.2 REQUIRED)

#Bring the headers, such as Student.h into the project
include_directories(include)


find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(SOURCES src/main.cpp)
file(GLOB SOURCES "src/*.cpp")

add_executable(main ${SOURCES})
target_link_libraries ( main ${PCL_LIBRARIES} ${catkin_LIBRARIES} )

install(TARGETS main DESTINATION /usr/lib)
