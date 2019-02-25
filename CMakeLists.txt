cmake_minimum_required(VERSION 2.8)

#set  findpackage files path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

project(Monocular_Measure)

add_definitions(-std=c++11)

find_package(OpenCV 3.0 REQUIRED)

if(NOT OpenCV_FOUND)
	message(FATAL_ERROR "Caffe Not Found!")
endif (NOT OpenCV_FOUND)

include_directories(
${PROJECT_SOURCE_DIR}/include
add_subdirecroty(sources)
${OpenCV_INCLUDE_DIRS}
)
#add all files 
aux_source_directory(. DIRSRCS)

#create dir in project
file(GLOB CURRENT_INCLUDE include/*.h)
file(GLOB CURRENT_SOURCE  sources/*.cpp)

#add subdir  subdir must have cmakelist.txt 
#add_subdirectory(sources)

add_executable(${PROJECT_NAME} ${DIRSRCS} ${CURRENT_INCLUDE} ${CURRENT_SOURCE})
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
)
