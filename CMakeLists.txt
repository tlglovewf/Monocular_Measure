cmake_minimum_required(VERSION 2.8)
project(Monocular_Measure)

add_definitions(-std=c++11)

find_package(OpenCV 3.0 REQUIRED)

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
