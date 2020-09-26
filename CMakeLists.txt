cmake_minimum_required (VERSION 3.12)
#cmake_policy(SET CMP0015 NEW)
file(GLOB SRCS src/*.cpp lefdef_util/src/*/*.cpp)
project (ObfusX)
# Uncomment the following line to use wire lifting instead of via perturbation
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLIFT")
set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAG} "-no-pie")
find_package (Python3 COMPONENTS Development)
add_executable(main ${SRCS})

#link_directories(lefdef_util/lib/linux)

target_link_libraries(main 
	${Python3_LIBRARIES}
	${CMAKE_SOURCE_DIR}/lefdef_util/lib/linux/liblef.a
	${CMAKE_SOURCE_DIR}/lefdef_util/lib/linux/libdef.a)
target_include_directories(main PUBLIC 
	${Python3_INCLUDE_DIRS} 
	src
	lefdef_util/src
	lefdef_util/src/include 
	lefdef_util/src/common
	lefdef_util/src/util)
set(CMAKE_BUILD_TYPE "Release")
