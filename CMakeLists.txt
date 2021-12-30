cmake_minimum_required(VERSION 3.0)

#工程名字
project(DBSCAN)

#第三方库
find_package(PCL REQUIRED)

#头文件路径
include_directories(${PCL_INCLUDE_DIRS})

#define the PCL_VER_1_6_OR_OLDER preprocessor to compile qPCL with older versions of PCL
if ( PCL_VERSION VERSION_LESS  1.7 ) # VERSION_GREATER Works just like "greater or equal"
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS PCL_VER_1_6_OR_OLDER )
endif()

#链接库
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

#生成可执行的文件
add_executable(${PROJECT_NAME} main.cpp)

#指定目标在运行时要寻找的链接库
target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES})


