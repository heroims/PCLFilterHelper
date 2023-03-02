cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME pcl_filter_helper)

set(PRO_SOURCES pcl_filter_helper.h pcl_filter_helper.cpp)

project(${PROJECTNAME})

find_package(PCL 1.8.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECTNAME} SHARED  ${PRO_SOURCES})
add_library(${PROJECTNAME}_static STATIC  ${PRO_SOURCES})

set_target_properties(${PROJECTNAME} PROPERTIES OUTPUT_NAME ${PROJECTNAME})
set_target_properties(${PROJECTNAME}_static PROPERTIES OUTPUT_NAME ${PROJECTNAME})

set_target_properties(${PROJECTNAME} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(${PROJECTNAME}_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

target_link_libraries (${PROJECTNAME} ${PCL_LIBRARIES})
target_link_libraries (${PROJECTNAME}_static ${PCL_LIBRARIES})

target_include_directories(${PROJECTNAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>)
target_include_directories(${PROJECTNAME}_static PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>)