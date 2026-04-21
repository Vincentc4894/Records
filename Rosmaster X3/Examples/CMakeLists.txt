cmake_minimum_required(VERSION 3.22.1)
project(pointCloud)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

#CGAL_Qt6 is needed for the drawing.
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt6)
if (NOT CGAL_FOUND)
  message(FATAL_ERROR "CGAL not found")
endif ()
if(CGAL_Qt6_FOUND)
  #required to use basic_viewer
  add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT6_NO_KEYWORDS)
else()
  message(FATAL_ERROR "qt6 not found")
endif()
#create the executable of the application
create_single_source_cgal_program("pointCloud.cpp")
if(CGAL_Qt6_FOUND)
  #link it with the required CGAL libraries
  target_link_libraries(pointCloud PUBLIC CGAL::CGAL_Qt6)
else()
  message(FATAL_ERROR "qt6 not found")
endif()
