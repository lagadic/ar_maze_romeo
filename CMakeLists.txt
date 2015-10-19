cmake_minimum_required(VERSION 2.8)
project(solveMaze_Romeo)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(qibuild)


find_package(PythonLibs 2.7 REQUIRED)
IF (PYTHONLIBS_FOUND)
  include_directories(${PYTHON_INCLUDE_DIRS})
 #target_link_libraries (OFW ${PYTHON_LIBRARIES})
ENDIF (PYTHONLIBS_FOUND)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find the Panda3D modules
find_package(Panda3D REQUIRED)


#--------------------------------------------------------------------
# Add ViSP dependencies
#--------------------------------------------------------------------
FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

FIND_PACKAGE (Threads)


include_directories(${PROJECT_SOURCE_DIR}/p3util)
include_directories(${PROJECT_BINARY_DIR}/include)




qi_create_lib(p3util
  SRC
    p3util/cOnscreenText.cpp
    p3util/cOnscreenText.h
)


qi_stage_lib(p3util)


#add_library (p3util
#    p3util/cOnscreenText.cpp
#    p3util/cOnscreenText.h

#)



add_subdirectory (ball_in_maze)




