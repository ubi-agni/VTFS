#*********************************************************************
#**                                                                 **
#** File   : CMakeLists.txt                                         **
#** Authors: Qiang Li                                               **
#**                                                                 **
#**                                                                 **
#** GNU LESSER GENERAL PUBLIC LICENSE                               **
#** This file may be used under the terms of the GNU Lesser General **
#** Public License version 3.0 as published by the                  **
#**                                                                 **
#** Free Software Foundation and appearing in the file LICENSE.LGPL **
#** included in the packaging of this file.  Please review the      **
#** following information to ensure the license requirements will   **
#** be met: http://www.gnu.org/licenses/lgpl-3.0.txt                **
#**                                                                 **
#*********************************************************************

cmake_minimum_required(VERSION 2.8.2)

# this sets the projects name
project(VTFS)

# the projects version number
set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 1)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(BUILD_SHARED_LIBS TRUE)

#************************************************************************
#  RPath settings                                                       *
#************************************************************************
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# some output
message(STATUS "")
message(STATUS "==============================================================")
message(STATUS "Configuring ${PROJECT_NAME}:")

# checking build type, setting to release when not set
IF(NOT CMAKE_BUILD_TYPE)
  MESSAGE(STATUS "No Specific build type specified: using Release")
  SET(CMAKE_BUILD_TYPE "Release")
ENDIF()

# some compiler flags for the different build settings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC -std=c++0x -DQT_NO_KEYWORDS -DCBF_NDEBUG")
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

#==============================================================================
# external libs
#==============================================================================

## search for icl
#find_package(ICL REQUIRED HINTS ${ICL_ROOT})
#add_definitions(${ICL_CFLAGS})
#add_definitions(${ICL_DEFINITIONS})
#include_directories(${ICL_INCLUDE_DIRS})

# search for eigen
find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
        include_directories(${EIGEN3_INCLUDE_DIR})
endif()
MESSAGE(STATUS ${EIGEN3_INCLUDE_DIR})

find_package(CBF REQUIRED HINTS ${CBF_ROOT})
include_directories(${CBF_INCLUDE_DIRS})
MESSAGE(STATUS "cbf_INCLUDE_DIR: ${CBF_INCLUDE_DIRS}")

find_package(orocos_kdl REQUIRED HINTS ${orocos_kdl_ROOT})
include_directories(${orocos_kdl_INCLUDE_DIRS})
MESSAGE(STATUS "orocos_kdl_INCLUDE_DIR: ${orocos_kdl_INCLUDE_DIRS}")

# rsb
FIND_PACKAGE(RSB REQUIRED HINTS ${RSB_ROOT})
IF(RSB_FOUND)
        INCLUDE_DIRECTORIES(${RSB_INCLUDE_DIRS})
        ADD_DEFINITIONS(${RSB_DEFINITIONS})
        ADD_DEFINITIONS(${RSB_CFLAGS})
        LINK_DIRECTORIES(${RSB_LIBRARY_DIRS})
ENDIF()

# rst
FIND_PACKAGE(RST REQUIRED sandbox HINTS ${RST_ROOT})
IF(RST_FOUND)
        INCLUDE_DIRECTORIES(${RST_INCLUDE_DIRS})
        ADD_DEFINITIONS(${RST_CFLAGS})
        LINK_DIRECTORIES(${RST_LIBRARY_DIRS})
ENDIF()

## Find catkin macros and libraries
find_package(catkin QUIET COMPONENTS
  tf
  sensor_msgs
  roscpp
  tf_conversions
  cd_dynamics
  )
if(catkin_FOUND)
  # declare this as a catkin package
  add_definitions(-DHAVE_ROS)
  include_directories(${catkin_INCLUDE_DIRS})
else()
  message(WARNING "ROS environment not found, building without ROS support")
endif()

find_library(OPENKC_LIB openkcfri HINTS ${CMAKE_PREFIX_PATH}/../lib)
find_file(OPENKC_INCLUDE fri_okc_comm.h HINTS ${CMAKE_PREFIX_PATH}/../include)
get_filename_component(OPENKC_INCLUDE_DIRS ${OPENKC_INCLUDE} PATH)
include_directories("${OPENKC_INCLUDE_DIRS}")


set(CORE_LIBS ${OPENKC_LIB} ${CBF_LIBRARIES} ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES}
  ${RSB_LIBRARIES} ${RST_LIBRARIES} )
message("CORE_LIBS" ${CORE_LIBS})



#==============================================================================

# message(STATUS "Looking for doxygen")
# find_program(DOXYGEN_BIN NAMES doxygen)
# if(NOT "${DOXYGEN_BIN}" STREQUAL "DOXYGEN_BIN-NOTFOUND")
#   message(STATUS "found Doxygen, a documentation can be built with 'make doc'")
#   add_custom_target(doc
#     COMMAND ${DOXYGEN_BIN} ${PROJECT_BINARY_DIR}/Doxyfile
#     COMMENT "Creating html and tex documentation in doc/"
#   )
#   install(
#     DIRECTORY ${PROJECT_BINARY_DIR}/doc/html
#     DESTINATION "share/doc/${PROJECT_NAME}/"
#     DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ WORLD_EXECUTE OWNER_EXECUTE GROUP_EXECUTE
#     FILE_PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE
#  )
# else()
#   message(STATUS "  doxygen not found, documentation will not be available")
# endif()

configure_file(
  "${PROJECT_SOURCE_DIR}/Doxyfile.in"
  "${PROJECT_BINARY_DIR}/Doxyfile"
)

include_directories(src)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(app)

install(DIRECTORY etc/
   DESTINATION etc/)

