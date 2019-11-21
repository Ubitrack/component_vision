project(ubitrack_component_vision)

cmake_minimum_required(VERSION 3.4.0)

include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

include(GNUInstallDirs)
include(cmake/UbitrackConfig)

option(WITH_OPENCL "Enable OpenCL support" ON)

# OpenGL
SET(HAVE_OPENGL 0)
find_package(OpenGL)
IF(OpenGL_FOUND)
  SET(HAVE_OPENGL 1)
  MESSAGE(STATUS "Found OpenGL: ${OpenGL_INCLUDE_DIR} - ${OpenGL_LIBRARIES}")
ENDIF(OpenGL_FOUND)

if(WITH_OPENCL)
	# OpenCL
	SET(HAVE_OPENCL 0)
	find_package(OpenCL)
	IF(OpenCL_FOUND)
	  include_directories(${OpenCL_INCLUDE_DIR})
	  SET(HAVE_OPENCL 1)
	  MESSAGE(STATUS "Found OpenCL: ${OpenCL_INCLUDE_DIR} - ${OpenCL_LIBRARY}")
	ENDIF(OpenCL_FOUND)
endif()

# since we depend on it we now we have utvision:
set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_UTVISION")

add_subdirectory(src/utVisionComponents)
add_subdirectory(src/utVisionIOComponents)


ubitrack_install_utql("doc/utql/*.xml" "doc/utql/*/*.xml"  "doc/utql/*/*/*.xml")
ubitrack_install_doc("doc/utqlDoc/*" "doc/utqlDoc/*/*" "doc/utqlDoc/*/*/*")
ubitrack_install_doc("doc/marker/*" "doc/marker/*/*")
