cmake_minimum_required(VERSION 2.8)

# add prefix to every string in a variable
FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

FUNCTION(PREPEND_CPP var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.cpp")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_CPP)

FUNCTION(PREPEND_H var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.h")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_H)

project(ARIS)

# set the platform.h, which define the OS platform
if(UNIX)
	set(PLATFORM "PLATFORM_IS_LINUX")
	set(CMAKE_C_COMPILER "/usr/bin/gcc")
	set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#	set(CMAKE_C_COMPILER "/usr/bin/clang")
#	set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
endif(UNIX)
if(WIN32)
	set(PLATFORM "PLATFORM_IS_WINDOWS")
endif(WIN32)

configure_file(
	"${PROJECT_SOURCE_DIR}/common_headers/Platform.h.in" 
	"${PROJECT_BINARY_DIR}/common_headers/Platform.h")
set(INCLUDE_HEADER_FILES "${PROJECT_BINARY_DIR}/common_headers/Platform.h")

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
endif(UNIX)

# set dependency include folder
if(UNIX)
set(XENOMAI_INCLUDE_DIRECTORIES "/usr/xenomai/include")
set(ETHERLAB_INCLUDE_DIRECTORIES "/opt/etherlab/include")
set(LAPACK_INCLUDE_DIRECTORIES "/usr/lapack/include")
endif(UNIX)
if(WIN32)
set(LAPACK_INCLUDE_DIRECTORIES "C:/lapack/include/")
endif(WIN32)
set(ARIS_COMMOM_HEADERS ${PROJECT_BINARY_DIR}/common_headers)

include_directories("src/Aris_Core")
include_directories("src/Aris_ExpCal")
include_directories("src/Aris_DynKer")
include_directories("src/Aris_Plan")
include_directories("src/Aris_Control")
include_directories(${ARIS_COMMOM_HEADERS})
include_directories(${ETHERLAB_INCLUDE_DIRECTORIES})
include_directories(${XENOMAI_INCLUDE_DIRECTORIES})
include_directories(${LAPACK_INCLUDE_DIRECTORIES})

if(UNIX)
link_directories(/usr/lapack/lib)
endif(UNIX)
if(WIN32)
link_directories(C:/lapack/lib)
endif(WIN32)

# Aris_Core project
set(SOURCE Aris_Socket Aris_Core tinyxml2 Aris_Message Aris_XML)
PREPEND_CPP(FULL_SRC src/Aris_Core ${SOURCE})
PREPEND_H(FULL_H src/Aris_Core ${SOURCE})
add_library(Aris_Core STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# Aris_ExpCal project
set(SOURCE Aris_ExpCal)
PREPEND_CPP(FULL_SRC src/Aris_ExpCal ${SOURCE})
PREPEND_H(FULL_H src/Aris_ExpCal ${SOURCE})
add_library(Aris_ExpCal STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# Aris_DynKer project
set(SOURCE Aris_DynKer Aris_DynModel Aris_DynModelBase)
PREPEND_CPP(FULL_SRC src/Aris_DynKer ${SOURCE})
PREPEND_H(FULL_H src/Aris_DynKer ${SOURCE})
add_library(Aris_DynKer STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# Aris_Plan project
set(SOURCE Aris_Plan)
PREPEND_CPP(FULL_SRC src/Aris_Plan ${SOURCE})
PREPEND_H(FULL_H src/Aris_Plan ${SOURCE})
add_library(Aris_Plan STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# Aris_Control project only in linux
if(UNIX)
set(SOURCE_FILES Aris_Control.cpp Aris_ControlData.cpp Aris_Sensor.cpp Aris_Device.cpp Aris_SysBase.cpp )
PREPEND(FULL_SRC src/Aris_Control ${SOURCE_FILES})
add_library(Aris_Control STATIC ${FULL_SRC})
set(HEADER_FILES Aris_Control.h Aris_ControlData.h)
PREPEND(FULL_SRC ${PROJECT_SOURCE_DIR}/src/Aris_Control ${HEADER_FILES})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_SRC})
endif(UNIX)

################################### build demos for aris_Core ####################################
# Make demo projects
set(MAKE_DEMO " ")
if(MAKE_DEMO)

# Set link labraries
if(WIN32)
set(SYSTEM_LINK_LIBS ws2_32)
set(LAPACK_LINK_LIBS liblapacke liblapack cblas libblas)
set(ARIS_LINK_LIBS Aris_Plan Aris_ExpCal Aris_DynKer Aris_Core)
endif(WIN32)
if(UNIX)
set(SYSTEM_LINK_LIBS pthread gfortran)
set(LAPACK_LINK_LIBS lapacke lapack cblas refblas)
set(ARIS_LINK_LIBS Aris_Control Aris_Plan Aris_ExpCal Aris_DynKer Aris_Core)
endif(UNIX)

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Core/demo_Core ${SOURCE_FILES})
add_executable(demo_Core ${FULL_SRC})
target_link_libraries(demo_Core ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${SYSTEM_LINK_LIBS})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Core/demo_Message ${SOURCE_FILES})
add_executable(demo_Message ${FULL_SRC})
target_link_libraries(demo_Message ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${SYSTEM_LINK_LIBS})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Core/demo_Socket_Client ${SOURCE_FILES})
add_executable(demo_Socket_Client ${FULL_SRC})
target_link_libraries(demo_Socket_Client ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${SYSTEM_LINK_LIBS})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Core/demo_Socket_Server ${SOURCE_FILES})
add_executable(demo_Socket_Server ${FULL_SRC})
target_link_libraries(demo_Socket_Server ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${SYSTEM_LINK_LIBS})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_ExpCal/demo_ExpCal ${SOURCE_FILES})
add_executable(demo_ExpCal ${FULL_SRC})
target_link_libraries(demo_ExpCal ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${SYSTEM_LINK_LIBS})

endif(MAKE_DEMO)











############################################ install aris ############################################
if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/Aris")
	install(TARGETS Aris_Control ARCHIVE DESTINATION lib)
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/Aris")
	set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} src/Aris_Control/Aris_ControlData.h)
endif(WIN32)



install(TARGETS Aris_Core Aris_DynKer Aris_ExpCal Aris_Plan ARCHIVE DESTINATION lib)
install(FILES ${INCLUDE_HEADER_FILES} DESTINATION include)
install(DIRECTORY src/Aris_Control/resource/ DESTINATION resource/Aris_Control)



############################################ uninstall aris ############################################
if(UNIX)
	add_custom_target(uninstall rm -r /usr/Aris)
endif(UNIX)












