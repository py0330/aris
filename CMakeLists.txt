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

configure_file("${PROJECT_SOURCE_DIR}/common_headers/Platform.h.in" "${PROJECT_BINARY_DIR}/common_headers/Platform.h")
set(INCLUDE_HEADER_FILES "${PROJECT_BINARY_DIR}/common_headers/Platform.h")

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_BUILD_TYPE "Release")

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
endif(UNIX)

# set debug and release options
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CONFIGURATION_TYPES Debug Release)

# set include and lib folders
if(UNIX)
set(GENERAL_INCLUDE_DIR "/usr/Aris_Dependent")
set(SYSTEM_LINK_LIBS pthread)
set(ARIS_INCLUDE_DIR ${PROJECT_BINARY_DIR}/common_headers src/Aris_Core src/Aris_DynKer src/Aris_ExpCal src/Aris_Plan src/Aris_Sensor src/Aris_Control2 src/Aris_Control)
set(ARIS_LINK_LIBS Aris_Control Aris_Plan Aris_Sensor Aris_DynKer Aris_ExpCal Aris_Control2 Aris_Core)
set(XNNITE_INCLUDE_DIR "/usr/include/nite")#Driver for kinect
set(XNNITE_LINK_DIR "/usr/lib")
set(XNNITE_LINK_LIBS libXnVNite_1_5_2.so)
set(OPENNI_INCLUDE_DIR "/usr/include/ni")#open NI for kinect
set(OPENNI_LINK_DIR "/usr/lib")
set(OPENNI_LINK_LIBS OpenNI libOpenNI.so)
set(XENOMAI_INCLUDE_DIR "/usr/xenomai/include")
set(XENOMAI_LINK_DIR "/usr/xenomai/lib")
set(XENOMAI_LINK_LIBS native rtdm xenomai)
set(ETHERLAB_INCLUDE_DIR "/opt/etherlab/include")
set(ETHERLAB_LINK_DIR "/opt/etherlab/lib")
set(ETHERLAB_LINK_LIBS ethercat rt)
set(XSENS_INCLUDE_DIR "/usr/Aris_Dependent/xsens/include")
set(XSENS_LINK_LIBS xdeviceclass xcommunication xstypes)
set(XSENS_LINK_DIR "/usr/Aris_Dependent/xsens/lib")


set(ALL_INCLUDE_DIR ${GENERAL_INCLUDE_DIR} ${ARIS_INCLUDE_DIR} ${ETHERLAB_INCLUDE_DIR} ${XENOMAI_INCLUDE_DIR} ${OPENNI_INCLUDE_DIR} ${XNNITE_INCLUDE_DIR}  ${XSENS_INCLUDE_DIR})
set(ALL_LINK_DIR ${ARIS_LINK_DIR} ${XSENS_LINK_DIR} ${ETHERLAB_LINK_DIR} ${OPENNI_LINK_DIR} ${XNNITE_LINK_DIR} ${XENOMAI_LINK_DIR})
set(ALL_LINK_LIBS ${ARIS_LINK_LIBS} ${XSENS_LINK_LIBS} ${ETHERLAB_LINK_LIBS} ${XENOMAI_LINK_LIBS} ${OPENNI_LINK_LIBS} ${OPENNI_LINK_LIBS} ${SYSTEM_LINK_LIBS})
endif(UNIX)
if(WIN32)
set(GENERAL_INCLUDE_DIR "C:/Aris_Dependent")
set(SYSTEM_LINK_LIBS ws2_32 setupapi)
set(ARIS_INCLUDE_DIR ${PROJECT_BINARY_DIR}/common_headers src/Aris_Core src/Aris_DynKer src/Aris_ExpCal src/Aris_Plan src/Aris_Sensor src/Aris_Control2)
set(ARIS_LINK_LIBS Aris_Plan Aris_Sensor Aris_DynKer Aris_ExpCal Aris_Control2 Aris_Core)
set(XSENS_INCLUDE_DIR "C:/Aris_Dependent/xsens/include")
set(XSENS_LINK_DIR "C:/Aris_Dependent/xsens/lib")
set(XSENS_LINK_LIBS xdeviceclass)


set(ALL_INCLUDE_DIR ${GENERAL_INCLUDE_DIR} ${ARIS_INCLUDE_DIR} ${ETHERLAB_INCLUDE_DIR} ${XENOMAI_INCLUDE_DIR} ${XSENS_INCLUDE_DIR})
set(ALL_LINK_DIR ${XSENS_LINK_DIR})
set(ALL_LINK_LIBS ${ARIS_LINK_LIBS} ${XSENS_LINK_LIBS} ${SYSTEM_LINK_LIBS})
endif(WIN32)


include_directories(${ALL_INCLUDE_DIR})
link_directories(${ALL_LINK_DIR})


# Aris_Core project
set(SOURCE Aris_Socket Aris_Core tinyxml2 Aris_Message Aris_XML)
PREPEND_CPP(FULL_SRC src/Aris_Core ${SOURCE})
PREPEND_H(FULL_H src/Aris_Core ${SOURCE})
add_library(Aris_Core STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# Aris_Control project
set(SOURCE Aris_Motion Aris_EtherCat Aris_ControlData)
PREPEND_CPP(FULL_SRC src/Aris_Control2 ${SOURCE})
PREPEND_H(FULL_H src/Aris_Control2 ${SOURCE})
add_library(Aris_Control2 STATIC ${FULL_SRC} ${FULL_H})
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

# Aris_Sensor project
if(UNIX)
set(SOURCE Aris_Sensor Aris_IMU Aris_Vision)
endif(UNIX)
if(WIN32)
set(SOURCE Aris_Sensor Aris_IMU)
endif(WIN32)
PREPEND_CPP(FULL_SRC src/Aris_Sensor ${SOURCE})
PREPEND_H(FULL_H src/Aris_Sensor ${SOURCE})
add_library(Aris_Sensor STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# Aris_Control project only in linux
if(UNIX)
set(SOURCE_FILES Aris_Control.cpp Aris_ControlData.cpp Aris_Control_Sensor.cpp Aris_Device.cpp Aris_SysBase.cpp )
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

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Core/demo_Core ${SOURCE_FILES})
add_executable(demo_Core ${FULL_SRC})
target_link_libraries(demo_Core ${ALL_LINK_LIBS})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Control/demo_Control ${SOURCE_FILES})
add_executable(demo_Control ${FULL_SRC})
target_link_libraries(demo_Control ${ALL_LINK_LIBS})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Core/demo_Message ${SOURCE_FILES})
add_executable(demo_Message ${FULL_SRC})
target_link_libraries(demo_Message ${ALL_LINK_LIBS})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Core/demo_Socket_Client ${SOURCE_FILES})
add_executable(demo_Socket_Client ${FULL_SRC})
target_link_libraries(demo_Socket_Client ${ALL_LINK_LIBS})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Core/demo_Socket_Server ${SOURCE_FILES})
add_executable(demo_Socket_Server ${FULL_SRC})
target_link_libraries(demo_Socket_Server ${ALL_LINK_LIBS})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_ExpCal/demo_ExpCal ${SOURCE_FILES})
add_executable(demo_ExpCal ${FULL_SRC})
target_link_libraries(demo_ExpCal ${ALL_LINK_LIBS})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Sensor/demo_Sensor ${SOURCE_FILES})
add_executable(demo_Sensor ${FULL_SRC})
target_link_libraries(demo_Sensor ${ALL_LINK_LIBS})

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



install(TARGETS Aris_Core Aris_DynKer Aris_ExpCal Aris_Plan Aris_Sensor ARCHIVE DESTINATION lib)
install(FILES ${INCLUDE_HEADER_FILES} DESTINATION include)
install(DIRECTORY src/Aris_Control/resource/ DESTINATION resource/Aris_Control)



############################################ uninstall aris ############################################
if(UNIX)
	add_custom_target(uninstall rm -r /usr/Aris)
endif(UNIX)












