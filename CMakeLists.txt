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
	set(PLATFORM "UNIX")
	set(CMAKE_C_COMPILER "/usr/bin/gcc")
	set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#	set(CMAKE_C_COMPILER "/usr/bin/clang")
#	set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
endif(UNIX)
if(WIN32)
	set(PLATFORM "PLATFORM_IS_WINDOWS")
endif(WIN32)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_BUILD_TYPE "Release")

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
endif(UNIX)

# set definitions to disable some warning
if(WIN32)
add_definitions(-DWIN32 -D_WINSOCK_DEPRECATED_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
endif(WIN32)
if(UNIX)
add_definitions(-DUNIX)
endif(UNIX)

# set debug and release options
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CONFIGURATION_TYPES Debug Release)

# set include and lib folders
if(UNIX)
set(GENERAL_INCL_DIR "/usr/Aris_Dependent")
set(SYSTEM_LINK_LIB pthread)
set(XENOMAI_INCL_DIR "/usr/xenomai/include")
set(XENOMAI_LINK_DIR "/usr/xenomai/lib")
set(XENOMAI_LINK_LIB native rtdm xenomai)
set(ETHERLAB_INCL_DIR "/opt/etherlab/include")
set(ETHERLAB_LINK_DIR "/opt/etherlab/lib")
set(ETHERLAB_LINK_LIB ethercat rt)
set(XSENS_INCL_DIR "/usr/Aris_Dependent/xsens/include")
set(XSENS_LINK_LIB xdeviceclass xcommunication xstypes)
set(XSENS_LINK_DIR "/usr/Aris_Dependent/xsens/lib")
set(XNNITE_INCL_DIR "/usr/include/nite")#Driver for kinect
set(XNNITE_LINK_DIR "/usr/lib")
set(XNNITE_LINK_LIB )
set(OPENNI_INCL_DIR "/usr/include/ni")#open NI for kinect
set(OPENNI_LINK_DIR "/usr/lib")
set(OPENNI_LINK_LIB OpenNI)
set(ARIS_INCL_DIR ${PROJECT_BINARY_DIR}/common_headers src/Aris_Core src/Aris_DynKer src/Aris_ExpCal src/Aris_Plan src/Aris_Sensor src/Aris_Control)
set(ARIS_LINK_LIB Aris_Plan Aris_Sensor Aris_DynKer Aris_ExpCal Aris_Control Aris_Core)


set(ALL_INCL_DIR ${ARIS_INCL_DIR} ${OPENNI_INCL_DIR} ${XNNITE_INCL_DIR} ${XSENS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR})
set(ALL_LINK_DIR ${ARIS_LINK_DIR} ${OPENNI_LINK_DIR} ${XNNITE_LINK_DIR} ${XSENS_LINK_DIR} ${ETHERLAB_LINK_DIR} ${XENOMAI_LINK_DIR})
set(ALL_LINK_LIB ${ARIS_LINK_LIB} ${OPENNI_LINK_LIB} ${XNNITE_LINK_LIB} ${XSENS_LINK_LIB} ${ETHERLAB_LINK_LIB} ${XENOMAI_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(UNIX)
if(WIN32)
set(GENERAL_INCL_DIR "C:/Aris_Dependent")
set(SYSTEM_LINK_LIB ws2_32 setupapi)#ws2_32 is needed for socket setuaipi is needed for xsens
set(XSENS_INCL_DIR "C:/Aris_Dependent/xsens/include")
set(XSENS_LINK_DIR "C:/Aris_Dependent/xsens/lib")
set(XSENS_LINK_LIB xdeviceclass)
set(ARIS_INCL_DIR ${PROJECT_BINARY_DIR}/common_headers src/Aris_Core src/Aris_DynKer src/Aris_ExpCal src/Aris_Plan src/Aris_Sensor src/Aris_Control)
set(ARIS_LINK_LIB Aris_Plan Aris_Sensor Aris_DynKer Aris_ExpCal Aris_Control Aris_Core)


set(ALL_INCL_DIR ${ARIS_INCL_DIR} ${XSENS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR})
set(ALL_LINK_DIR ${XSENS_LINK_DIR})
set(ALL_LINK_LIB ${ARIS_LINK_LIB} ${XSENS_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(WIN32)


include_directories(${ALL_INCL_DIR})
link_directories(${ALL_LINK_DIR})


# Aris_Core project
set(SOURCE Aris_Socket Aris_Core tinyxml2 Aris_Message Aris_XML)
PREPEND_CPP(FULL_SRC src/Aris_Core ${SOURCE})
PREPEND_H(FULL_H src/Aris_Core ${SOURCE})
add_library(Aris_Core STATIC ${FULL_SRC} ${FULL_H})
set(INCLUDE_HEADER_FILES ${INCLUDE_HEADER_FILES} ${FULL_H})

# Aris_Control project
set(SOURCE Aris_Motion Aris_EtherCat Aris_Pipe)
PREPEND_CPP(FULL_SRC src/Aris_Control ${SOURCE})
PREPEND_H(FULL_H src/Aris_Control ${SOURCE})
add_library(Aris_Control STATIC ${FULL_SRC} ${FULL_H})
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


################################### build tests for Aris ####################################
enable_testing()

add_executable(test_DynKer test/test_DynKer.cpp)
target_link_libraries(test_DynKer ${ALL_LINK_LIB})
add_test(NAME test_DynKer COMMAND test_DynKer)
set_tests_properties (test_DynKer PROPERTIES FAIL_REGULAR_EXPRESSION "failed")




################################### build demos for Aris ####################################
# Make demo projects
set(MAKE_DEMO " ")
if(MAKE_DEMO)

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Core/demo_Core ${SOURCE_FILES})
add_executable(demo_Core ${FULL_SRC})
target_link_libraries(demo_Core ${ALL_LINK_LIB})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Control/demo_Control ${SOURCE_FILES})
add_executable(demo_Control ${FULL_SRC})
target_link_libraries(demo_Control ${ALL_LINK_LIB})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Core/demo_Message ${SOURCE_FILES})
add_executable(demo_Message ${FULL_SRC})
target_link_libraries(demo_Message ${ALL_LINK_LIB})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Core/demo_Socket_Client ${SOURCE_FILES})
add_executable(demo_Socket_Client ${FULL_SRC})
target_link_libraries(demo_Socket_Client ${ALL_LINK_LIB})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Core/demo_Socket_Server ${SOURCE_FILES})
add_executable(demo_Socket_Server ${FULL_SRC})
target_link_libraries(demo_Socket_Server ${ALL_LINK_LIB})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_ExpCal/demo_ExpCal ${SOURCE_FILES})
add_executable(demo_ExpCal ${FULL_SRC})
target_link_libraries(demo_ExpCal ${ALL_LINK_LIB})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_Sensor/demo_Sensor ${SOURCE_FILES})
add_executable(demo_Sensor ${FULL_SRC})
target_link_libraries(demo_Sensor ${ALL_LINK_LIB})

endif(MAKE_DEMO)











############################################ install aris ############################################
if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/Aris")
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/Aris")
endif(WIN32)


install(DIRECTORY sh DESTINATION .)
install(TARGETS Aris_Core Aris_DynKer Aris_ExpCal Aris_Plan Aris_Sensor Aris_Control ARCHIVE DESTINATION lib)
install(FILES ${INCLUDE_HEADER_FILES} DESTINATION include)
install(DIRECTORY src/Aris_Control/resource/ DESTINATION resource/Aris_Control)

############################################ uninstall aris ############################################
if(UNIX)
	add_custom_target(uninstall rm -r /usr/Aris)
endif(UNIX)












