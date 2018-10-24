cmake_minimum_required(VERSION 3.12)

project(aris VERSION 1.0.0)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_CXX_STANDARD 17)

# set definitions to disable some warning
if(WIN32)
	add_definitions(-DWIN32 -D_WINSOCK_DEPRECATED_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
endif(WIN32)
if(UNIX)
	add_definitions(-DUNIX)
endif(UNIX)

# set debug and release options
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CONFIGURATION_TYPES Debug Release)

# set include and lib folders
if(UNIX)
	set(RELY_LINK_LIB pthread stdc++fs)#pthread is needed for concurrency

	option (USE_XENOMAI "Use Xenomai RT system" OFF)
	if(USE_XENOMAI)
		message("using xenomai")
		add_definitions(-DUSE_XENOMAI)
		set(RELY_INCL_DIR "/usr/xenomai/include" ${RELY_INCL_DIR})
		set(RELY_LINK_DIR "/usr/xenomai/lib" ${RELY_LINK_DIR})
		set(RELY_LINK_LIB native rtdm xenomai ${RELY_LINK_LIB})
	endif(USE_XENOMAI)

	option (USE_ETHERLAB "Use Etherlab Ethercat drivers" OFF)
	if(USE_ETHERLAB)
		message("using etherlab")
		add_definitions(-DUSE_ETHERLAB)
		set(RELY_INCL_DIR "/opt/etherlab/include" ${RELY_INCL_DIR})
		set(RELY_LINK_DIR "/opt/etherlab/lib" ${RELY_LINK_DIR})
		set(RELY_LINK_LIB ethercat_rtdm ${RELY_LINK_LIB})
	endif(USE_ETHERLAB)
endif(UNIX)
if(WIN32)
	set(RELY_LINK_LIB "ws2_32")#ws2_32 is needed for socket
endif(WIN32)

set(ARIS_INCL_DIR src/ src/aris_robot src/aris_server src/aris_sensor src/aris_plan src/aris_dynamic src/aris_control src/aris_core)
set(ARIS_LINK_LIB aris_robot aris_server aris_sensor aris_plan aris_dynamic aris_control aris_core)
set(ALL_INCL_DIR ${ARIS_INCL_DIR} ${RELY_INCL_DIR})
set(ALL_LINK_DIR ${ARIS_LINK_DIR} ${RELY_LINK_DIR})
set(ALL_LINK_LIB ${ARIS_LINK_LIB} ${RELY_LINK_LIB})

include_directories(${ALL_INCL_DIR})
link_directories(${ALL_LINK_DIR})

# set uninstall command
if(UNIX)
	add_custom_target(uninstall rm -r /usr/aris)
endif(UNIX)

# set install directories
if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/aris")
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/aris")
endif(WIN32)

add_definitions(-DARIS_INSTALL_PATH="${CMAKE_INSTALL_PREFIX}")

install(DIRECTORY sh DESTINATION .)
install(FILES src/aris.h DESTINATION include)

# aris core project
file(GLOB HEADERS src/aris_core/*.h src/aris_core/aris_core.h)
file(GLOB SOURCES src/aris_core/*.cpp)
add_library(aris_core STATIC ${HEADERS} ${SOURCES})
set_target_properties(aris_core PROPERTIES PUBLIC_HEADER "${HEADERS}")
target_include_directories(aris_core PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_core/>
	$<INSTALL_INTERFACE:include>)

# aris control project
file(GLOB HEADERS src/aris_control/*.h src/aris_control/aris_control.h)
file(GLOB SOURCES src/aris_control/*.cpp)
add_library(aris_control STATIC ${HEADERS} ${SOURCES})
set_target_properties(aris_control PROPERTIES PUBLIC_HEADER "${HEADERS}")
target_include_directories(aris_control PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_core/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_control/>
	$<INSTALL_INTERFACE:include>)
install(DIRECTORY src/aris_control/resource/ DESTINATION resource/aris_control)

# aris dynamic project
file(GLOB HEADERS src/aris_dynamic/*.h src/aris_dynamic/aris_dynamic.h)
file(GLOB SOURCES src/aris_dynamic/*.cpp)
add_library(aris_dynamic STATIC ${HEADERS} ${SOURCES})
set_target_properties(aris_dynamic PROPERTIES PUBLIC_HEADER "${HEADERS}")
target_include_directories(aris_dynamic PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_core/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_dynamic/>
	$<INSTALL_INTERFACE:include>)

# aris plan project
file(GLOB HEADERS src/aris_plan/*.h src/aris_plan/aris_plan.h)
file(GLOB SOURCES src/aris_plan/*.cpp)
add_library(aris_plan STATIC ${HEADERS} ${SOURCES})
set_target_properties(aris_plan PROPERTIES PUBLIC_HEADER "${HEADERS}")
target_include_directories(aris_plan PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_core/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_control/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_dynamic/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_plan/>
	$<INSTALL_INTERFACE:include>)

# aris sensor project
file(GLOB HEADERS src/aris_sensor/*.h src/aris_sensor/aris_sensor.h)
file(GLOB SOURCES src/aris_sensor/*.cpp)
add_library(aris_sensor STATIC ${HEADERS} ${SOURCES})
set_target_properties(aris_sensor PROPERTIES PUBLIC_HEADER "${HEADERS}")
target_include_directories(aris_sensor PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_core/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_sensor/>
	$<INSTALL_INTERFACE:include>)

# aris server project
file(GLOB HEADERS src/aris_server/*.h src/aris_server/aris_server.h)
file(GLOB SOURCES src/aris_server/*.cpp)
add_library(aris_server STATIC ${HEADERS} ${SOURCES})
set_target_properties(aris_server PROPERTIES PUBLIC_HEADER "${HEADERS}")
target_include_directories(aris_server PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_core/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_control/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_dynamic/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_plan/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_sensor/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_server/>
	$<INSTALL_INTERFACE:include>)
	
# aris robot project
file(GLOB HEADERS src/aris_robot/*.h src/aris_robot/aris_robot.h)
file(GLOB SOURCES src/aris_robot/*.cpp)
add_library(aris_robot STATIC ${HEADERS} ${SOURCES})
set_target_properties(aris_robot PROPERTIES PUBLIC_HEADER "${HEADERS}")
target_include_directories(aris_robot PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_core/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_control/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_dynamic/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_plan/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_sensor/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_server/>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/aris_robot/>
	$<INSTALL_INTERFACE:include>)

install(TARGETS aris_robot aris_core aris_control aris_dynamic aris_plan aris_sensor aris_server
	PUBLIC_HEADER DESTINATION include
	ARCHIVE 
	CONFIGURATIONS Debug DESTINATION lib/debug)

install(TARGETS aris_robot aris_core aris_control aris_dynamic aris_plan aris_sensor aris_server
	EXPORT aris-targets
	PUBLIC_HEADER DESTINATION include
	ARCHIVE 
	CONFIGURATIONS Release DESTINATION lib/release)

install(EXPORT aris-targets
	NAMESPACE aris::
	FILE aris-config.cmake
	DESTINATION share/aris/cmake)

#export(EXPORT aris-targets FILE aris-config.cmake)
#export(PACKAGE aris)
#export(TARGETS aris_robot aris_server aris_sensor aris_plan aris_dynamic aris_control aris_core FILE aris-config.cmake)
#export(TARGETS aris_robot aris_server aris_sensor aris_plan aris_dynamic aris_control aris_core FILE aris-config.cmake)
#export(EXPORT aris-targets)
#export(TARGETS aris_robot aris_server aris_sensor aris_plan aris_dynamic aris_control aris_core FILE aris-config.cmake)
#export(PACKAGE aris)

################################### build tests for aris ####################################
enable_testing()

# test core
file(GLOB SOURCES test/test_core/*)
add_executable(test_core ${SOURCES})
target_link_libraries(test_core aris_core ${RELY_LINK_LIB})
add_test(NAME test_core COMMAND test_core)
set_tests_properties (test_core PROPERTIES FAIL_REGULAR_EXPRESSION "failed")

# test control
file(GLOB SOURCES test/test_control/*)
add_executable(test_control ${SOURCES})
target_link_libraries(test_control aris_control aris_core ${RELY_LINK_LIB})
add_test(NAME test_control COMMAND test_control)
set_tests_properties (test_control PROPERTIES FAIL_REGULAR_EXPRESSION "failed")

# test dynamic
file(GLOB SOURCES test/test_dynamic/*)
add_executable(test_dynamic ${SOURCES})
target_link_libraries(test_dynamic aris_dynamic aris_core ${RELY_LINK_LIB})
add_test(NAME test_dynamic COMMAND test_dynamic)
set_tests_properties (test_dynamic PROPERTIES FAIL_REGULAR_EXPRESSION "failed")
install(DIRECTORY test/test_dynamic/resource/ DESTINATION resource/test_dynamic)

# test plan
file(GLOB SOURCES test/test_plan/*)
add_executable(test_plan ${SOURCES})
target_link_libraries(test_plan aris_plan aris_dynamic aris_control aris_core ${RELY_LINK_LIB})
add_test(NAME test_plan COMMAND test_plan)
set_tests_properties (test_plan PROPERTIES FAIL_REGULAR_EXPRESSION "failed")

# test server
file(GLOB SOURCES test/test_server/*)
add_executable(test_server ${SOURCES})
target_link_libraries(test_server aris_server aris_sensor aris_plan aris_dynamic aris_control aris_core ${RELY_LINK_LIB})
add_test(NAME test_server COMMAND test_server)
set_tests_properties (test_server PROPERTIES FAIL_REGULAR_EXPRESSION "failed")

# test robot
file(GLOB SOURCES test/test_robot/*)
add_executable(test_robot ${SOURCES})
target_link_libraries(test_robot aris_robot aris_server aris_plan aris_dynamic aris_control aris_core ${RELY_LINK_LIB})
add_test(NAME test_robot COMMAND test_robot)
set_tests_properties (test_robot PROPERTIES FAIL_REGULAR_EXPRESSION "failed")

add_executable(rbt test/rbt/client.cpp)
target_link_libraries(rbt ${ALL_LINK_LIB})

################################### build demos for aris ####################################
# Make demo projects
set(MAKE_DEMO " ")
if(MAKE_DEMO)

add_executable(demo_object demo/demo_object/main.cpp)
target_link_libraries(demo_object aris_core ${RELY_LINK_LIB})

add_executable(demo_command_parser_xml demo/demo_command_parser_xml/main.cpp)
target_link_libraries(demo_command_parser_xml aris_core ${RELY_LINK_LIB})
install(DIRECTORY demo/demo_command_parser_xml/resource/ DESTINATION resource/demo_command_parser_xml)

add_executable(demo_command_parser_cpp demo/demo_command_parser_cpp/main.cpp)
target_link_libraries(demo_command_parser_cpp aris_core ${RELY_LINK_LIB})

file(GLOB SOURCES demo/demo_Aris_ExpCal/demo_ExpCal/*)
add_executable(demo_ExpCal ${SOURCES})
target_link_libraries(demo_ExpCal aris_core ${RELY_LINK_LIB})

file(GLOB SOURCES demo/demo_model_scara/*)
add_executable(demo_model_scara ${SOURCES})
target_link_libraries(demo_model_scara aris_dynamic aris_core ${RELY_LINK_LIB})
install(DIRECTORY demo/demo_model_scara/resource/ DESTINATION resource/demo_model_scara)
install(DIRECTORY demo/demo_model_scara/doc/ DESTINATION doc/html/demo_model_scara)

file(GLOB SOURCES demo/demo_stewart/*)
add_executable(demo_stewart ${SOURCES})
target_link_libraries(demo_stewart aris_dynamic aris_core ${RELY_LINK_LIB})
install(DIRECTORY demo/demo_stewart/resource/ DESTINATION resource/demo_stewart)

file(GLOB SOURCES demo/demo_hulk/*)
add_executable(demo_hulk ${SOURCES})
target_link_libraries(demo_hulk ${ALL_LINK_LIB})
install(DIRECTORY demo/demo_hulk/resource/ DESTINATION resource/demo_hulk)

file(GLOB SOURCES demo/demo_ur/*)
add_executable(demo_ur ${SOURCES})
target_link_libraries(demo_ur ${ALL_LINK_LIB})
install(DIRECTORY demo/demo_ur/resource/ DESTINATION resource/demo_ur)

file(GLOB SOURCES demo/demo_server/*)
add_executable(demo_server ${SOURCES})
target_link_libraries(demo_server ${ALL_LINK_LIB})

endif(MAKE_DEMO)
