cmake_minimum_required(VERSION 3.12)

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

project(aris)

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
if(WIN32)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
endif(WIN32)
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
file(GLOB SOURCES src/aris_core/*.h src/aris_core/*.cpp)
add_library(aris_core STATIC ${SOURCES} src/aris_core/aris_core.h)
install(TARGETS aris_core ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_core ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} src/aris_core/aris_core.h DESTINATION include)

# aris control project
file(GLOB SOURCES src/aris_control/*.h src/aris_control/*.cpp)
add_library(aris_control STATIC ${SOURCES} src/aris_control/aris_control.h)
install(TARGETS aris_control ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_control ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} src/aris_control/aris_control.h DESTINATION include)
install(DIRECTORY src/aris_control/resource/ DESTINATION resource/aris_control)

# aris dynamic project
file(GLOB SOURCES src/aris_dynamic/*.h src/aris_dynamic/*.cpp)
add_library(aris_dynamic STATIC ${SOURCES} src/aris_dynamic/aris_dynamic.h)
install(TARGETS aris_dynamic ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_dynamic ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} src/aris_dynamic/aris_dynamic.h DESTINATION include)

# aris plan project
file(GLOB SOURCES src/aris_plan/*.h src/aris_plan/*.cpp)
add_library(aris_plan STATIC ${SOURCES} src/aris_plan/aris_plan.h)
install(TARGETS aris_plan ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_plan ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} src/aris_plan/aris_plan.h DESTINATION include)

# aris sensor project
file(GLOB SOURCES src/aris_sensor/*.h src/aris_sensor/*.cpp)
add_library(aris_sensor STATIC ${SOURCES} src/aris_sensor/aris_sensor.h)
install(TARGETS aris_sensor ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_sensor ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} src/aris_sensor/aris_sensor.h DESTINATION include)

# aris control server project
file(GLOB SOURCES src/aris_server/*.h src/aris_server/*.cpp)
add_library(aris_server STATIC ${SOURCES} src/aris_server/aris_server.h)
install(TARGETS aris_server ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_server ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} DESTINATION include)

# aris robot project
file(GLOB SOURCES src/aris_robot/*.h src/aris_robot/*.cpp)
add_library(aris_robot STATIC ${SOURCES} src/aris_robot/aris_robot.h)
install(TARGETS aris_robot ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_robot ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} src/aris_robot/aris_robot.h DESTINATION include)
install(DIRECTORY src/aris_robot/resource/ DESTINATION resource/aris_robot)

################################### build tests for aris ####################################
enable_testing()

# test core
#set(SOURCE_FILES main.cpp test_core_pipe.h test_core_pipe.cpp test_core_log.h test_core_log.cpp test_core_msg.h test_core_msg.cpp test_core_object.h test_core_object.cpp test_core_socket.h test_core_socket.cpp test_core_websocket.h test_core_websocket.cpp test_core_command.h test_core_command.cpp)
#PREPEND(FULL_SRC test/test_core ${SOURCE_FILES})
#add_executable(test_core ${FULL_SRC})
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

set(SOURCE_FILES client.cpp)
PREPEND(FULL_SRC test/rbt ${SOURCE_FILES})
add_executable(rbt ${FULL_SRC})
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

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_ExpCal/demo_ExpCal ${SOURCE_FILES})
add_executable(demo_ExpCal ${FULL_SRC})
target_link_libraries(demo_ExpCal aris_core ${RELY_LINK_LIB})

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_model_scara ${SOURCE_FILES})
add_executable(demo_model_scara ${FULL_SRC})
target_link_libraries(demo_model_scara aris_dynamic aris_core ${RELY_LINK_LIB})
install(DIRECTORY demo/demo_model_scara/resource/ DESTINATION resource/demo_model_scara)
install(DIRECTORY demo/demo_model_scara/doc/ DESTINATION doc/html/demo_model_scara)

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_stewart ${SOURCE_FILES})
add_executable(demo_stewart ${FULL_SRC})
target_link_libraries(demo_stewart aris_dynamic aris_core ${RELY_LINK_LIB})
install(DIRECTORY demo/demo_stewart/resource/ DESTINATION resource/demo_stewart)

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_hulk ${SOURCE_FILES})
add_executable(demo_hulk ${FULL_SRC})
target_link_libraries(demo_hulk ${ALL_LINK_LIB})
install(DIRECTORY demo/demo_hulk/resource/ DESTINATION resource/demo_hulk)

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_ur ${SOURCE_FILES})
add_executable(demo_ur ${FULL_SRC})
target_link_libraries(demo_ur ${ALL_LINK_LIB})
install(DIRECTORY demo/demo_ur/resource/ DESTINATION resource/demo_ur)

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_server ${SOURCE_FILES})
add_executable(demo_server ${FULL_SRC})
target_link_libraries(demo_server ${ALL_LINK_LIB})

endif(MAKE_DEMO)
