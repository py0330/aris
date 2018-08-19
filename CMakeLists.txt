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

project(aris)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)


set(CMAKE_CXX_STANDARD 17)

# set compile options
if(UNIX)
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif(UNIX)

# set definitions to disable some warning
if(WIN32)
	add_definitions(-DWIN32 -D_WINSOCK_DEPRECATED_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
endif(WIN32)
if(UNIX)
	add_definitions(-DUNIX)
endif(UNIX)

# set debug and release options
if(WIN32)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
endif(WIN32)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CONFIGURATION_TYPES Debug Release)

# set include and lib folders
if(UNIX)
	set(RELY_LINK_LIB pthread)#pthread is needed for concurrency

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

set(ARIS_INCL_DIR src/ src/aris_server src/aris_sensor src/aris_robot src/aris_dynamic src/aris_control src/aris_plan src/aris_core)
set(ARIS_LINK_LIB aris_server aris_sensor aris_robot aris_plan aris_dynamic aris_control aris_core)
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
set(SOURCE aris_core_command aris_core_socket aris_core_pipe aris_core_expression_calculator aris_core_log aris_core_msg aris_core_object tinyxml2 aris_core_basic_type)
PREPEND_CPP(FULL_SRC src/aris_core ${SOURCE})
PREPEND_H(FULL_H src/aris_core ${SOURCE})
add_library(aris_core STATIC ${FULL_SRC} ${FULL_H} src/aris_core/aris_core.h src/aris.h)
install(TARGETS aris_core ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_core ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} src/aris_core/aris_core.h DESTINATION include)

# aris plan project
set(SOURCE aris_plan_function aris_plan_root)
PREPEND_CPP(FULL_SRC src/aris_plan ${SOURCE})
PREPEND_H(FULL_H src/aris_plan ${SOURCE})
add_library(aris_plan STATIC ${FULL_SRC} ${FULL_H} src/aris_plan/aris_plan.h)
install(TARGETS aris_plan ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_plan ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} src/aris_plan/aris_plan.h DESTINATION include)

# aris control project
set(SOURCE aris_control_ethercat aris_control_ethercat_kernel aris_control_master_slave aris_control_controller_motion aris_control_rt_timer)
PREPEND_CPP(FULL_SRC src/aris_control ${SOURCE})
PREPEND_H(FULL_H src/aris_control ${SOURCE})
add_library(aris_control STATIC ${FULL_SRC} ${FULL_H} src/aris_control/aris_control.h)
install(TARGETS aris_control ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_control ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} src/aris_control/aris_control.h DESTINATION include)
install(DIRECTORY src/aris_control/resource/ DESTINATION resource/aris_control)

# aris dynamic project
set(SOURCE aris_dynamic_plan aris_dynamic_matrix aris_dynamic_block_matrix aris_dynamic_cell aris_dynamic_spline aris_dynamic_screw
 aris_dynamic_model_basic aris_dynamic_model_coordinate aris_dynamic_model_interaction aris_dynamic_model_solver
 aris_dynamic_model_simulation aris_dynamic_model aris_dynamic_simple_model)
PREPEND_CPP(FULL_SRC src/aris_dynamic ${SOURCE})
PREPEND_H(FULL_H src/aris_dynamic ${SOURCE})
add_library(aris_dynamic STATIC ${FULL_SRC} ${FULL_H} src/aris_dynamic/aris_dynamic.h)
install(TARGETS aris_dynamic ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_dynamic ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} src/aris_dynamic/aris_dynamic.h DESTINATION include)

# aris robot project
set(SOURCE aris_robot_ur)
PREPEND_CPP(FULL_SRC src/aris_robot ${SOURCE})
PREPEND_H(FULL_H src/aris_robot ${SOURCE})
add_library(aris_robot STATIC ${FULL_SRC} ${FULL_H} src/aris_robot/aris_robot.h)
install(TARGETS aris_robot ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_robot ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} src/aris_robot/aris_robot.h DESTINATION include)
install(DIRECTORY src/aris_robot/resource/ DESTINATION resource/aris_robot)

# aris sensor project
set(SOURCE aris_sensor_base)
PREPEND_CPP(FULL_SRC src/aris_sensor ${SOURCE})
PREPEND_H(FULL_H src/aris_sensor ${SOURCE})
add_library(aris_sensor STATIC ${FULL_SRC} ${FULL_H} src/aris_sensor/aris_sensor.h)
install(TARGETS aris_sensor ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_sensor ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} src/aris_sensor/aris_sensor.h DESTINATION include)

# aris control server project
set(SOURCE aris_server aris_server_widget)
PREPEND_CPP(FULL_SRC src/aris_server ${SOURCE})
PREPEND_H(FULL_H src/aris_server ${SOURCE})
add_library(aris_server STATIC ${FULL_SRC} ${FULL_H})
install(TARGETS aris_server ARCHIVE CONFIGURATIONS Debug DESTINATION lib/debug)
install(TARGETS aris_server ARCHIVE CONFIGURATIONS Release DESTINATION lib/release)
install(FILES ${FULL_H} DESTINATION include)


################################### build tests for aris ####################################
enable_testing()

set(SOURCE_FILES main.cpp test_core_pipe.h test_core_pipe.cpp test_core_log.h test_core_log.cpp test_core_msg.h test_core_msg.cpp test_core_object.h test_core_object.cpp test_core_socket.h test_core_socket.cpp test_core_command.h test_core_command.cpp)
PREPEND(FULL_SRC test/test_core ${SOURCE_FILES})
add_executable(test_core ${FULL_SRC})
target_link_libraries(test_core aris_core ${RELY_LINK_LIB})
add_test(NAME test_core COMMAND test_core)
set_tests_properties (test_core PROPERTIES FAIL_REGULAR_EXPRESSION "failed")

set(SOURCE_FILES main.cpp test_plan_function.h test_plan_function.cpp)
PREPEND(FULL_SRC test/test_plan ${SOURCE_FILES})
add_executable(test_plan ${FULL_SRC})
target_link_libraries(test_plan aris_plan aris_dynamic aris_control aris_core ${RELY_LINK_LIB})
add_test(NAME test_plan COMMAND test_plan)
set_tests_properties (test_plan PROPERTIES FAIL_REGULAR_EXPRESSION "failed")

set(SOURCE_FILES main.cpp test_control_ethercat.h test_control_ethercat.cpp test_control_motion.h test_control_motion.cpp test_control_master_slave.h test_control_master_slave.cpp)
PREPEND(FULL_SRC test/test_control ${SOURCE_FILES})
add_executable(test_control ${FULL_SRC})
target_link_libraries(test_control aris_control aris_core ${RELY_LINK_LIB})
add_test(NAME test_control COMMAND test_control)
set_tests_properties (test_control PROPERTIES FAIL_REGULAR_EXPRESSION "failed")

set(SOURCE_FILES main.cpp test_dynamic_matrix.h test_dynamic_matrix.cpp test_dynamic_block_matrix.h test_dynamic_block_matrix.cpp test_dynamic_cell.h test_dynamic_cell.cpp test_dynamic_spline.h test_dynamic_spline.cpp test_dynamic_screw.h test_dynamic_screw.cpp test_dynamic_model.h test_dynamic_model.cpp test_dynamic_model_coordinate.h test_dynamic_model_coordinate.cpp test_dynamic_model_compute.h test_dynamic_model_compute.cpp test_dynamic_model_interaction.h test_dynamic_model_interaction.cpp test_dynamic_model_multi.h test_dynamic_model_multi.cpp test_dynamic_simple_model.h test_dynamic_simple_model.cpp test_dynamic_plan.h test_dynamic_plan.cpp)
PREPEND(FULL_SRC test/test_dynamic ${SOURCE_FILES})
add_executable(test_dynamic ${FULL_SRC})
target_link_libraries(test_dynamic aris_dynamic aris_core ${RELY_LINK_LIB})
add_test(NAME test_dynamic COMMAND test_dynamic)
set_tests_properties (test_dynamic PROPERTIES FAIL_REGULAR_EXPRESSION "failed")
install(DIRECTORY test/test_dynamic/resource/ DESTINATION resource/test_dynamic)

set(SOURCE_FILES main.cpp test_control_server.h test_control_server.cpp)
PREPEND(FULL_SRC test/test_control_server ${SOURCE_FILES})
add_executable(test_control_server ${FULL_SRC})
target_link_libraries(test_control_server aris_server aris_sensor aris_robot aris_plan aris_dynamic aris_control aris_core ${RELY_LINK_LIB})
add_test(NAME test_control_server COMMAND test_control_server)
set_tests_properties (test_control_server PROPERTIES FAIL_REGULAR_EXPRESSION "failed")

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
target_link_libraries(demo_ur aris_robot aris_control aris_dynamic aris_core ${RELY_LINK_LIB})
install(DIRECTORY demo/demo_ur/resource/ DESTINATION resource/demo_ur)

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_server_ur ${SOURCE_FILES})
add_executable(demo_server_ur ${FULL_SRC})
target_link_libraries(demo_server_ur ${ALL_LINK_LIB})


endif(MAKE_DEMO)
