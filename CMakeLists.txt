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

# set unix compiler
if(UNIX)
	set(CMAKE_C_COMPILER "/usr/bin/gcc")
	set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#	set(CMAKE_C_COMPILER "/usr/bin/clang")
#	set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
endif(UNIX)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
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
set(SYSTEM_LINK_LIB pthread)
set(XENOMAI_INCL_DIR "/usr/xenomai/include")
set(XENOMAI_LINK_DIR "/usr/xenomai/lib")
set(XENOMAI_LINK_LIB native rtdm xenomai)
set(ETHERLAB_INCL_DIR "/opt/etherlab/include")
set(ETHERLAB_LINK_DIR "/opt/etherlab/lib")
set(ETHERLAB_LINK_LIB ethercat rt)
set(ARIS_INCL_DIR src/ src/aris_server src/aris_core src/aris_dynamic src/aris_sensor src/aris_control)
set(ARIS_LINK_LIB aris_server aris_sensor aris_dynamic aris_control aris_core)

set(ALL_INCL_DIR ${ARIS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR})
set(ALL_LINK_DIR ${ARIS_LINK_DIR} ${ETHERLAB_LINK_DIR} ${XENOMAI_LINK_DIR})
set(ALL_LINK_LIB ${ARIS_LINK_LIB} ${ETHERLAB_LINK_LIB} ${XENOMAI_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(UNIX)
if(WIN32)
set(SYSTEM_LINK_LIB ws2_32 setupapi)#ws2_32 is needed for socket setuaipi is needed for xsens
set(ARIS_INCL_DIR src/ src/aris_server src/aris_core src/aris_dynamic src/aris_sensor src/aris_control)
set(ARIS_LINK_LIB aris_server aris_sensor aris_dynamic aris_control aris_core)

set(ALL_INCL_DIR ${ARIS_INCL_DIR})
set(ALL_LINK_DIR ${ARIS_LINK_DIR})
set(ALL_LINK_LIB ${ARIS_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(WIN32)

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

install(DIRECTORY sh DESTINATION .)
install(FILES src/aris.h DESTINATION include)

# aris core project
set(SOURCE aris_core_command aris_core_msg aris_core_socket aris_core_pipe aris_core_expression_calculator aris_core_msg_loop aris_core_xml tinyxml2)
PREPEND_CPP(FULL_SRC src/aris_core ${SOURCE})
PREPEND_H(FULL_H src/aris_core ${SOURCE})
add_library(aris_core STATIC ${FULL_SRC} ${FULL_H} src/aris_core/aris_core.h)
install(TARGETS aris_core ARCHIVE DESTINATION lib)
install(FILES ${FULL_H} src/aris_core/aris_core.h DESTINATION include)

# aris control project
set(SOURCE aris_control_motion aris_control_ethercat)
PREPEND_CPP(FULL_SRC src/aris_control ${SOURCE})
PREPEND_H(FULL_H src/aris_control ${SOURCE})
add_library(aris_control STATIC ${FULL_SRC} ${FULL_H} src/aris_control/aris_control.h)
install(TARGETS aris_control ARCHIVE DESTINATION lib)
install(FILES ${FULL_H} src/aris_control/aris_control.h DESTINATION include)
install(DIRECTORY src/aris_control/resource/ DESTINATION resource/aris_control)

# aris dynamic project
set(SOURCE aris_dynamic_plan aris_dynamic_kernel aris_dynamic_model)
PREPEND_CPP(FULL_SRC src/aris_dynamic ${SOURCE})
PREPEND_H(FULL_H src/aris_dynamic ${SOURCE})
add_library(aris_dynamic STATIC ${FULL_SRC} ${FULL_H} src/aris_dynamic/aris_dynamic.h)
install(TARGETS aris_dynamic ARCHIVE DESTINATION lib)
install(FILES ${FULL_H} src/aris_dynamic/aris_dynamic.h DESTINATION include)

# aris sensor project
set(SOURCE aris_sensor_base)
PREPEND_CPP(FULL_SRC src/aris_sensor ${SOURCE})
PREPEND_H(FULL_H src/aris_sensor ${SOURCE})
add_library(aris_sensor STATIC ${FULL_SRC} ${FULL_H} src/aris_sensor/aris_sensor.h)
install(TARGETS aris_sensor ARCHIVE DESTINATION lib)
install(FILES ${FULL_H} src/aris_sensor/aris_sensor.h DESTINATION include)

# aris control server project
set(SOURCE aris_server aris_server_widget)
PREPEND_CPP(FULL_SRC src/aris_server ${SOURCE})
PREPEND_H(FULL_H src/aris_server ${SOURCE})
add_library(aris_server STATIC ${FULL_SRC} ${FULL_H})
install(TARGETS aris_server ARCHIVE DESTINATION lib)
install(FILES ${FULL_H} DESTINATION include)


################################### build tests for aris ####################################
enable_testing()

set(SOURCE_FILES main.cpp test_dynamic_kernel.h test_dynamic_kernel.cpp test_dynamic_model.h test_dynamic_model.cpp)
PREPEND(FULL_SRC test/test_dynamic ${SOURCE_FILES})
add_executable(test_dynamic ${FULL_SRC})
target_link_libraries(test_dynamic ${ALL_LINK_LIB})
add_test(NAME test_dynamic COMMAND test_dynamic)
set_tests_properties (test_dynamic PROPERTIES FAIL_REGULAR_EXPRESSION "failed")
install(DIRECTORY test/test_dynamic/resource/ DESTINATION resource/test_dynamic)

set(SOURCE_FILES main.cpp test_core_pipe.h test_core_pipe.cpp test_core_msg.h test_core_msg.cpp test_core_xml.h test_core_xml.cpp test_core_socket.h test_core_socket.cpp test_core_command.h test_core_command.cpp)
PREPEND(FULL_SRC test/test_core ${SOURCE_FILES})
add_executable(test_core ${FULL_SRC})
target_link_libraries(test_core ${ALL_LINK_LIB})
add_test(NAME test_core COMMAND test_core)
set_tests_properties (test_core PROPERTIES FAIL_REGULAR_EXPRESSION "failed")

set(SOURCE_FILES main.cpp test_control_ethercat.h test_control_ethercat.cpp test_control_motion.h test_control_motion.cpp)
PREPEND(FULL_SRC test/test_control ${SOURCE_FILES})
add_executable(test_control ${FULL_SRC})
target_link_libraries(test_control ${ALL_LINK_LIB})
add_test(NAME test_control COMMAND test_control)
set_tests_properties (test_control PROPERTIES FAIL_REGULAR_EXPRESSION "failed")

set(SOURCE_FILES main.cpp test_control_server.h test_control_server.cpp)
PREPEND(FULL_SRC test/test_control_server ${SOURCE_FILES})
add_executable(test_control_server ${FULL_SRC})
target_link_libraries(test_control_server ${ALL_LINK_LIB})
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

set(SOURCE_FILES main.cpp)
PREPEND(FULL_SRC demo/demo_Aris_ExpCal/demo_ExpCal ${SOURCE_FILES})
add_executable(demo_ExpCal ${FULL_SRC})
target_link_libraries(demo_ExpCal ${ALL_LINK_LIB})

endif(MAKE_DEMO)
