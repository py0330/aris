cmake_minimum_required(VERSION 3.12)

project(aris VERSION 2.3.2.220803 DESCRIPTION "add which root for scara")

# version
configure_file(${PROJECT_SOURCE_DIR}/config/aris_version.h.in aris_version.h)

## set cpp configurations ##
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH bin)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/aris/aris-${CMAKE_PROJECT_VERSION}")
	add_custom_target(uninstall rm -r /usr/aris)
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/aris/aris-${CMAKE_PROJECT_VERSION}")
endif(WIN32)

## set definitions to disable some warning ##
if(WIN32)
	add_definitions(-DWIN32 -D_WINSOCK_DEPRECATED_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4251")
endif(WIN32)
if(UNIX)
	add_definitions(-DUNIX)
	add_definitions(-fPIC)
endif(UNIX)
add_definitions(-DARIS_INSTALL_PATH="${CMAKE_INSTALL_PREFIX}")

## set rely libs ##
# std & socker rely #
if(UNIX)
	set(RELY_LINK_LIB atomic pthread stdc++fs dl)#pthread is needed for concurrency
	set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config)
elseif(WIN32)
	set(RELY_LINK_LIB "ws2_32")#ws2_32 is needed for socket
endif()

# rt timer #
set(RT_TIMER "NON" CACHE STRING "")
if(UNIX)
	set_property(CACHE RT_TIMER PROPERTY STRINGS "NON" "RT_PREEMT" "XENOMAI3")
	
	if("${RT_TIMER}" STREQUAL "NON")
		add_definitions(-DARIS_USE_RT_NON)
	elseif("${RT_TIMER}" STREQUAL "RT_PREEMT")
		add_definitions(-DARIS_USE_RT_PREEMT)
	elseif("${RT_TIMER}" STREQUAL "XENOMAI3")
		add_definitions(-DARIS_USE_XENOMAI3)
		set(XENOMAI_ROOT_DIR "/usr/xenomai")
		find_package(Xenomai REQUIRED QUIET)
		set(RELY_INCL_DIR ${XENOMAI_INCLUDE_DIRS} ${RELY_INCL_DIR})
		set(RELY_LINK_DIR "/usr/xenomai/lib" ${RELY_LINK_DIR})
		set(RELY_LINK_LIB ${XENOMAI_LIBRARIES} ${RELY_LINK_LIB})
	endif()
elseif(WIN32)
	set_property(CACHE RT_TIMER PROPERTY STRINGS "NON")
	if ("${RT_TIMER}" STREQUAL "NON")
		add_definitions(-DARIS_USE_RT_NON)
	endif()
endif()
message("RT_TIMER : ${RT_TIMER}")

# ethercat #
set(ETHERCAT "SIMULATION" CACHE STRING "")
if(UNIX)
	set_property(CACHE ETHERCAT PROPERTY STRINGS "SIMULATION" "ETHERLAB")
	
	if("${ETHERCAT}" STREQUAL "SIMULATION")
		add_definitions(-DARIS_USE_ETHERCAT_SIMULATION)
	elseif("${ETHERCAT}" STREQUAL "ETHERLAB")
		add_definitions(-DARIS_USE_ETHERLAB)
		set(RELY_INCL_DIR "/opt/etherlab/include" ${RELY_INCL_DIR})
		set(RELY_LINK_DIR "/opt/etherlab/lib" ${RELY_LINK_DIR})
		if("${RT_TIMER}" STREQUAL "XENOMAI3")
			set(RELY_LINK_LIB ethercat_rtdm ${RELY_LINK_LIB})
		else()
			set(RELY_LINK_LIB ethercat ${RELY_LINK_LIB})
		endif()
	endif()
elseif(WIN32)
	set_property(CACHE ETHERCAT PROPERTY STRINGS "SIMULATION")
	if ("${ETHERCAT}" STREQUAL "SIMULATION")
		add_definitions(-DARIS_USE_ETHERCAT_SIMULATION)
	endif()
endif()
message("ETHERCAT : ${ETHERCAT}")

link_directories(${RELY_LINK_DIR})

## build targets ##
include (GenerateExportHeader) 
foreach(target robot server plan dynamic control sensor core ext)
	file(GLOB HEADERS include/aris/${target}/*.hpp include/aris/${target}/*.h)
	file(GLOB SOURCES src/${target}/*.cpp src/${target}/*.c src/${target}/*.hpp src/${target}/*.h ${HEADERS})
	add_library(aris_${target} OBJECT ${HEADERS} ${SOURCES})
	target_compile_definitions(aris_${target} PRIVATE aris_lib_EXPORTS)
	target_include_directories(aris_${target}
		PUBLIC 
			$<INSTALL_INTERFACE:include>
			$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
		PRIVATE
			${PROJECT_SOURCE_DIR}/src
			${PROJECT_BINARY_DIR}
			${RELY_INCL_DIR}
	)
	
	set_target_properties(aris_${target} PROPERTIES PUBLIC_HEADER "${HEADERS}")
	set(aris_LIBRARIES ${aris_LIBRARIES} optimized aris::aris_${target} debug aris::debug::aris_${target})
	set(ARIS_LINK_LIB ${ARIS_LINK_LIB} aris_${target})
	set(ARIS_OBJECTS ${ARIS_OBJECTS} $<TARGET_OBJECTS:aris_${target}>)
	set(ARIS_HEADERS ${ARIS_HEADERS} include/aris/${target})
	
	install(FILES ${HEADERS} DESTINATION include/aris/${target})
	#install(TARGETS aris_${target} EXPORT aris-targets-debug PUBLIC_HEADER DESTINATION "include/aris/${target}")
	#install(TARGETS aris_${target} EXPORT aris-targets-release PUBLIC_HEADER DESTINATION "include/aris/${target}")

	if(EXISTS ${PROJECT_SOURCE_DIR}/src/${target}/resource)
		install(DIRECTORY src/${target}/resource/ DESTINATION resource/${target})
	endif()
endforeach(target)
add_library(aris_lib SHARED ${ARIS_OBJECTS})
generate_export_header(aris_lib EXPORT_MACRO_NAME ARIS_API)
target_link_libraries(aris_lib PUBLIC ${RELY_LINK_LIB})
target_include_directories(aris_lib
	PUBLIC 
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
	PRIVATE
		${PROJECT_SOURCE_DIR}/src
		${RELY_INCL_DIR}
)

set(aris_LIBRARIES optimized aris::aris_lib debug aris::debug::aris_lib)

install(TARGETS aris_lib EXPORT aris-targets-debug CONFIGURATIONS Debug DESTINATION "lib/debug")
install(TARGETS aris_lib EXPORT aris-targets-release CONFIGURATIONS Release DESTINATION "lib/release")
set(ARIS_LINK_LIB aris_lib)

## install exports ##
install(EXPORT aris-targets-debug NAMESPACE aris::debug:: FILE aris-targets-debug.cmake DESTINATION "share/aris/cmake")
install(EXPORT aris-targets-release NAMESPACE aris:: FILE aris-targets-release.cmake DESTINATION "share/aris/cmake")
install(FILES include/aris.hpp ${PROJECT_BINARY_DIR}/aris_lib_export.h ${PROJECT_BINARY_DIR}/aris_version.h DESTINATION include)

## install config ##
include(CMakePackageConfigHelpers)
set(aris_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)
set(aris_LIBRARIES ${aris_LIBRARIES} ${RELY_LINK_LIB})
configure_package_config_file(config/aris-config.cmake.in ${PROJECT_BINARY_DIR}/aris-config.cmake INSTALL_DESTINATION "share/aris/cmake")
install(FILES ${PROJECT_BINARY_DIR}/aris-config.cmake DESTINATION "share/aris/cmake")

## install tools ##
install(DIRECTORY sh DESTINATION .)


################################### build tests for aris ####################################
option(BUILD_TESTS "build aris tests" OFF)
if(BUILD_TESTS)
	enable_testing()
	## build tests ##
	set(ARIS_TESTS test_core test_control test_dynamic test_plan test_server test_robot)
	foreach(aris_test ${ARIS_TESTS})
		file(GLOB SOURCES test/${aris_test}/*)
		add_executable(${aris_test} ${SOURCES})
		target_link_libraries(${aris_test} ${ARIS_LINK_LIB} ${RELY_LINK_LIB})
		add_test(NAME ${aris_test} COMMAND ${aris_test})
		set_tests_properties(${aris_test} PROPERTIES FAIL_REGULAR_EXPRESSION "failed")
		if(EXISTS ${PROJECT_SOURCE_DIR}/test/${aris_test}/resource)
			install(DIRECTORY test/${aris_test}/resource/ DESTINATION resource/${aris_test})
		endif()
	endforeach(aris_test)
endif(BUILD_TESTS)
################################### build demos for aris ####################################
option(BUILD_DEMOS "build aris demos" OFF)
if(BUILD_DEMOS)
	set(ARIS_DEMOS 	rbt demo_object demo_command_parser_xml demo_command_parser_cpp demo_expression_calculator
					demo_model_delta demo_model_scara demo_model_seven_axis demo_model_multi demo_stewart demo_hulk demo_ur 
					demo_server demo_server_scara demo_server_puma demo_rokae_server demo_reflection)
	foreach(aris_demo ${ARIS_DEMOS})
		file(GLOB SOURCES demo/${aris_demo}/*.h demo/${aris_demo}/*.cpp)
		add_executable(${aris_demo} ${SOURCES})
		target_link_libraries(${aris_demo} aris_lib ${RELY_LINK_LIB})
		if(EXISTS ${PROJECT_SOURCE_DIR}/demo/${aris_demo}/resource)
			install(DIRECTORY demo/${aris_demo}/resource/ DESTINATION resource/${aris_demo})
		endif()
		if(EXISTS ${PROJECT_SOURCE_DIR}/demo/${aris_demo}/doc)
			install(DIRECTORY demo/${aris_demo}/doc/ DESTINATION doc/html/${aris_demo})
		endif()
	endforeach(aris_demo)
endif(BUILD_DEMOS)
################################### build tools for aris ####################################
option(BUILD_TOOLS "build aris tools" OFF)
if(BUILD_TOOLS)
	foreach(aris_tool doc_server)
		file(GLOB SOURCES tool/${aris_tool}/*.h tool/${aris_tool}/*.c tool/${aris_tool}/*.cpp)
		add_executable(tool_${aris_tool} ${SOURCES})
		target_link_libraries(tool_${aris_tool} ${ARIS_LINK_LIB} ${RELY_LINK_LIB})
		install(TARGETS tool_${aris_tool} DESTINATION "tool")
	endforeach(aris_tool)
endif(BUILD_TOOLS)
################################### build docs for aris ####################################
option(BUILD_DOCS "build aris docs" OFF)
if(BUILD_DOCS)
	find_package(Doxygen
		REQUIRED dot
		OPTIONAL_COMPONENTS mscgen dia)
	if (DOXYGEN_FOUND)
		message("doxygen found")
		# set input and output files
		# set(DOXYGEN_IN "${CMAKE_CURRENT_LIST_DIR}/doc/Doxyfile.in")
		# set(DOXYGEN_OUT "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")

		# request to configure the file
		# configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

		# note the option ALL which allows to build the docs together with the application
		# add_custom_target( docs ALL
		#	COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
		#	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		#	COMMENT "Generating API documentation with Doxygen"
		#	VERBATIM )

		set(DOXYGEN_EXTRACT_ALL "YES")
		set(DOXYGEN_MARKDOWN_SUPPORT "YES")
		set(DOXYGEN_EXTRACT_PACKAGE "YES")
		set(DOXYGEN_EXTRACT_STATIC "YES")
		set(DOXYGEN_EXTRACT_LOCAL_CLASSES "NO")
		set(DOXYGEN_SHOW_GROUPED_MEMB_INC "YES")
		set(DOXYGEN_EXAMPLE_PATH ${PROJECT_SOURCE_DIR}/demo)
		set(DOXYGEN_INPUT_ENCODING "UTF-8")
		set(DOXYGEN_DOXYFILE_ENCODING "UTF-8")

		doxygen_add_docs(
			doxygen 
		    ${PROJECT_SOURCE_DIR}/demo ${PROJECT_SOURCE_DIR}/include
			ALL
			COMMENT "Generate man pages")

		install(DIRECTORY ${PROJECT_BINARY_DIR}/html/ DESTINATION doc/html/)
	else (DOXYGEN_FOUND)
		message("Doxygen need to be installed to generate the doxygen documentation")
	endif (DOXYGEN_FOUND)
endif(BUILD_DOCS)

