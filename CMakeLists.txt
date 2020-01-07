cmake_minimum_required(VERSION 3.12)

project(aris VERSION 1.5.0)

## set cpp configurations ##
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CONFIGURATION_TYPES Debug Release)
if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/aris/aris-${CMAKE_PROJECT_VERSION}")
	add_custom_target(uninstall rm -r /usr/aris)
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/aris/aris-${CMAKE_PROJECT_VERSION}")
endif(WIN32)

## set rely libs ##
if(UNIX)
	option (USE_XENOMAI "Use Xenomai 3 RT system" OFF)
	option (USE_ETHERLAB "Use Etherlab Ethercat drivers" OFF)

	set(RELY_LINK_LIB atomic pthread stdc++fs)#pthread is needed for concurrency
	set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config)

	if(USE_XENOMAI)
		message("using xenomai")
		add_definitions(-DARIS_USE_XENOMAI)
		set(XENOMAI_ROOT_DIR "/usr/xenomai")
		find_package(Xenomai REQUIRED QUIET)
		set(RELY_INCL_DIR ${XENOMAI_INCLUDE_DIRS} ${RELY_INCL_DIR})
		set(RELY_LINK_DIR "/usr/xenomai/lib" ${RELY_LINK_DIR})
		set(RELY_LINK_LIB ${XENOMAI_LIBRARIES} ${RELY_LINK_LIB})
	endif(USE_XENOMAI)
	
	if(USE_ETHERLAB)
		message("using etherlab")
		add_definitions(-DARIS_USE_ETHERLAB)
		set(RELY_INCL_DIR "/opt/etherlab/include" ${RELY_INCL_DIR})
		set(RELY_LINK_DIR "/opt/etherlab/lib" ${RELY_LINK_DIR})
		if(USE_XENOMAI)
			set(RELY_LINK_LIB ethercat_rtdm ${RELY_LINK_LIB})
		else(USE_XENOMAI)
			set(RELY_LINK_LIB ethercat ${RELY_LINK_LIB})
		endif(USE_XENOMAI)
		
	endif(USE_ETHERLAB)
endif(UNIX)
if(WIN32)
	set(RELY_LINK_LIB "ws2_32")#ws2_32 is needed for socket
endif(WIN32)

## set definitions to disable some warning ##
if(WIN32)
	add_definitions(-DWIN32 -D_WINSOCK_DEPRECATED_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
endif(WIN32)
if(UNIX)
	add_definitions(-DUNIX)
endif(UNIX)
add_definitions(-DARIS_INSTALL_PATH="${CMAKE_INSTALL_PREFIX}")

## build targets ##
foreach(target robot server plan dynamic control sensor core)
	file(GLOB HEADERS include/aris/${target}/*.hpp include/aris/${target}/*.h)
	file(GLOB SOURCES src/${target}/*.cpp src/${target}/*.c src/${target}/*.hpp src/${target}/*.h ${HEADERS})
	add_library(aris_${target} STATIC ${HEADERS} ${SOURCES})
	target_include_directories(aris_${target}
		PUBLIC 
			$<INSTALL_INTERFACE:include>
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/src
			${RELY_INCL_DIR}
	)
	
	set_target_properties(aris_${target} PROPERTIES PUBLIC_HEADER "${HEADERS}")
	set(aris_LIBRARIES ${aris_LIBRARIES} optimized aris::aris_${target} debug aris::debug::aris_${target})
	set(ARIS_LINK_LIB ${ARIS_LINK_LIB} aris_${target})
	
	install(TARGETS aris_${target} EXPORT aris-targets-debug PUBLIC_HEADER DESTINATION "include/aris/${target}" ARCHIVE CONFIGURATIONS Debug DESTINATION "lib/debug")
	install(TARGETS aris_${target} EXPORT aris-targets-release PUBLIC_HEADER DESTINATION "include/aris/${target}" ARCHIVE CONFIGURATIONS Release DESTINATION "lib/release")

	if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${target}/resource)
		install(DIRECTORY src/${target}/resource/ DESTINATION resource/${target})
	endif()
endforeach(target)
if(UNIX)
	set(ARIS_LINK_LIB -Wl,--start-group ${ARIS_LINK_LIB} -Wl,--end-group)
endif(UNIX)

## install exports ##
install(EXPORT aris-targets-debug NAMESPACE aris::debug:: FILE aris-targets-debug.cmake DESTINATION "share/aris/cmake")
install(EXPORT aris-targets-release NAMESPACE aris:: FILE aris-targets-release.cmake DESTINATION "share/aris/cmake")
install(FILES include/aris.hpp DESTINATION include)

## install config ##
include(CMakePackageConfigHelpers)
set(aris_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(aris_LIBRARIES ${aris_LIBRARIES} ${RELY_LINK_LIB})
configure_package_config_file("config/aris-config.cmake.in" "${CMAKE_BINARY_DIR}/aris-config.cmake" INSTALL_DESTINATION "share/aris/cmake")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/aris-config.cmake" DESTINATION "share/aris/cmake")

## install tools ##
install(DIRECTORY sh DESTINATION .)

link_directories(${RELY_LINK_DIR})
################################### build tests for aris ####################################
option(BUILD_TESTS "build aris tests" OFF)
if(BUILD_TESTS)
	enable_testing()
	## build tests ##
	set(ARIS_TESTS test_core test_control test_dynamic test_plan test_server test_robot)
	foreach(aris_test ${ARIS_TESTS})
		file(GLOB SOURCES test/${aris_test}/*)
		add_executable(${aris_test} ${SOURCES})
		#target_include_directories(${aris_test} PUBLIC include ${RELY_INCL_DIR})
		#target_link_directories(${aris_test} ${RELY_LINK_DIR})
		target_link_libraries(${aris_test} ${ARIS_LINK_LIB} ${RELY_LINK_LIB})
		add_test(NAME ${aris_test} COMMAND ${aris_test})
		set_tests_properties(${aris_test} PROPERTIES FAIL_REGULAR_EXPRESSION "failed")
		if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/${aris_test}/resource)
			install(DIRECTORY test/${aris_test}/resource/ DESTINATION resource/${aris_test})
		endif()
	endforeach(aris_test)
endif(BUILD_TESTS)
################################### build demos for aris ####################################
option(BUILD_DEMOS "build aris demos" OFF)
if(BUILD_DEMOS)
	set(ARIS_DEMOS 	rbt demo_object demo_command_parser_xml demo_command_parser_cpp demo_expression_calculator
					demo_model_scara demo_stewart demo_hulk demo_ur demo_server demo_rokae_server)
	foreach(aris_demo ${ARIS_DEMOS})
		file(GLOB SOURCES demo/${aris_demo}/*.h demo/${aris_demo}/*.cpp)
		add_executable(${aris_demo} ${SOURCES})
		target_link_libraries(${aris_demo} ${ARIS_LINK_LIB} ${RELY_LINK_LIB})
		add_test(NAME ${aris_demo} COMMAND ${aris_demo})
		set_tests_properties(${aris_demo} PROPERTIES FAIL_REGULAR_EXPRESSION "failed")
		if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/demo/${aris_demo}/resource)
			install(DIRECTORY demo/${aris_demo}/resource/ DESTINATION resource/${aris_demo})
		endif()
		if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/demo/${aris_demo}/doc)
			install(DIRECTORY demo/${aris_demo}/doc/ DESTINATION doc/html/${aris_demo})
		endif()
	endforeach(aris_demo)
endif(BUILD_DEMOS)
################################### build docs for aris ####################################
option(BUILD_DOCS "build aris docs" OFF)
if(BUILD_DOCS)
	find_package(Doxygen
		REQUIRED dot
		OPTIONAL_COMPONENTS mscgen dia)
	if (DOXYGEN_FOUND)
		message("doxygen found")
		# set input and output files
		set(DOXYGEN_IN "${CMAKE_CURRENT_LIST_DIR}/doc/Doxyfile.in")
		set(DOXYGEN_OUT "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")
		set(DOXYGEN_LATEX_MAKEINDEX_CMD "makeindex")
		set(DOXYGEN_EXTRACT_ALL "YES")
		# request to configure the file
		# configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

		# note the option ALL which allows to build the docs together with the application
		# add_custom_target( docs ALL
		#	COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
		#	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		#	COMMENT "Generating API documentation with Doxygen"
		#	VERBATIM )
		#doxygen_add_docs(
		#	doxygen
		#    ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/demo
		#	COMMENT "Generate man pages")
	else (DOXYGEN_FOUND)
		message("Doxygen need to be installed to generate the doxygen documentation")
	endif (DOXYGEN_FOUND)
endif(BUILD_DOCS)
