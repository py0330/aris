cmake_minimum_required(VERSION 2.8)

project(my_project)

# set unix compiler
if(UNIX)
#	set(CMAKE_C_COMPILER "/usr/bin/gcc")
#	set(CMAKE_CXX_COMPILER "/usr/bin/g++")
	set(CMAKE_C_COMPILER "/usr/bin/clang-3.8")
	set(CMAKE_CXX_COMPILER "/usr/bin/clang++-3.8")
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
	set(ALL_LINK_LIB pthread)#pthread is needed for concurrency

	option (USE_XENOMAI "Use Xenomai RT system" OFF)
	if(USE_XENOMAI)
		message("using xenomai")
		add_definitions(-DUSE_XENOMAI)
		set(ALL_INCL_DIR "/usr/xenomai/include" ${ALL_INCL_DIR})
		set(ALL_LINK_DIR "/usr/xenomai/lib" ${ALL_LINK_DIR})
		set(ALL_LINK_LIB native rtdm xenomai ${ALL_LINK_LIB})
	endif(USE_XENOMAI)

	option (USE_ETHERLAB "Use Etherlab Ethercat drivers" OFF)
	if(USE_ETHERLAB)
		message("using etherlab")
		add_definitions(-DUSE_ETHERLAB)
		set(ALL_INCL_DIR "/opt/etherlab/include" ${ALL_INCL_DIR})
		set(ALL_LINK_DIR "/opt/etherlab/lib" ${ALL_LINK_DIR})
		set(ALL_LINK_LIB ethercat_rtdm ${ALL_LINK_LIB})
	endif(USE_ETHERLAB)
endif(UNIX)
if(WIN32)
	set(ALL_LINK_LIB "ws2_32")#ws2_32 is needed for socket
endif(WIN32)

set(ARIS_INCL_DIR "C:/aris/include")
set(ARIS_LINK_DIR "C:/aris/lib")
set(ARIS_LINK_LIB aris_server aris_sensor aris_robot aris_dynamic aris_control aris_core)

set(ALL_INCL_DIR ${ARIS_INCL_DIR} ${ALL_INCL_DIR})
set(ALL_LINK_DIR ${ARIS_LINK_DIR} ${ALL_LINK_DIR})
set(ALL_LINK_LIB ${ARIS_LINK_LIB} ${ALL_LINK_LIB})

include_directories(${ALL_INCL_DIR})
link_directories(${ALL_LINK_DIR})



set(SOURCE_FILES src/main.cpp)
add_executable(my_project ${SOURCE_FILES})
target_link_libraries(my_project ${ALL_LINK_LIB})
