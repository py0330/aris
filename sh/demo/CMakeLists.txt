cmake_minimum_required(VERSION 3.12)

project(my_project)

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

set(ARIS_INCL_DIR "C:/aris/include")
set(ARIS_LINK_DIR "C:/aris/lib")
set(ARIS_LINK_LIB aris_robot aris_server aris_sensor aris_plan aris_dynamic aris_control aris_core)

set(ALL_INCL_DIR ${ARIS_INCL_DIR} ${ALL_INCL_DIR})
set(ALL_LINK_DIR ${ARIS_LINK_DIR} ${ALL_LINK_DIR})
set(ALL_LINK_LIB ${ARIS_LINK_LIB} ${ALL_LINK_LIB})

include_directories(${ALL_INCL_DIR})
link_directories(${ALL_LINK_DIR})

set(SOURCE_FILES src/main.cpp)
add_executable(my_project ${SOURCE_FILES})
target_link_libraries(my_project ${ALL_LINK_LIB})
