cmake_minimum_required(VERSION 3.10)

project(plainH264)

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

set(CMAKE_CXX_STANDARD 17)

if (MSVC)
    SET(CMAKE_CXX_FLAGS "/nologo /DWIN32=1 /D_WINDOWS /W3 /GR /EHsc /MP /arch:AVX2 /GF /guard:cf- /wd4996 /wd4068 /wd5033 /w14388 /w14389 /w14101 /w14189 /std:c++17 /GS- /Gw /Qspectre- /fp:fast")
    SET(CMAKE_CXX_FLAGS_DEBUG "/D_DEBUG /MTd /ZI /Ob0 /Od /RTC1 /Gy")
    SET(CMAKE_C_FLAGS "/nologo /DWIN32=1 /D_WINDOWS /W3 /MP /arch:AVX2 /GF /guard:cf- /wd4996 /wd4068 /Qspectre-")
    SET(CMAKE_C_FLAGS_DEBUG "/D_DEBUG /MTd /ZI /Ob0 /Od /RTC1 /Gy")
    add_compile_options("/MP")
endif()

if (UNIX)
    add_definitions(-DUNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -fvisibility=hidden -Wno-unused-function -Wno-catch-value -Wno-sign-compare -Wno-int-in-bool-context")
	find_package( Threads REQUIRED )
    link_libraries(${CMAKE_THREAD_LIBS_INIT})
endif()

add_definitions(-DGLEW_NO_GLU -DCMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_subdirectory(shared)
add_subdirectory(viewer)

set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "viewer")
