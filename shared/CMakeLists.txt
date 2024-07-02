project(shared)

set(files
	math.cpp
	math.h
	std_ext.cpp
	std_ext.h
	output.cpp
	output.h
	file.cpp
	file.h
	time.cpp
	time.h
)

source_group("files" FILES ${files})

add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_USE_MATH_DEFINES)

add_library(${PROJECT_NAME} ${files})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR})
