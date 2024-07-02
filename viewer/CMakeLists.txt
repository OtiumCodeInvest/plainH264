
project(viewer)

include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rdparty ${CMAKE_SOURCE_DIR}/3rdparty/glew-2.1.0/include ${CMAKE_SOURCE_DIR}/3rdparty/glfw/include ${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking)
add_definitions(-DGLEW_STATIC)

set(src_files_imgui
	${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui.h
	${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui.cpp
	${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui_demo.cpp
	${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui_draw.cpp
	${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui_tables.cpp
	${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui_widgets.cpp
	${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui_internal.h
	${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/backends/imgui_impl_glfw.cpp
	${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/backends/imgui_impl_glfw.h
	${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/backends/imgui_impl_opengl3.cpp
	${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/backends/imgui_impl_opengl3.h
)

set(FILES
	decodeimage.cpp
	encodeimage.cpp
	writetext.cpp
	viewer.cpp
	atlas.cpp
	${CMAKE_SOURCE_DIR}/3rdparty/glew-2.1.0/src/glew.c
)

find_package(OpenGL REQUIRED)

if(MSVC)
	add_executable(${PROJECT_NAME} WIN32 ${FILES} ${src_files_imgui})
	target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/3rdparty/glfw/lib-vc2022/glfw3_mt.lib)
	target_link_libraries(${PROJECT_NAME} shared ws2_32.lib iphlpapi.lib ${LIBRARIES} OpenGL::GL)
	set_property(TARGET ${PROJECT_NAME} PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
	add_executable(${PROJECT_NAME} ${FILES} ${src_files_imgui})
	find_package(glfw3 3.3 REQUIRED)
	target_link_libraries(${PROJECT_NAME} shared glfw OpenGL::GL ${CMAKE_DL_LIBS})
endif()
