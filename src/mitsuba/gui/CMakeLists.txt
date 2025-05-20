# Add SDL subdirectory (assumes SDL3's CMakeLists.txt is configured correctly)
add_subdirectory(SDL)

# Build ImGui static library
add_library(imgui STATIC
    imgui/imgui.cpp 
    imgui/imgui_draw.cpp
    imgui/imgui_demo.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/backends/imgui_impl_sdl3.cpp
    imgui/backends/imgui_impl_sdlrenderer3.cpp
    imgui/misc/cpp/imgui_stdlib.cpp
)

target_include_directories(imgui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/misc/cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SDL/include
)

target_compile_options(imgui PRIVATE -O3)

add_library(GUI STATIC link.cpp) 

target_include_directories(GUI PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/SDL/include
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/misc/cpp
)

target_link_libraries(GUI PUBLIC SDL3::SDL3 imgui)
