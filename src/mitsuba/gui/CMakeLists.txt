add_library(GUI INTERFACE)
add_subdirectory(SDL)

include_directories(
    SDL/include/
    imgui/
    imgui/backends
    imgui/misc/cpp
)

add_library(imgui
    imgui/imgui.cpp 
    imgui/imgui_draw.cpp
    imgui/imgui_demo.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/backends/imgui_impl_sdl3.cpp  # Only SDL backend
    imgui/backends/imgui_impl_sdlrenderer3.cpp  # Only SDL backend
    imgui/misc/cpp/imgui_stdlib.cpp
)

target_include_directories(GUI INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/SDL/include
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/misc/cpp
)
target_compile_options(imgui PRIVATE -O3)
target_link_libraries(GUI INTERFACE SDL3::SDL3 imgui)