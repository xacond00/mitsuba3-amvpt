include_directories(${ASMJIT_INCLUDE_DIRS})

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(gui)

add_executable(mitsuba-bin mitsuba.cpp mitsuba_render.cpp window.cpp program.cpp)

target_compile_options(mitsuba-bin PRIVATE -O3)

target_link_libraries(mitsuba-bin PRIVATE mitsuba GUI)

add_custom_command(TARGET mitsuba-bin POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:SDL3::SDL3>  # gets the path to the built SDL3.dll
    $<TARGET_FILE_DIR:mitsuba-bin>
)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  target_link_libraries(mitsuba-bin PRIVATE asmjit)
endif()

if (UNIX AND NOT APPLE)
  target_link_libraries(mitsuba-bin PRIVATE dl)
endif()

set_target_properties(mitsuba-bin PROPERTIES OUTPUT_NAME mitsuba)
