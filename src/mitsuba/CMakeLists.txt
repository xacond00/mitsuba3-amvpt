include_directories(
  ${ASMJIT_INCLUDE_DIRS}
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(gui)
add_executable(mitsuba-bin mitsuba.cpp mitsuba_render.cpp window.cpp program.cpp)
target_link_libraries(mitsuba-bin PRIVATE mitsuba GUI)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  target_link_libraries(mitsuba-bin PRIVATE asmjit)
endif()

if (UNIX AND NOT APPLE)
  target_link_libraries(mitsuba-bin PRIVATE dl)
endif()

set_target_properties(mitsuba-bin PROPERTIES OUTPUT_NAME mitsuba)
