#pragma once
#include <mitsuba/core/bitmap.h>
namespace mitsuba {
void init_jit();
void exit_jit();
int render(int argc, char *argv[]);
} // namespace mitsuba
