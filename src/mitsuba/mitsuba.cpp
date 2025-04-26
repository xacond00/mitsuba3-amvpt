/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/

#include "mitsuba_render.h"
#include "program.h"
int main(int argc, char *argv[]) {
    mitsuba::init_jit();
    mitsuba::fs::path path = argv[0];
    path = path.filename();
    #if defined(_WIN32)
    path = path.replace_extension(".exe");
    const char* comspec = std::getenv("COMSPEC");
    if (comspec) {
        std::string comspec_str = comspec;
        if(comspec_str.find("powershell") != std::string::npos){
            path = "./" + path.string();
        }
    }
    #else
    path = "./" + path.string();
    #endif
    int ret = 0;
    if (argc > 1) {
        ret = mitsuba::render(argc, argv);
    } else {
        mitsuba::Program prog(path.string());
        ret = prog.run();
    }
    mitsuba::exit_jit();
    return ret;
}