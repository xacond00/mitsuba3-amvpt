#pragma once
/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/

#include <cstdint>
#include <string>
#include <vector>
#include "util.h"

namespace mitsuba {
    
// Class to interface with CLI mitsuba::render() function.
struct MiArgs {
    MiArgs(const std::string &prog_name = "mitsuba") : prog_name(prog_name) { update_args(); }
    std::string args    = "";
    std::string scene   = "../scenes/bedroom/scene.xml";
    #if defined(MI_ENABLE_CUDA)
    std::string variant = "cuda_rgb";
    #elif defined(MI_ENABLE_LLVM)
    std::string variant = "llvm_rgb";
    #else
    std::string variant = "scalar_rgb";
    #endif
    std::string output  = "";
    std::string extra   = "";
    const std::string prog_name;
    int Olvl    = 5;
    int spp     = 0;
    bool bitmap = true;

    std::string get_output() const { return bitmap ? "tmp" : output; }
    // Updates args string from all parameters
    void update_args() {
        std::string out = get_output();
        out             = blank(out) ? "" : "-o " + wrap(out) + ' ';
        std::string var = blank(variant) ? "" : "-m " + wrap(variant) + ' ';
        std::string scn = blank(scene) ? "" : wrap(scene) + ' ';
        std::string ext = blank(extra) ? "" : strip(extra) + ' ';
        std::string str = prog_name + ' ' + strip(var + scn + out + ext) + " -n " + std::to_string(spp) + " -O" +
                          std::to_string(Olvl);
        std::swap(str, args);
    }
    const char *get_args() {
        update_args();
        return args.c_str();
    }

    // Returns raw arguments as vector<char>
    std::vector<char> raw_args() const {
        auto str = strip(args);
        std::vector<char> result(str.begin(), str.end());
        result.push_back('\0');
        return result;
    }

    // Create list of argv from monolithic string of argmuents
    const std::vector<char *> parse_args(std::vector<char> &args) const {
        args = raw_args();
        std::vector<char *> argv;
        argv.reserve(args.size() / 4 + 1);
        char *arg_ptr = nullptr;
        bool in_str   = false;

        if (args.empty() || args[0] == '\0')
            return { nullptr };

        for (uint32_t i = 0; i < args.size(); i++) {
            auto &c = args[i];

            if (c == '"') {
                in_str = !in_str;
                if (in_str) {
                    arg_ptr = args.data() + i + 1;
                } else {
                    argv.push_back(arg_ptr);
                    c       = '\0';
                    arg_ptr = nullptr;
                }
            } else if (!in_str && (c == ' ' || c == '\0')) {
                if (arg_ptr) {
                    argv.push_back(arg_ptr);
                    arg_ptr = nullptr;
                }
                c = '\0';
            } else if (!arg_ptr) {
                arg_ptr = args.data() + i;
            }
        }

        if (arg_ptr) {
            argv.push_back(arg_ptr);
        }

        argv.push_back(nullptr);
        return argv;
    }
};

} // namespace mitsuba