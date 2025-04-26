#pragma once
/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/
#include "mi_args.h"
#include "mitsuba/core/filesystem.h"
#include "preset.h"
#include "window.h"
#include <mitsuba/core/bitmap.h>
namespace mitsuba {

class Program {
public:
    Program(const std::string &prog_name) : m_args(prog_name) {
        SDL_Init(SDL_INIT_EVENTS | SDL_INIT_VIDEO | SDL_INIT_GAMEPAD);
        IMGUI_CHECKVERSION();
        m_view     = Window("View", 1280, 720, SDL_PIXELFORMAT_ARGB8888);
        m_menu     = Window("Menu", 480, 720, m_imgui_menu);
        m_view_res = vec2i(m_view.dims());
        load_image(m_image_path);
        Preset::merge_file(m_presets, "presets.csv");
        // SDL_SetWindowAlwaysOnTop(m_menu.window(), true);
    }
    ~Program() {
        m_view.free();
        m_menu.free();
        SDL_Quit();
    }

    int run();

private:
    std::function<void()> m_imgui_menu = [this]() { imgui_menu(); };
    void imgui_menu();
    void display_image(bool screenshot = false);
    void load_image(fs::path path) {
        const char *extensions[] = { ".exr", ".rgbe", ".png", ".jpg", ".jpeg", ".pfm", ".tga" };
        bool exists              = fs::exists(path);
        auto ext                 = path.extension();
        if (ext != "") {
            exists = false;
            for (uint32_t i = 0; i < 6; i++) {
                if (ext == extensions[i] && fs::exists(path)) {
                    exists = true;
                    break;
                }
            }
        }
        for (uint32_t i = 0; i < 6; i++) {
            if (exists)
                break;
            path = path.replace_extension(extensions[i]);
            exists |= fs::exists(path);
        }
        if (exists) {
            auto result = Bitmap(path);
            if (result.height()) {
                m_image       = result.convert(Bitmap::PixelFormat::RGB, Struct::Type::UInt8, true);
                m_update_view = true;
            }
        }
    }
    void export_quilt() const { m_image->write(m_export_path); }
    void mitsuba_render() {
        int ret = std::system(m_args.get_args());
        if (ret == 0) {
            fs::path path = m_args.get_output();
            if (blank(m_args.output)) {
                path = m_args.scene;
                path.replace_extension(".exr");
            }
            load_image(path);
        }
    }
    MiArgs m_args;
    ref<Bitmap> m_image;
    Preset m_default_preset = { "LKG4x8", -0.5f,           -0.05f,      354.548f, -0.114f,
                                0.00013f, vec2f(1.f, 1.f), vec2i(4, 8), false };
    Preset m_preset{ m_default_preset };
    std::vector<Preset> m_presets{ m_default_preset };
    Window m_view;
    Window m_menu;
    SDL_Event event;
    std::string m_image_path  = "../outputs/LKG48/1.exr";
    std::string m_export_path = "../outputs/LKG48/1.png";
    vec2i m_view_res;
    uint32_t m_sel_pres = -1;
    bool m_running      = true;
    bool m_quilt        = false;
    bool m_nearest      = false;
    bool m_update_view  = false;
    bool m_keep_aspect  = false;
};

} // namespace mitsuba