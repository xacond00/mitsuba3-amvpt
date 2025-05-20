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
        m_menu     = Window("Menu", 540, 720, m_imgui_menu);
        m_view_res = vec2i(m_view.dims());
        load_image(m_image_path);
        Preset::merge_file(m_presets, "presets.csv");
        fs::create_directory("output");
        fs::create_directory("input");
        fs::create_directory("scenes");
        // SDL_SetWindowAlwaysOnTop(m_menu.window(), true);
    }
    ~Program() {
        m_view.free();
        m_menu.free();
        SDL_Quit();
    }

    int run();

private:
    /* Menu definition that is passed to m_view */
    std::function<void()> m_imgui_menu = [this]() { imgui_menu(); };
    void imgui_menu();
    void display_image(bool screenshot = false);
    /* load image */
    void load_image(fs::path path) {
        const char *extensions[] = { ".exr", ".rgbe", ".png", ".jpg", ".jpeg", ".pfm", ".tga" };
        bool exists              = fs::exists(path);
        auto ext                 = path.extension();
        if (ext != "") {
            exists = false;
            for (uint32_t i = 0; i < 6; i++) {
                if (ext == extensions[i] && fs::exists(path)) {
                    exists = true;
                    path   = path.replace_extension(ext);
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
                /* Needs Mitsuba JIT instance to be running locally */
                m_image       = result.convert(Bitmap::PixelFormat::RGB, Struct::Type::UInt8, true);
                m_update_view = true;
            }
            Log(Info, "Loaded image: %s", path);
            m_image_path  = path.string();
            m_export_path = "output/" + path.filename().replace_extension("png").string();
        } else
            Log(Warn, "Path doesn't exist !");
    }
    /* Export quilt as png */
    void export_quilt() const {
        if(m_export_path == m_image_path){
            Log(Warn, "Can't overwrite source image !");
        }
        auto path = fs::path(m_export_path);
        if (fs::exists(path.parent_path()) || path.parent_path().empty()) {
            m_image->write(path);
            Log(Info, "Image saved to: %s", path.replace_extension("png"));
        } else{
            Log(Warn, "Path doesn't exist !");
        }
    }
    /* Render through Mitsuba CLI */
    void mitsuba_render() {
        // We have to launch whole new instance of this program
        // that launches separate JIT session
        // Otherwise *Crash*
        int ret = std::system(m_args.get_args());
        if (ret == 0) {
            fs::path path = m_args.get_output();
            if (blank(path.string())) {
                path = m_args.scene;
                path.replace_extension("exr");
            }
            load_image(path);
        }
    }
    const char* theme_str() const{
        return !m_theme ? "Dark" : m_theme == 1 ? "Light" : "Classic";
    };
    MiArgs m_args;
    ref<Bitmap> m_image;
    Preset m_default_preset = { "LKG4x8", -0.5f,           -0.05f,      354.548f, -0.114f,
                                0.00013f, vec2f(1.f, 1.f), vec2i(4, 8), false };
    Preset m_preset{ m_default_preset };
    std::vector<Preset> m_presets{ m_default_preset };
    Window m_view;
    Window m_menu;
    SDL_Event event;
    std::string m_image_path  = "input/1.exr";
    std::string m_export_path = "output/1.png";
    vec2i m_view_res;
    uint32_t m_sel_pres = -1;
    int m_theme = 0;
    bool m_running      = true;
    bool m_quilt        = false;
    bool m_nearest      = false;
    bool m_update_view  = false;
    bool m_keep_aspect  = false;
};

} // namespace mitsuba