/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/

#include "program.h"
#include <cmath>
#include <filesystem>
#include <imgui_stdlib.h>
#include <nanothread/nanothread.h>
#include <thread>
namespace mitsuba {

int Program::run() {
    m_running = true;
    while (m_running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                m_running = false;
                break;
            }

            if (m_view.scan_event(event)) {
                m_update_view = true;
                m_view_res    = vec2i(m_view.dims());
                // SDL_SetWindowAlwaysOnTop(m_menu.window(), true);
                // SDL_SetWindowAlwaysOnTop(m_view.window(), false);
            }
            if (m_menu.scan_event(event)) {
            }
            if (event.type == SDL_EVENT_DROP_FILE) {
                mitsuba::fs::path path = event.drop.data;
                if (path.extension() == ".xml") {
                    auto abs_path        = path.string();
                    std::string rel_path = std::filesystem::relative(abs_path).string();
                    m_args.scene         = abs_path.length() < rel_path.length() ? abs_path : rel_path;
                    m_args.update_args();
                } else {
                    load_image(path);
                }
            }
        }
        m_running = m_view.valid() || m_menu.valid();
        display_image();
        m_view.render();
        m_menu.render();
        // SDL_Delay(10);
    }
    return 0;
}

void Program::imgui_menu() {
    using namespace ImGui;
    SetNextWindowPos({ 0, 0 });
    SetNextWindowSize({ (float) m_menu.width(), (float) m_menu.height() });

    Begin("Menu", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);

    if (CollapsingHeader("Rendering menu")) {
        if (Button("Render !")) {
            mitsuba_render();
        }
        bool args_changed = false;
        SameLine();
        args_changed |= Checkbox("Replace view", &m_args.bitmap);
        args_changed |= InputText("Scene", &m_args.scene);
        args_changed |= SliderInt("Spp", &m_args.spp, 0, 16384, "%d");
        args_changed |= InputText("Variant", &m_args.variant);
        args_changed |= InputText("Output name", &m_args.output);
        args_changed |= InputText("Args", &m_args.extra);
        args_changed |= SliderInt("Optim. level", &m_args.Olvl, 0, 5);
        // int spp
        if (args_changed) {
            m_args.update_args();
        }
        PushTextWrapPos();
        Text("%s", m_args.args.c_str());
        PopTextWrapPos();
    }

    Separator();

    if (CollapsingHeader("Viewing menu")) {
        bool load_im = InputText("##load_img", &m_image_path, ImGuiInputTextFlags_EnterReturnsTrue);
        SameLine();
        load_im |= Button("Load##img");
        if (load_im && !blank(m_image_path))
            load_image(m_image_path);
        bool save_im = InputText("##save_img", &m_export_path, ImGuiInputTextFlags_EnterReturnsTrue);
        SameLine();
        save_im |= Button("Screenshot");
        bool path_good = !blank(m_export_path);

        if (save_im && path_good) {
            display_image(true);
        }

        if (InputInt2("##Resolution", m_view_res, ImGuiInputTextFlags_EnterReturnsTrue)) {
            auto &res = m_view_res;
            auto orig = m_view.dims();
            res       = min(max(res, 200), 8192);
            if (m_keep_aspect) {
                double asp = orig[0] / double(orig[1]);
                if (uint32_t(res[0]) != orig[0]) {
                    res[1] = res[0] / asp;
                } else if (uint32_t(res[1]) != orig[1]) {
                    res[0] = res[1] * asp;
                }
            }
            if (uint32_t(res[0]) != orig[0] || uint32_t(res[1]) != orig[1]) {
                m_update_view = true;
                SDL_SetWindowSize(m_view.window(), res[0], res[1]);
            }
        }
        SameLine();
        if (Checkbox("Keep aspect", &m_keep_aspect)) {
            if (m_keep_aspect) {
                auto orig  = m_view.dims();
                double asp = orig[0] / double(orig[1]);
                SDL_SetWindowAspectRatio(m_view.window(), asp, asp);
            } else {
                SDL_SetWindowAspectRatio(m_view.window(), 0, 1e6);
            }
        }
        if (Button("Aspect to quilt") && m_image) {
            double asp = m_image->width() / double(m_image->height());
            SDL_SetWindowAspectRatio(m_view.window(), asp, asp);
            if (!m_keep_aspect) {
                SDL_SetWindowAspectRatio(m_view.window(), 0, 1e6);
            }
        }
        SameLine();
        if (Button("Export quilt to PNG") && path_good) {
            export_quilt();
        }
        m_update_view |= load_im;
        m_update_view |= Checkbox("Display quilt", &m_quilt);
        SameLine();
        m_update_view |= Checkbox("Nearest scaling", &m_nearest);
    }
    
    Separator();
    
    if (CollapsingHeader("Presets menu")) {
        Text("Database:"), SameLine();
        if (Button("Import##presets")) {
            auto result = Preset::import_file("presets.csv");
            if (!result.empty()) {
                std::swap(result, m_presets);
            }
        }
        SameLine();
        if (Button("Export##presets")) {
            Preset::export_file(m_presets, "presets.csv");
        }
        SameLine();
        if (Button("Merge##presets")) {
            Preset::merge_file(m_presets, "presets.csv");
        }
        Text("%s", Preset::line_header());
        float height = ImGui::GetTextLineHeightWithSpacing() * std::min(m_presets.size(), 10UL);
        if (ImGui::BeginListBox("##Preset list", ImVec2(-1, height))) {
            for (uint32_t i = 0; i < m_presets.size(); i++) {
                std::string name = std::string(m_presets[i]);
                bool is_selected = (m_sel_pres == i);
                if (ImGui::Selectable(name.c_str(), is_selected)) {
                    m_sel_pres = i;
                }
                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
                    m_update_view    = true;
                    m_default_preset = m_preset = m_presets[m_sel_pres];
                }
                if (is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndListBox();
        }
        /*
        { // Table format overflows -> not used currently
            auto header = m_preset.table_header();
            if (ImGui::BeginTable("CSV Table", header.size(), ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                //ImGui::TableSetupScrollFreeze(0,1); // freeze header row
                for (size_t col = 0; col < header.size(); ++col) {
                    ImGui::TableSetupColumn(header[col].c_str());
                }
                ImGui::TableHeadersRow();

                for (size_t row = 0; row < m_presets.size(); ++row) {
                    ImGui::TableNextRow();
                    for (size_t col = 0; col < header.size(); ++col) {
                        auto items = m_preset.table_row();
                        ImGui::TableSetColumnIndex(col);
                        ImGui::TextUnformatted(items[col].c_str());
                    }
                }

                ImGui::EndTable();
            }
        }*/
        Text("Preset:"), SameLine();
        if (Button("Load##preset") && m_sel_pres < m_presets.size()) {
            m_update_view    = true;
            m_default_preset = m_preset = m_presets[m_sel_pres];
        }
        SameLine();
        if (Button("Save##preset")) {
            Preset::update_list(m_preset, m_presets);
        }
        SameLine();
        if (Button("Reset##preset")) {
            m_update_view = true;
            m_preset      = m_default_preset;
        }
        SameLine();
        if (Button("Erase##preset") && m_sel_pres < m_presets.size()) {
            m_presets.erase(m_presets.begin() + m_sel_pres);
            if (m_sel_pres == m_presets.size()) {
                m_sel_pres--;
            }
        }
        m_update_view |= InputText("Preset Name", &m_preset.name);
        m_update_view |= SliderFloat("Focus", &m_preset.focus, -1, 1);
        m_update_view |= SliderFloat("Center", &m_preset.center, -1, 1);
        m_update_view |= DragFloat("Pitch", &m_preset.pitch, 1, 1200);
        m_update_view |= SliderFloat("Tilt", &m_preset.tilt, -1, 1);
        m_update_view |= SliderFloat("Subp", &m_preset.subp, -1e-3f, 1e-3f, "%.4e");
        m_update_view |= SliderFloat2("View", m_preset.view, 0, 1);
        m_update_view |= SliderInt2("Grid", m_preset.grid, 1, 32);
        m_update_view |= Checkbox("Flip RGB", &m_preset.flip_rgb);
    }
    
    Separator();
    
    if (CollapsingHeader("Misc")) {
        if (SliderInt("Theme", &m_theme, 0, 2, theme_str())) {
            switch (m_theme) {
                case 0:
                    ImGui::StyleColorsDark();
                    break;
                case 1:
                    ImGui::StyleColorsLight();
                    break;
                default:
                    ImGui::StyleColorsClassic();
                    break;
            }
        }
        Text("%.2f fps", (double) GetIO().Framerate);
    }
    
    Separator();
    
    End();
}

void Program::display_image(bool screenshot) {
    if (!m_image || (!m_update_view && !screenshot))
        return;

    const auto &p = m_preset;
    uint8_t *dst;
    vec2u dst_dim;
    uint32_t dst_ch = 4;
    bool flip_rgb   = p.flip_rgb;

    ref<Bitmap> img;
    if (screenshot) {
        dst_dim = vec2u(m_view.width(), m_view.height());
        img     = new Bitmap(Bitmap::PixelFormat::RGB, Struct::Type::UInt8, Bitmap::Vector2u(dst_dim[0], dst_dim[1]));
        dst_ch  = 3;
        dst_dim[0] *= dst_ch;
        flip_rgb = true;
        dst      = img->uint8_data();
    } else {
        std::tie(dst, dst_dim) = m_view.get_surf();
    }
    flip_rgb &= dst_ch > 2;
    // Get raw data
    const uint8_t *src = m_image->uint8_data();
    uint32_t src_ch    = m_image->channel_count();
    vec2u src_dim(m_image->width(), m_image->height());
    float grid_n   = p.grid[0] * p.grid[1];
    float grid_scl = 1.f / grid_n;
    auto igrid     = 1.f / vec2f(p.grid);

    auto dst_scl = 1.f / (vec2f(dst_dim) - vec2f(1.f));
    auto src_scl = vec2f(src_dim) - vec2f(1.f);

    uint32_t threads = std::thread::hardware_concurrency();
    threads          = threads > 0 ? threads : 4;
    uint block_size  = std::max(dst_dim[1] / threads, 1U);
    
    // Interlacing algorithm inspired by: https://git.fit.vutbr.cz/imilet/FitGraphics/src/branch/master/src/LKG/
    // Parallel CPU for loop, needs scalar JIT instance to be running
    dr::parallel_for(dr::blocked_range<size_t>(0, dst_dim[1], block_size), [&](const dr::blocked_range<size_t> &range) {
        for (uint32_t dy = range.begin(); dy < range.end(); dy++) {
            float y0 = dy * dst_scl[1];
            for (uint32_t dx = 0; dx < dst_dim[0]; dx += dst_ch) {
                float x0 = dx * dst_scl[0];
                for (uint32_t c = 0; c < dst_ch; c++) {
                    vec2f xy(x0, y0);
                    if (c >= 3) {
                        dst[dy * dst_dim[0] + dx + c] = 255;
                        continue;
                    }
                    uint32_t ch = flip_rgb ? c : 2 - c;
                    if (!m_quilt) {
                        // y axis has to be flipped here
                        float z = p.pitch * (xy[0] + c * p.subp + (1.f - xy[1]) * p.tilt) - p.center;
                        z       = 1.f - (z - floorf(z));
                        // camera id
                        float z1 = floorf(z * grid_n);
                        // inverse camera id
                        float z2 = floorf((1.f - z) * grid_n);
                        // depth
                        float w = p.focus * (1.f - 2.f * clip(z1 * grid_scl));
                        // Find x and y local coordinates
                        xy[0] = mod(z1, (float) p.grid[0]) + clip(xy[0] + w);
                        xy[1] = floorf(z2 * igrid[0]) + xy[1];

                        xy = v_clip(p.view * igrid * xy);
                    }
                    xy                             = src_scl * xy;
                    dst[dy * dst_dim[0] + dx + ch] = interpolate2d(src, xy, src_dim, c, src_ch, m_nearest);
                }
            }
        }
    });
    // Screenshot
    if (screenshot && img) {
        if (m_export_path == m_image_path) {
            Log(Warn, "Can't overwrite source image !");
        }
        auto path = fs::path(m_export_path);
        if (fs::exists(path.parent_path()) || path.parent_path().empty()) {
            img->write(path);
            Log(Info, "Image saved to: %s", path);
        } else {
            Log(Warn, "Path doesn't exist !");
        }
    } else if (m_update_view) {
        m_view.set_surf();
    }
    m_update_view = false;
}

} // namespace mitsuba