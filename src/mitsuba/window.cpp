#include "window.h"

/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/

namespace mitsuba {

Window::Window(const char *title, uint32_t width, uint32_t height, SDL_PixelFormat format,
               const std::function<void()> &imgui_menu) {
    // Create window and renderer
    m_closed     = true;
    bool success = SDL_CreateWindowAndRenderer(title, width, height, SDL_WINDOW_RESIZABLE, &m_window, &m_renderer);
    if (!success) {
        SDL_Log("Error: SDL_CreateRenderer(): %s\n", SDL_GetError());
        return;
    }
    SDL_SetWindowMinimumSize(m_window, 200, 200);
    if (format) {
        m_format = format;
        m_surf   = SDL_CreateTexture(m_renderer, format, SDL_TEXTUREACCESS_STREAMING, width, height);
        if (!m_surf) {
            SDL_Log("Error: SDL_CreateTexture(): %s\n", SDL_GetError());
            return;
        }
    }
    if (imgui_menu) {
        m_imgui_ctx  = ImGui::CreateContext();
        m_imgui_menu = imgui_menu;
        ImGuiIO &io  = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
        ImGui::SetCurrentContext(m_imgui_ctx);
        ImGui::StyleColorsDark();
        ImGui_ImplSDL3_InitForSDLRenderer(m_window, m_renderer);
        ImGui_ImplSDLRenderer3_Init(m_renderer);
    }
    m_title          = title;
    m_mouse_focus    = false;
    m_keyboard_focus = false;
    m_width          = width;
    m_height         = height;

    SDL_SetRenderVSync(m_renderer, 1);
    SDL_SetRenderDrawColor(m_renderer, 0, 0, 0, 0);
    m_windowID   = SDL_GetWindowID(m_window);
    m_fullscreen = SDL_GetWindowFullscreenMode(m_window);
    m_shown      = true;
    m_closed     = false;
}

bool Window::scan_event(SDL_Event &e) {
    if (e.window.windowID != m_windowID || !valid())
        return false;
    if (imgui()) {
        ImGui::SetCurrentContext(m_imgui_ctx);
        ImGui_ImplSDL3_ProcessEvent(&e);
    }
    switch (e.type) {
        case SDL_EVENT_WINDOW_SHOWN:
            m_shown = true;
            break;

        case SDL_EVENT_WINDOW_HIDDEN:
            m_shown = false;
            break;
        case SDL_EVENT_WINDOW_EXPOSED:
            break;
        case SDL_EVENT_WINDOW_RESIZED:
            return resize(e.window.data1, e.window.data2);
            break;
        case SDL_EVENT_WINDOW_MINIMIZED:
            m_minimized = true;
            break;

        case SDL_EVENT_WINDOW_MAXIMIZED:
            m_minimized = false;
            break;

        case SDL_EVENT_WINDOW_RESTORED:
            m_minimized = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_ENTER:
            m_mouse_focus = true;
            break;

        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            m_mouse_focus = false;
            break;

        case SDL_EVENT_WINDOW_FOCUS_GAINED:
            m_keyboard_focus = true;
            break;

        case SDL_EVENT_WINDOW_FOCUS_LOST:
            m_keyboard_focus = false;
            break;

        case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
            close();
            break;
        case SDL_EVENT_DROP_FILE:

            break;
        case SDL_EVENT_KEY_UP: {
            // codes[e.key.scancode] = false;
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            if (e.key.scancode == SDL_SCANCODE_F11) {
                m_fullscreen = !m_fullscreen;
                SDL_SetWindowFullscreen(m_window, m_fullscreen);
                SDL_SyncWindow(m_window);
            }
            if (e.key.scancode == SDL_SCANCODE_ESCAPE) {
                if (m_fullscreen) {
                    SDL_SetWindowFullscreen(m_window, false);
                    SDL_SyncWindow(m_window);
                    m_fullscreen = false;
                } else {
                    close();
                }
            }
            // codes[e.key.scancode] = true;
            break;
        }
        default:
            break;
    }
    return false;
}

void Window::render() {
    if (!m_minimized && valid()) {
        // Clear screen
        SDL_SetRenderDrawColor(m_renderer, 0, 0, 0, 0);
        SDL_RenderClear(m_renderer);
        if (m_surf && !m_locked_surf) {
            SDL_RenderTexture(m_renderer, m_surf, nullptr, nullptr);
        }
        if (imgui()) {
            ImGui::SetCurrentContext(m_imgui_ctx);
            ImGui_ImplSDLRenderer3_NewFrame();
            ImGui_ImplSDL3_NewFrame();
            ImGui::NewFrame();
            m_imgui_menu();
            ImGui::Render();
            ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), m_renderer);
        }
        // Update screen
        SDL_RenderPresent(m_renderer);
    }
}

std::tuple<uint8_t *, vec2u> Window::get_surf() {
    unsigned char *pixels = nullptr;
    int pitch             = 0;
    if (!m_locked_surf && m_surf) {
        SDL_LockTexture(m_surf, nullptr, (void **) &pixels, &pitch);
        m_locked_surf = true;
    }
    return { pixels, vec2u(uint32_t(pitch), height()) };
}

void Window::set_surf() {
    if (m_locked_surf && m_surf) {
        SDL_UnlockTexture(m_surf);
        m_locked_surf = false;
    }
}

bool Window::resize(uint32_t width, uint32_t height) {
    if (height && width && (height != m_height || width != m_width)) {
        m_width  = width;
        m_height = height;
        if (m_surf) {
            SDL_DestroyTexture(m_surf);
            m_surf = SDL_CreateTexture(m_renderer, m_format, SDL_TEXTUREACCESS_STREAMING, width, height);
        }
        return true;
    }
    return false;
}

void Window::free() {
    if (imgui()) {
        ImGui::SetCurrentContext(m_imgui_ctx);
        ImGui_ImplSDLRenderer3_Shutdown();
        ImGui_ImplSDL3_Shutdown();
        ImGui::DestroyContext();
    }
    if (m_surf) {
        SDL_DestroyTexture(m_surf);
        m_surf = nullptr;
    }
    if (m_renderer) {
        SDL_DestroyRenderer(m_renderer);
        m_renderer = nullptr;
    }
    if (m_window) {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }
    m_closed = true;
}

} // namespace mitsuba