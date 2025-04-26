#pragma once
/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/

#include "util.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace mitsuba {

struct Preset {
    std::string name;
    float center, focus, pitch, tilt, subp;
    vec2f view;
    vec2i grid;
    bool flip_rgb;
    // Export a preset to an output stream (semicolon-separated format)
    friend std::ostream &operator<<(std::ostream &os, const Preset &p) {
        os << p.name << ';' << p.center << ';' << p.focus << ';' << p.pitch << ';' << p.tilt << ';' << p.subp << ';'
           << p.view[0] << ';' << p.view[1] << ';' << p.grid[0] << ';' << p.grid[1] << ';' << p.flip_rgb << '\n';
        return os;
    }
    operator std::string()const{
        std::ostringstream ss;
        ss << *this;
        return ss.str();
    }

    // Import a preset from an input stream
    friend std::istream &operator>>(std::istream &is, Preset &p) {
        std::string line;
        if (std::getline(is, line)) {
            std::stringstream ss(line);
            std::getline(ss, p.name, ';');
            ss >> p.center;
            ss.ignore();
            ss >> p.focus;
            ss.ignore();
            ss >> p.pitch;
            ss.ignore();
            ss >> p.tilt;
            ss.ignore();
            ss >> p.subp;
            ss.ignore();
            ss >> p.view[0];
            ss.ignore();
            ss >> p.view[1];
            ss.ignore();
            ss >> p.grid[0];
            ss.ignore();
            ss >> p.grid[1];
            ss.ignore();
            ss >> p.flip_rgb;
        }
        return is;
    }

    // Save all presets to a file (overwrite)
    static void export_file(const std::vector<Preset> &presets, const std::string &filename) {
        std::ofstream file(filename);
        if (!file) {
            std::cerr << "Error opening file for writing: " << filename << '\n';
            return;
        }
        for (const auto &p : presets) {
            file << p;
        }
    }

    // Load all presets from a file
    static std::vector<Preset> import_file(const std::string &filename) {
        std::vector<Preset> presets;
        std::ifstream file(filename);
        if (!file) {
            std::cerr << "Error opening file for reading: " << filename << '\n';
            return presets;
        }
        Preset p;
        while (file >> p) {
            presets.push_back(p);
        }
        return presets;
    }

    static void update_list(const Preset &preset, std::vector<Preset> &presets) {
        for (auto &p : presets) {
            if (p.name == preset.name) {
                p = preset;
                return;
            }
        }
        presets.push_back(preset);
    }

    static void update_list(const std::vector<Preset> &new_presets, std::vector<Preset> &presets) {
        for (const auto &p : new_presets) {
            update_list(p, presets);
        }
    }

    static void merge_file(std::vector<Preset> &new_presets, const std::string &filename) {
        std::vector<Preset> presets = import_file(filename);
        update_list(new_presets, presets);
        export_file(presets, filename);
        new_presets = presets;
    }
};

} // namespace mitsuba