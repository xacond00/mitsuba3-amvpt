#pragma once
/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/

#include <cmath>
#include <cstdint>
#include <string>
#include <type_traits>

// Macros to reduce vector math boilerplate

#define OP_VEC(OP)                                                                                                     \
    vec_t &operator OP##=(const vec_t & b) {                                                                           \
        vec_t &a = *this;                                                                                              \
        for (uint32_t i = 0; i < N; i++)                                                                               \
            a[i] OP## = b[i];                                                                                          \
        return a;                                                                                                      \
    }                                                                                                                  \
    friend vec_t operator OP(vec_t a, const vec_t &b) {                                                                \
        for (uint32_t i = 0; i < N; i++)                                                                               \
            a[i] OP## = b[i];                                                                                          \
        return a;                                                                                                      \
    }

#define BIN_STD(OP)                                                                                                    \
    friend vec_t OP(vec_t a, const vec_t &b) {                                                                         \
        for (uint32_t i = 0; i < N; i++)                                                                               \
            a[i] = std::OP(a[i], b[i]);                                                                                \
        return a;                                                                                                      \
    }

#define UN_STD(OP)                                                                                                     \
    friend vec_t OP(vec_t a) {                                                                                         \
        for (uint32_t i = 0; i < N; i++)                                                                               \
            a[i] = std::OP(a[i]);                                                                                      \
        return a;                                                                                                      \
    }

namespace mitsuba {

// Misc functions
inline std::string strip(std::string str) {
    auto beg = str.find_first_not_of(' ');
    if (beg == std::string::npos)
        return "";
    auto end = str.find_last_not_of(' ');
    return str.substr(beg, end - beg + 1);
}

inline std::string wrap(std::string str) {
    str = strip(str);
    return str.find(' ') != str.npos ? '"' + str + '"' : str;
}

inline bool blank(const std::string &str) {
    for (const auto &c : str) {
        if (c != ' ' && c != '\0')
            return false;
    }
    return true;
}

template <typename T> inline T clip(T x) { return std::min(std::max(x, T(0)), T(1)); }
template <typename T, typename Tp> inline T mod(T x, Tp y) { return x - y * std::floor(x / y); }
template <typename T, typename Tp> T lerp2(T a, T b, Tp t) { return (Tp(1) - t) * a + t * b; }

// Standalone vector implementation to fit my needs
template <class T, uint32_t N> struct vec_t {

    vec_t() {}

    template <typename U = T, typename std::enable_if_t<(N > 1), int> = 0> vec_t(T t) {
        for (uint32_t i = 0; i < N; i++)
            data[i] = t;
    }

    template <typename... Args, typename std::enable_if_t<(sizeof...(Args) > 0 && sizeof...(Args) < N), int> = 0>
    vec_t(T t, Args... args) : data{ t, T(args)... } {}

    template <class Tp> explicit vec_t(const vec_t<Tp, N> &v) {
        for (uint32_t i = 0; i < N; i++) {
            data[i] = v[i];
        }
    }

    T operator[](uint32_t i) const { return data[i]; }
    T &operator[](uint32_t i) { return data[i]; }

    operator T *() { return data; }
    operator const T *() const { return data; }
    BIN_STD(max)
    BIN_STD(min)
    UN_STD(floor)
    UN_STD(abs)

    friend vec_t v_clip(vec_t a) {
        for (uint32_t i = 0; i < N; i++) {
            a[i] = clip(a[i]);
        }
        return a;
    }

    OP_VEC(+)
    OP_VEC(-)
    OP_VEC(*)
    OP_VEC(/)

    T data[N] = {};
};

using vec2u = vec_t<uint32_t, 2>;
using vec2i = vec_t<int, 2>;
using vec2f = vec_t<float, 2>;

// Function to get interpolated value inside image using either nearest neighbor or bilinear
template <class T>
inline T interpolate2d(const T *data, const vec2f &xy, const vec2u &wh, uint32_t c, uint32_t nch,
                       bool nearest = false) {

    float x = xy[0], y = xy[1];
    uint32_t w = wh[0], h = wh[1];

    if (nearest) {
        uint32_t x1 = x + 0.5f;
        uint32_t y1 = y + 0.5f;
        return data[(x1 + y1 * w) * nch + c];
    }
    uint32_t x1 = x;
    uint32_t y1 = y;
    float tx    = x - x1;
    float ty    = y - y1;
    T a00, a10, a01, a11;
    uint32_t x2 = x1 + 1 < w ? x1 + 1 : x1;
    uint32_t y2 = y1 + 1 < h ? y1 + 1 : y1;
    a00         = data[(x1 + y1 * w) * nch + c];
    a10         = data[(x2 + y1 * w) * nch + c];
    a01         = data[(x1 + y2 * w) * nch + c];
    a11         = data[(x2 + y2 * w) * nch + c];

    return lerp2(lerp2(a00, a10, tx), lerp2(a01, a11, tx), ty);
}

} // namespace mitsuba

#undef BIN_STD
#undef UN_STD
#undef OP_VEC