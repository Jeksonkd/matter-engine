#pragma once

#include <cmath>

namespace p2d {

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float x_, float y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator-() const { return {-x, -y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }

    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
    Vec2& operator*=(float s) { x *= s; y *= s; return *this; }

    float dot(const Vec2& o) const { return x * o.x + y * o.y; }
    float cross(const Vec2& o) const { return x * o.y - y * o.x; }
    float length() const { return std::sqrt(x * x + y * y); }
    float lengthSq() const { return x * x + y * y; }

    Vec2 normalized() const {
        float len = length();
        if (len < 1e-9f) return {0.0f, 0.0f};
        return {x / len, y / len};
    }

    // 90 degree counter-clockwise perpendicular
    Vec2 perp() const { return {-y, x}; }
};

inline Vec2 operator*(float s, const Vec2& v) { return v * s; }

// scalar x vector cross product (result is a vector), matches box2d-lite convention
inline Vec2 cross(float s, const Vec2& v) { return {-s * v.y, s * v.x}; }

inline Vec2 rotate(const Vec2& v, float angleRad) {
    float c = std::cos(angleRad);
    float s = std::sin(angleRad);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

} // namespace p2d
