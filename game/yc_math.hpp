#pragma once

#include <cmath>

namespace yc_math
{
    struct vec3_t {
        float x;
        float y;
        float z;


        vec3_t operator-(const vec3_t& other) const {
            return {x - other.x, y - other.y, z - other.z};
        }

        vec3_t operator/(const float scalar) const {
            return {x / scalar, y / scalar, z / scalar};
        }

        vec3_t& operator+=(const vec3_t& other) {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        vec3_t& operator-=(const vec3_t& other) {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }

        vec3_t& operator*=(const float scalar) {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }

        vec3_t& operator/=(const float scalar) {
            x /= scalar;
            y /= scalar;
            z /= scalar;
            return *this;
        }

        bool operator==(const vec3_t& other) const {
            constexpr float epsilon = std::numeric_limits<float>::epsilon();
            return std::fabs(x - other.x) < epsilon && std::fabs(y - other.y) < epsilon && std::fabs(z - other.z) < epsilon;
        }

        bool operator!=(const vec3_t& other) const {
            return !(*this == other);
        }
    };

    inline vec3_t operator+(const vec3_t& v1, const vec3_t& v2) {
        return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
    }

    inline vec3_t operator*(const vec3_t& v, const float scalar) {
        return {v.x * scalar, v.y * scalar, v.z * scalar};
    }

    inline vec3_t operator*(const float scalar, const vec3_t& v) {
        return {v.x * scalar, v.y * scalar, v.z * scalar};
    }

    struct triangle_t {
        vec3_t a;
        vec3_t b;
        vec3_t c;

        triangle_t(vec3_t a, vec3_t b, vec3_t c) : a(a), b(b), c(c) { }
    };

    inline float dot(const vec3_t& v1, const vec3_t& v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }
    inline vec3_t cross(const vec3_t& v1, const vec3_t& v2) {
        return {v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x};
    }
    inline float magnitude(const vec3_t& v) { return sqrt(dot(v, v)); }
    inline vec3_t normalize(const vec3_t& v) {
        const float mag = magnitude(v);
        return {v.x / mag, v.y / mag, v.z / mag};
    }
}
