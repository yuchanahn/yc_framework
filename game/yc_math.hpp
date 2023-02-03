#pragma once

#include <cmath>
#include <optional>

namespace yc_math
{
    /////// Vector3 /////
    struct vec3_t {
        double x;
        double y;
        double z;

        vec3_t operator-(const vec3_t& other) const { return {x - other.x, y - other.y, z - other.z}; }
        vec3_t operator-(const vec3_t&& other) const { return {x - other.x, y - other.y, z - other.z}; }
        vec3_t operator/(const double scalar) const { return {x / scalar, y / scalar, z / scalar}; }
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
        vec3_t& operator*=(const double scalar) {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }
        vec3_t& operator/=(const double scalar) {
            x /= scalar;
            y /= scalar;
            z /= scalar;
            return *this;
        }
        bool operator==(const vec3_t& other) const {
            constexpr double epsilon = std::numeric_limits<double>::epsilon();
            return std::fabs(x - other.x) < epsilon && std::fabs(y - other.y) < epsilon && std::fabs(z - other.z) <
                epsilon;
        }
        bool operator!=(const vec3_t& other) const { return !(*this == other); }
    };

    inline vec3_t operator+(const vec3_t& v1, const vec3_t& v2) { return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z}; }
    inline vec3_t operator*(const vec3_t& v, const double scalar) { return {v.x * scalar, v.y * scalar, v.z * scalar}; }
    inline vec3_t operator*(const double scalar, const vec3_t& v) { return {v.x * scalar, v.y * scalar, v.z * scalar}; }
    inline vec3_t operator*(const vec3_t& v1, const vec3_t& v2) { return {v1.x * v2.x, v1.y * v2.y, v1.z * v2.z}; }
    
    struct triangle_t {
        vec3_t a;
        vec3_t b;
        vec3_t c;

        triangle_t() = default;
        triangle_t(vec3_t a, vec3_t b, vec3_t c) : a(a), b(b), c(c) { }
    };
    
    inline double dot(const vec3_t& v1, const vec3_t& v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }
    inline vec3_t cross(const vec3_t& v1, const vec3_t& v2) { return {v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x}; }
    inline double magnitude(const vec3_t& v) { return sqrt(dot(v, v)); }
    inline vec3_t normalize(const vec3_t& v) { const double mag = magnitude(v); return {v.x / mag, v.y / mag, v.z / mag}; }
    auto max(auto v1, auto v2) { return v1 > v2 ? v1 : v2; }
    auto min(auto v1, auto v2) { return v1 < v2 ? v1 : v2; }
    auto saturate(auto v) { return max(min(v, 1.f), 0.f); }
    inline double distance(const vec3_t& a, const vec3_t& b) { return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2) + pow(b.z - a.z, 2)); }
    
    /////// Quaternion /////
    struct qut_t {
        double w;
        double x;
        double y;
        double z;

        qut_t operator*(const qut_t& other) const {
            return {
                w * other.w - x * other.x - y * other.y - z * other.z,
                w * other.x + x * other.w + y * other.z - z * other.y,
                w * other.y - x * other.z + y * other.w + z * other.x,
                w * other.z + x * other.y - y * other.x + z * other.w
            };
        }
        qut_t operator*(const double scalar) const { return {w * scalar, x * scalar, y * scalar, z * scalar}; }
        vec3_t operator*(const vec3_t& v) const {
            const vec3_t q(x, y, z);
            const auto tt = 2.0 * cross(q, v);
            return v + (w * tt) + cross(q, tt);
        }
        vec3_t operator*(const vec3_t&& v) const {
            const vec3_t q(x, y, z);
            const auto tt = 2.0 * cross(q, v);
            return v + (w * tt) + cross(q, tt);
        }
        qut_t operator/(const double scalar) const { return {w / scalar, x / scalar, y / scalar, z / scalar}; }
        qut_t& operator*=(const qut_t& other) {
            *this = *this * other;
            return *this;
        }
        qut_t& operator*=(const double scalar) {
            w *= scalar;
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }
        qut_t& operator/=(const double scalar) {
            w /= scalar;
            x /= scalar;
            y /= scalar;
            z /= scalar;
            return *this;
        }
        bool operator==(const qut_t& other) const {
            constexpr double epsilon = std::numeric_limits<double>::epsilon();
            return std::fabs(w - other.w) < epsilon && std::fabs(x - other.x) < epsilon && std::fabs(y - other.y) <
                epsilon && std::fabs(z - other.z) < epsilon;
        }
        bool operator!=(const qut_t& other) const { return !(*this == other); }
    };

    inline auto conjugate(const qut_t& q) { return qut_t(q.w, -q.x, -q.y, -q.z); }

    inline vec3_t operator+(const vec3_t& v, const qut_t& q) {
        const qut_t vq(0, v.x, v.y, v.z);
        const auto [w, x, y, z] = q * vq * conjugate(q);
        return vec3_t(x, y, z);
    }
    
    struct line_segment_t {
        vec3_t start, end;
        line_segment_t(const vec3_t& start, const vec3_t& end) : start(start), end(end) {}

        [[nodiscard]] vec3_t closest_point(const vec3_t& point) const {
            const vec3_t line_dir = end - start;
            double t = dot(point - start, line_dir) / dot(line_dir, line_dir);
            t = std::clamp(t, 0.0, 1.0);
            return start + t * line_dir;
        }

        [[nodiscard]] std::optional<vec3_t> closest_point(const line_segment_t& other) const {
            const vec3_t u = end - start;
            const vec3_t v = other.end - other.start;
            const vec3_t w = start - other.start;

            const double a = dot(u,u);
            const double b = dot(u,v);
            if (std::abs(b) < std::numeric_limits<double>::epsilon()) {
                // Lines are parallel
                return {};
            }
            const double c = dot(v,v);
            const double d = dot(u,w);
            const double e = dot(v,w);
            const double det = a*c - b*b;
            double s = b*e - c*d;
            double t = b*d - a*e;
            if (s < 0) {
                s = 0;
            } else if (s > det) {
                s = det;
            }
            if (t < 0) {
                t = 0;
            } else if (t > det) {
                t = det;
            }
            return std::make_optional(start + (s/det)*u);
        }
    };
    
    inline bool col(const line_segment_t& a, const line_segment_t& b, const double radius) {
        if(!a.closest_point(b).has_value()) return false;
        return distance(a.closest_point(b).value(), b.closest_point(a).value()) < radius;
    }
}

namespace std
{
    template <class T>
    void hash_combine(std::size_t& s, const T& v) {  // NOLINT(cert-dcl58-cpp)
        std::hash<T> h;
        s ^= h(v) + 0x9e3779b9 + (s << 6) + (s >> 2);
    }

    template <>
    struct hash<yc_math::vec3_t> {
        std::size_t operator()(const yc_math::vec3_t& v) const noexcept {
            std::size_t hash;
            hash_combine(hash, v.x);
            hash_combine(hash, v.y);
            hash_combine(hash, v.z);
            return hash;
        }
    };

    template <>
    struct hash<yc_math::qut_t> {
        std::size_t operator()(const yc_math::qut_t& q) const noexcept {
            std::size_t hash;
            hash_combine(hash, q.w);
            hash_combine(hash, q.x);
            hash_combine(hash, q.y);
            hash_combine(hash, q.z);
            return hash;
        }
    };
}
