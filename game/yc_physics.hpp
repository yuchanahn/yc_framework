#pragma once
#include <vector>

#include "yc_math.hpp"

namespace yc_physics
{
    using namespace yc_math;


    struct capsule_t {
        vec3_t start;
        vec3_t end;
        float radius;

        capsule_t(const vec3_t start, const vec3_t end, const float radius) : start(start), end(end), radius(radius) {}
    };

    struct sphere_t {
        vec3_t center;
        float radius;

        sphere_t(const vec3_t center, const float radius) : center(center), radius(radius) { }
    };

    struct box_t {
        vec3_t min;
        vec3_t max;

        box_t(const vec3_t min, const vec3_t max) : min(min), max(max) { }
    };


    struct bvh_node_t {
        box_t bounds;
        bvh_node_t* left;
        bvh_node_t* right;
        std::vector<triangle_t> triangles;
    };

    struct bvh_t {
        bvh_node_t* root;

        bvh_t(std::vector<triangle_t> triangles) {
            root = build_bvh(triangles);
        }

        bool check_collision(const capsule_t& capsule) {
            return check_collision_recursive(root, capsule);
        }

    private:
        bvh_node_t* build_bvh(std::vector<triangle_t> triangles) {
            // code to recursively build the BVH
            // ...
        }
        bool check_collision_recursive(bvh_node_t* node, const capsule_t& capsule) {
            // code to recursively check for collision
            // ...
        }
    };
    
    struct terrain_t {
        std::vector<vec3_t> vertices;
        std::vector<unsigned int> indices;
        bvh_t bvh;

        terrain_t(std::vector<vec3_t> v, std::vector<unsigned int> ids)
            : vertices(std::move(v)), indices(std::move(ids)) {
            std::vector<triangle_t> triangles;
            for (unsigned int i = 0; i < indices.size(); i += 3) {
                vec3_t v1 = vertices[indices[i]];
                vec3_t v2 = vertices[indices[i+1]];
                vec3_t v3 = vertices[indices[i+2]];
                triangles.emplace_back(v1, v2, v3);
            }
            bvh = bvh_t(triangles);
        }

        bool check_collision(const capsule_t& capsule) {
            return bvh.check_collision(capsule);
        }
    };

    inline vec3_t closest_point_on_line_segment_to_box(const vec3_t start, const vec3_t end, const box_t& box) {
        vec3_t closest_point = start;
        if (start.x < box.min.x && end.x >= box.min.x)
            closest_point.x = box.min.x;
        else if (start.x > box.max.x && end.x <= box.max.x)
            closest_point.x = box.max.x;
        if (start.y < box.min.y && end.y >= box.min.y)
            closest_point.y = box.min.y;
        else if (start.y > box.max.y && end.y <= box.max.y)
            closest_point.y = box.max.y;
        if (start.z < box.min.z && end.z >= box.min.z)
            closest_point.z = box.min.z;
        else if (start.z > box.max.z && end.z <= box.max.z)
            closest_point.z = box.max.z;
        return closest_point;
    }

    inline vec3_t closest_point_on_triangle(const triangle_t& triangle, const vec3_t& point) {
        // Code to find the closest point on the triangle to the given point
        const vec3_t ab = triangle.b - triangle.a;
        const vec3_t ac = triangle.c - triangle.a;
        const vec3_t ap = point - triangle.a;
        const float d1 = dot(ab, ap);
        const float d2 = dot(ac, ap);
        if (d1 <= 0.0f && d2 <= 0.0f) {
            return triangle.a;
        }

        const vec3_t bp = point - triangle.b;
        const float d3 = dot(ab, bp);
        const float d4 = dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3) {
            return triangle.b;
        }

        const float vc = d1*d4 - d3*d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
            const float v = d1 / (d1 - d3);
            return triangle.a + v * ab;
        }

        const vec3_t cp = point - triangle.c;
        const float d5 = dot(ab, cp);
        const float d6 = dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6) {
            return triangle.c;
        }

        const float vb = d5*d2 - d1*d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
            const float w = d2 / (d2 - d6);
            return triangle.a + w * ac;
        }

        const float va = d3*d6 - d5*d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
            const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return triangle.b + w * (triangle.c - triangle.b);
        }

        const float denom = 1.0f / (va + vb + vc);
        const float v = vb * denom;
        const float w = vc * denom;
        return triangle.a + ab * v + ac * w;
    }

    inline vec3_t closest_point_on_line_segment_to_point(const vec3_t& start, const vec3_t& end, const vec3_t& point) {
        const vec3_t line_segment = end - start;
        float t = dot(point - start, line_segment) / dot(line_segment, line_segment);
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        return start + line_segment * t;
    }

    inline vec3_t closest_point_on_line_segment(const vec3_t& start, const vec3_t& end, const vec3_t& point) {
        const vec3_t line_segment = end - start;
        if (const float t = dot(point - start, line_segment) / dot(line_segment, line_segment); t < 0) {
            return start;
        }
        else {
            if (t > 1) {
                return end;
            }
            return start + t * line_segment;
        }
    }
    
    inline float distance_point_to_capsule(const vec3_t& point, const capsule_t& capsule) {
        const vec3_t closest_point = closest_point_on_line_segment_to_point(capsule.start, capsule.end, point);
        return magnitude(point - closest_point) - capsule.radius;
    }

    inline vec3_t closest_point_on_box(const box_t& box, const vec3_t& point) {
        vec3_t closest_point = point;
        if (point.x < box.min.x)
            closest_point.x = box.min.x;
        else if (point.x > box.max.x)
            closest_point.x = box.max.x;
        if (point.y < box.min.y)
            closest_point.y = box.min.y;
        else if (point.y > box.max.y)
            closest_point.y = box.max.y;
        if (point.z < box.min.z)
            closest_point.z = box.min.z;
        else if (point.z > box.max.z)
            closest_point.z = box.max.z;
        return closest_point;
    }

    inline float distance_between_line_segments(const vec3_t& start1, const vec3_t& end1, const vec3_t& start2, const vec3_t& end2) {
        const vec3_t u = end1 - start1;
        const vec3_t v = end2 - start2;
        const vec3_t w = start1 - start2;
        const float a = dot(u, u);
        const float b = dot(u, v);
        const float c = dot(v, v);
        const float d = dot(u, w);
        const float e = dot(v, w);
        const float D = a * c - b * b;
        float s_n, s_d = D;
        float t_n, t_d = D;

        if (D < std::numeric_limits<float>::epsilon()) {
            s_n = 0.0f;
            s_d = 1.0f;
            t_n = e;
            t_d = c;
        } else {
            s_n = (b * e - c * d);
            t_n = (a * e - b * d);
            if (s_n < 0.0f) {
                s_n = 0.0f;
                t_n = e;
                t_d = c;
            } else if (s_n > s_d) {
                s_n = s_d;
                t_n = e + b;
                t_d = c;
            }
        }

        if (t_n >= 0.0f) {
            t_n = 0.0f;
            if (-d < 0.0f)
                s_n = 0.0f;
            else if (-d > a)
                s_n = s_d;
            else {
                s_n = -d;
                s_d = a;
            }
        } else if (t_n > t_d) {
            t_n = t_d;
            if ((-d + b) < 0.0f)
                s_n = 0;
            else if ((-d + b) > a)
                s_n = s_d;
            else {
                s_n = (-d + b);
                s_d = a;
            }
        }

        const float sc = (std::abs(s_n) < std::numeric_limits<float>::epsilon() ? 0.0f : s_n / s_d);
        const float tc = (std::abs(t_n) < std::numeric_limits<float>::epsilon() ? 0.0f : t_n / t_d);

        const vec3_t d_p = w + (sc * u) - (tc * v);
        return magnitude(d_p);
    }

    inline bool check_collision(const capsule_t& capsule, const box_t& box) {
        const vec3_t closest_point = closest_point_on_line_segment_to_box(capsule.start, capsule.end, box);
        if (closest_point.x < box.min.x || closest_point.x > box.max.x ||
            closest_point.y < box.min.y || closest_point.y > box.max.y ||
            closest_point.z < box.min.z || closest_point.z > box.max.z) {
            return false;
            }
        const float distance = distance_point_to_capsule(closest_point, capsule);
        return distance < capsule.radius;
    }
    inline bool check_collision(const sphere_t& sphere, const box_t& box) {
        const vec3_t closest_point = closest_point_on_box(box, sphere.center);
        const float distance = magnitude(closest_point - sphere.center);
        return distance < sphere.radius;
    }
    inline bool check_collision(const sphere_t& sphere1, const sphere_t& sphere2) {
        const float distance = magnitude(sphere1.center - sphere2.center);
        return distance < sphere1.radius + sphere2.radius;
    }
    inline bool check_collision(const sphere_t& sphere, const terrain_t& terrain) {
        auto closest_triangle = terrain.bvh.getClosestTriangle(sphere.center);
        vec3_t closest_point = closest_point_on_triangle(closest_triangle, sphere.center);
        float distance = magnitude(closest_point - sphere.center);
        return distance < sphere.radius;
    }
    inline bool check_collision(const capsule_t& capsule, const sphere_t& sphere) {
        const vec3_t closest_point = closest_point_on_line_segment_to_point(capsule.start, capsule.end, sphere.center);
        const float distance = magnitude(closest_point - sphere.center);
        return distance < sphere.radius + capsule.radius;
    }
    inline bool check_collision(const capsule_t& capsule1, const capsule_t& capsule2) {
        const float distance = distance_between_line_segments(capsule1.start, capsule1.end, capsule2.start, capsule2.end);
        return distance < capsule1.radius + capsule2.radius;
    }
    inline bool check_collision(const capsule_t& capsule, const terrain_t& terrain) {
        auto closest_triangle = terrain.bvh.getClosestTriangle(capsule.start, capsule.end);
        vec3_t closest_point = closest_point_on_triangle(closest_triangle, capsule.start, capsule.end);
        float distance = magnitude(closest_point - closest_point_on_line_segment(capsule.start, capsule.end, closest_point));
        return distance < capsule.radius;
    }

    struct physics_world_t { };
}
