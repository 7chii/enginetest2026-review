#include "picking.h"
#include "camera.h"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cfloat>
#include <glm/gtc/quaternion.hpp>

// Robust slab intersection; works with rays parallel to an axis.
static bool rayAABB(const Ray& ray, const glm::vec3& mn,
                    const glm::vec3& mx, float& tHit) {
    float tMin = 0.0f;
    float tMax = FLT_MAX;
    for (int axis = 0; axis < 3; ++axis) {
        const float o = ray.o[axis];
        const float d = ray.d[axis];
        if (std::abs(d) < 1e-8f) {
            if (o < mn[axis] || o > mx[axis]) return false;
            continue;
        }
        float t1 = (mn[axis] - o) / d;
        float t2 = (mx[axis] - o) / d;
        if (t1 > t2) std::swap(t1, t2);
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        if (tMin > tMax) return false;
    }
    tHit = tMin;
    return tHit >= 0.0f;
}

static bool rayShape(const Ray& ray, const Shape& s, float& t) {
    if (s.type == ShapeType::Sphere) {
        const float radius = s.radius *
            std::max({s.scale.x, s.scale.y, s.scale.z});
        const glm::vec3 oc = ray.o - s.pos;
        const float b = glm::dot(oc, ray.d);
        const float c = glm::dot(oc, oc) - radius * radius;
        const float delta = b * b - c;
        if (delta < 0.0f) return false;
        const float sqrtDelta = std::sqrt(delta);
        const float nearT = -b - sqrtDelta;
        const float farT = -b + sqrtDelta;
        if (farT < 0.0f) return false;
        t = nearT >= 0.0f ? nearT : farT;
        return true;
    }

    // OBB: transform the ray into the object's rotated local frame.
    const glm::quat invRot = glm::inverse(s.rot);
    const Ray local {
        invRot * (ray.o - s.pos),
        invRot * ray.d
    };
    const glm::vec3 half = s.scale * 0.5f;
    return rayAABB(local, -half, half, t);
}

Selection pickShape(const Scene& scene, const Ray& ray, int ignoredShapeId) {
    Selection result;
    float nearest = FLT_MAX;

    auto test = [&](const std::vector<Shape>& shapes, ShapeType type) {
        for (int i = 0; i < static_cast<int>(shapes.size()); ++i) {
            const Shape& s = shapes[static_cast<std::size_t>(i)];
            if (s.id == ignoredShapeId) continue;
            float t = 0.0f;
            if (rayShape(ray, s, t) && t < nearest) {
                nearest = t;
                result = {type, i, t};
            }
        }
    };
    test(scene.pyramidsRef(), ShapeType::Pyramid);
    test(scene.rectsRef(), ShapeType::Rect);
    test(scene.spheresRef(), ShapeType::Sphere);
    return result;
}

Selection pickShape(const Scene& scene, const Camera& cam) {
    return pickShape(scene, Ray{cam.pos, glm::normalize(cam.front)});
}
