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


// Precise raycast used for shooting. Ray direction must be normalized.
// Pyramid triangles match the vertices used by Renderer::init().
namespace {
struct SurfaceHit {
    float distance = 0.0f;
    glm::vec3 normal{0.0f};
};

bool rayBoxSurface(const Ray& ray, const Shape& shape, SurfaceHit& out) {
    const glm::quat inverseRotation = glm::inverse(shape.rot);
    const glm::vec3 origin = inverseRotation * (ray.o - shape.pos);
    const glm::vec3 direction = inverseRotation * ray.d;
    const glm::vec3 half = glm::abs(shape.scale) * 0.5f;
    float enter = -FLT_MAX, leave = FLT_MAX;
    glm::vec3 enterNormal(0.0f), leaveNormal(0.0f);

    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(direction[axis]) < 1.0e-8f) {
            if (origin[axis] < -half[axis] || origin[axis] > half[axis])
                return false;
            continue;
        }
        float t0 = (-half[axis] - origin[axis]) / direction[axis];
        float t1 = ( half[axis] - origin[axis]) / direction[axis];
        glm::vec3 n0(0.0f), n1(0.0f);
        n0[axis] = -1.0f;
        n1[axis] = 1.0f;
        if (t0 > t1) {
            std::swap(t0, t1);
            std::swap(n0, n1);
        }
        if (t0 > enter) { enter = t0; enterNormal = n0; }
        if (t1 < leave) { leave = t1; leaveNormal = n1; }
        if (enter > leave) return false;
    }
    if (leave < 0.0f) return false;
    const bool fromOutside = enter >= 0.0f;
    out.distance = fromOutside ? enter : leave;
    out.normal = glm::normalize(shape.rot * (fromOutside ? enterNormal : leaveNormal));
    return true;
}

bool raySphereSurface(const Ray& ray, const Shape& shape, SurfaceHit& out) {
    const glm::quat inverseRotation = glm::inverse(shape.rot);
    const glm::vec3 radii = glm::max(glm::abs(shape.scale), glm::vec3(0.0001f));
    // Renderer scales a unit-radius sphere by Shape::scale.
    const glm::vec3 origin = (inverseRotation * (ray.o - shape.pos)) / radii;
    const glm::vec3 direction = (inverseRotation * ray.d) / radii;
    const float a = glm::dot(direction, direction);
    const float b = glm::dot(origin, direction);
    const float c = glm::dot(origin, origin) - 1.0f;
    const float discriminant = b * b - a * c;
    if (discriminant < 0.0f || a < 1.0e-12f) return false;
    const float root = std::sqrt(discriminant);
    const float near = (-b - root) / a;
    const float far = (-b + root) / a;
    if (far < 0.0f) return false;
    out.distance = near >= 0.0f ? near : far;
    const glm::vec3 hit = origin + out.distance * direction;
    out.normal = glm::normalize(shape.rot * (hit / radii));
    return true;
}

bool rayTriangle(const Ray& ray, const glm::vec3& a,
                 const glm::vec3& b, const glm::vec3& c,
                 SurfaceHit& out) {
    const glm::vec3 edge1 = b - a, edge2 = c - a;
    const glm::vec3 p = glm::cross(ray.d, edge2);
    const float det = glm::dot(edge1, p);
    if (std::abs(det) < 1.0e-8f) return false;
    const float invDet = 1.0f / det;
    const glm::vec3 t = ray.o - a;
    const float u = glm::dot(t, p) * invDet;
    if (u < 0.0f || u > 1.0f) return false;
    const glm::vec3 q = glm::cross(t, edge1);
    const float v = glm::dot(ray.d, q) * invDet;
    if (v < 0.0f || u + v > 1.0f) return false;
    const float distance = glm::dot(edge2, q) * invDet;
    if (distance < 0.0f) return false;
    out.distance = distance;
    out.normal = glm::normalize(glm::cross(edge1, edge2));
    return true;
}

bool rayPyramidSurface(const Ray& ray, const Shape& shape, SurfaceHit& out) {
    static const glm::vec3 vertices[5] = {
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},
        {0.5f, -0.5f, 0.5f}, {-0.5f, -0.5f, 0.5f},
        {0.0f, 0.5f, 0.0f}
    };
    static const int triangles[6][3] = {
        {4, 3, 2}, {4, 2, 1}, {4, 1, 0}, {4, 0, 3},
        {0, 1, 2}, {2, 3, 0}
    };
    const auto world = [&](int i) {
        return shape.pos + shape.rot * (vertices[i] * shape.scale);
    };
    float nearest = FLT_MAX;
    bool found = false;
    for (const auto& triangle : triangles) {
        SurfaceHit candidate;
        if (rayTriangle(ray, world(triangle[0]), world(triangle[1]),
                        world(triangle[2]), candidate) &&
            candidate.distance < nearest) {
            nearest = candidate.distance;
            out = candidate;
            found = true;
        }
    }
    return found;
}
} // namespace

RayHit raycastShapes(const Scene& scene, const Ray& ray,
                     float maxDistance, int ignoredShapeId) {
    RayHit result;
    if (maxDistance <= 0.0f) return result;
    float nearest = maxDistance;

    const auto test = [&](const std::vector<Shape>& shapes, ShapeType type) {
        for (int i = 0; i < static_cast<int>(shapes.size()); ++i) {
            const Shape& shape = shapes[static_cast<std::size_t>(i)];
            if (shape.id == ignoredShapeId) continue;
            SurfaceHit candidate;
            bool intersects = false;
            // Raycast against the carved mesh; don't hit invisible original
            // triangles in its cavity. Mesh vertices are in rotated local units.
            if (shape.damage.active()) {
                const glm::quat inverseRotation = glm::inverse(shape.rot);
                const Ray local{inverseRotation * (ray.o - shape.pos),
                                inverseRotation * ray.d};
                float closestMesh = nearest;
                // BVH ray traversal: avoid intersecting every damaged triangle.
                std::vector<int> triangleCandidates;
                if(!shape.damage.bvh.empty()) {
                    std::vector<int> stack{0};
                    while(!stack.empty()) {
                        const int index=stack.back();stack.pop_back();
                        const auto& node=shape.damage.bvh[index];
                        float entry=0.0f;
                        if(!rayAABB(local,node.min,node.max,entry) || entry>closestMesh)continue;
                        if(node.left>=0) {
                            stack.push_back(node.left);
                            stack.push_back(node.right);
                        }else for(int i=0;i<node.count;++i)
                            triangleCandidates.push_back(shape.damage.triangleOrder[node.first+i]);
                    }
                }else for(int i=0;i<static_cast<int>(shape.damage.triangles.size()/3);++i)
                    triangleCandidates.push_back(i);
                for(int triangle:triangleCandidates) {
                    const std::size_t tri=static_cast<std::size_t>(triangle)*3;
                    SurfaceHit surface;
                    if (rayTriangle(local, shape.damage.triangles[tri],
                                    shape.damage.triangles[tri + 1],
                                    shape.damage.triangles[tri + 2], surface) &&
                        surface.distance < closestMesh &&
                        surface.distance >= 0.001f) {
                        closestMesh = surface.distance;
                        candidate = surface;
                        intersects = true;
                    }
                }
                if (intersects) candidate.normal = glm::normalize(shape.rot * candidate.normal);
            } else switch (type) {
                case ShapeType::Rect:
                    intersects = rayBoxSurface(ray, shape, candidate);
                    break;
                case ShapeType::Pyramid:
                    intersects = rayPyramidSurface(ray, shape, candidate);
                    break;
                case ShapeType::Sphere:
                    intersects = raySphereSurface(ray, shape, candidate);
                    break;
                default: break;
            }
            if (intersects && candidate.distance < nearest &&
                candidate.distance >= 0.0f) {
                nearest = candidate.distance;
                result.selection = {type, i, nearest};
                result.point = ray.o + ray.d * nearest;
                result.normal = candidate.normal;
            }
        }
    };
    test(scene.pyramidsRef(), ShapeType::Pyramid);
    test(scene.rectsRef(), ShapeType::Rect);
    test(scene.spheresRef(), ShapeType::Sphere);
    return result;
}
