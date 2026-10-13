#pragma once
#include "scene.h"
#include "camera.h"

// Accurate surface hit for shots, including surface normal and world-space point.
struct RayHit {
    Selection selection{};
    glm::vec3 point{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    bool hit() const { return selection.index >= 0; }
};

RayHit raycastShapes(const Scene& scene, const Ray& ray,
                     float maxDistance, int ignoredShapeId = -1);

// Legacy camera-center ray, also used outside the editor.
Selection pickShape(const Scene& scene, const Camera& cam);

// Screen ray used by admin cursor picking. ignoredShapeId prevents picking the mech.
Selection pickShape(const Scene& scene, const Ray& ray, int ignoredShapeId = -1);
