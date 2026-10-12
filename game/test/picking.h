#pragma once
#include "scene.h"
#include "camera.h"

// Legacy camera-center ray, also used outside the editor.
Selection pickShape(const Scene& scene, const Camera& cam);

// Screen ray used by admin cursor picking. ignoredShapeId prevents picking the mech.
Selection pickShape(const Scene& scene, const Ray& ray, int ignoredShapeId = -1);
