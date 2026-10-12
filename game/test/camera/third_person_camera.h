
#pragma once

#include "camera.h"

#include <glm/glm.hpp>

class ThirdPersonCamera {
public:
    float distance = 8.0f;
    float height = 2.5f;
    float smoothness = 12.0f;

    void update(
        Camera& camera,
        const glm::vec3& playerPosition,
        float dt
    );
};
