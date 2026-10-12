
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "camera.h"

struct PlayerInput {
    float forward = 0.0f;
    float right = 0.0f;
    bool running = false;
};

class Player {
public:
    glm::vec3 position{0.0f, 1.0f, 0.0f};
    glm::vec3 velocity{0.0f};

    float yaw = 0.0f;
    float walkSpeed = 5.0f;
    float runSpeed = 9.0f;
    float acceleration = 12.0f;
    float turnSpeed = 10.0f;

    void update(
        const PlayerInput& input,
        const Camera& camera,
        float dt
    );
};
