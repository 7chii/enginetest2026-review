
#include "player.h"

#include <glm/gtc/constants.hpp>
#include <glm/common.hpp>

#include <cmath>

void Player::update(
    const PlayerInput& input,
    const Camera& camera,
    float dt
) {
    glm::vec3 forward = camera.front;
    forward.y = 0.0f;

    if (glm::length(forward) < 0.0001f)
        forward = glm::vec3(0, 0, -1);

    forward = glm::normalize(forward);

    glm::vec3 right = glm::normalize(
        glm::cross(forward, glm::vec3(0, 1, 0))
    );

    glm::vec3 direction =
        forward * input.forward +
        right * input.right;

    float length = glm::length(direction);

    if (length > 1.0f)
        direction /= length;

    float speed = input.running
        ? runSpeed
        : walkSpeed;

    glm::vec3 targetVelocity = direction * speed;

    float blend = 1.0f - std::exp(-acceleration * dt);

    velocity = glm::mix(
        velocity,
        targetVelocity,
        blend
    );

    position += velocity * dt;

    // Rotacionar o mech na direcao do movimento
    if (length > 0.001f) {
        float targetYaw = std::atan2(
            direction.x,
            direction.z
        );

        // Menor diferenca angular
        float angleDiff = std::remainder(
            targetYaw - yaw,
            glm::two_pi<float>()
        );

        float turnBlend = 1.0f -
            std::exp(-turnSpeed * dt);

        yaw += angleDiff * turnBlend;
    }
}
