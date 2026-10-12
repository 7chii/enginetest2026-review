
#include "third_person_camera.h"

#include <glm/glm.hpp>
#include <glm/common.hpp>

#include <cmath>

void ThirdPersonCamera::update(
    Camera& camera,
    const glm::vec3& playerPosition,
    float dt
) {
    glm::vec3 target =
        playerPosition + glm::vec3(0, height, 0);

    // Direcao definida pelo mouse
    glm::vec3 direction = glm::normalize(
        camera.front
    );

    glm::vec3 desiredPosition =
        target - direction * distance;

    float blend = 1.0f -
        std::exp(-smoothness * dt);

    camera.pos = glm::mix(
        camera.pos,
        desiredPosition,
        blend
    );

    // Olhar sempre para o mech
    glm::vec3 lookDirection =
        target - camera.pos;

    if (glm::length(lookDirection) > 0.0001f)
        camera.front = glm::normalize(lookDirection);
}
