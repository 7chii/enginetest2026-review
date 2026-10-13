#pragma once
#include <glm/glm.hpp>
#include <vector>

#include "scene.h"
#include "camera.h"

// Marks are stored relative to their target so that they follow moving shapes.
struct ShotMark {
    ShapeType type = ShapeType::None;
    int shapeIndex = -1;
    int shapeId = -1;
    glm::vec3 localPoint{0.0f};
    glm::vec3 localNormal{0.0f, 1.0f, 0.0f};
};

struct ShotBeam {
    glm::vec3 start{0.0f};
    glm::vec3 end{0.0f};
};

class Renderer {
public:
    Renderer();
    ~Renderer();

    void init();
    void draw(const Scene& scene, const Camera& cam);
    void drawShots(const Scene& scene, const Camera& cam,
                   const std::vector<ShotMark>& marks, const ShotBeam* beam);

    // helpers used da app for mouse / camera velocity stuffz
    void setPrevCameraPos(const glm::vec3& p);
    glm::vec3 getPrevCameraPos() const;

    void setLastMouse(float x, float y);
    float getLastX() const;
    float getLastY() const;

private:
    struct Impl;
    Impl* _p;
};
