#pragma once

#include <cstddef>
#include <glm/glm.hpp>

struct GLFWwindow;

#include "scene.h"
#include "camera.h"
#include "renderer.h"
#include "player/player.h"
#include "camera/third_person_camera.h"

class App {
public:
    App();
    ~App();

    bool init();
    void frame();
    bool shouldClose() const;
    void shutdown();

private:
    GLFWwindow* _window = nullptr;

    // Subsistemas originais
    Renderer _renderer;
    Scene _scene;
    Camera _camera;

    // Terceira pessoa
    Player _player;
    ThirdPersonCamera _thirdPersonCamera;
    std::size_t _playerShapeIndex = 0;

    // Timing
    float _lastFrame = 0.0f;
    float _deltaTime = 0.0f;

    // Ferramenta de arrastar/arremessar objetos
    bool _dragging = false;
    float _dragDistance = 0.0f;
    glm::vec3 _prevTargetPos{0.0f};
    glm::vec3 _lastTargetPos{0.0f};
    float _lastTargetDt = 0.0f;
    bool _havePrevTarget = false;
    bool _rightPrev = false;

    // Entrada e cursor
    bool _cameraActive = true;
    bool _firstMouse = true;

    // Modo de edicao: camera livre, mouse de picking e painel de shapes.
    bool _adminMode = false;
    bool _adminKeyWasDown = false;
    bool _leftPrev = false;
    Camera _savedGameCamera;
    glm::vec3 _dragOffset{0.0f};

    void setAdminMode(bool enabled);
    void processInput();
    static void cursorCallback(GLFWwindow* window, double xpos, double ypos);
};
