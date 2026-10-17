#include <glad/glad.h>

#include "app.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <utility>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "console.h"
#include "physics.h"
#include "picking.h"
#include "scene.h"
#include "shape.h"
#include "destruction.h"

// Mantido porque os modulos de fisica/scene podem consultar esta lista.
std::vector<Shape*> gShapes;

App::App() = default;
App::~App() = default;

bool App::init() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_STENCIL_BITS, 8);

    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    if (!monitor) {
        std::cerr << "Failed to get primary monitor\n";
        glfwTerminate();
        return false;
    }

    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    if (!mode) {
        std::cerr << "Failed to get video mode\n";
        glfwTerminate();
        return false;
    }

    glfwWindowHint(GLFW_REFRESH_RATE, mode->refreshRate);

    // Preserva o modo fullscreen da Engine Test, mas usa a resolucao real.
    _window = glfwCreateWindow(
        mode->width,
        mode->height,
        "Engine Test - Mech Third Person",
        monitor,
        nullptr
    );

    if (!_window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1); // VSync
    glfwSetWindowUserPointer(_window, this);

    // Registrado antes do backend ImGui (que encadeia callbacks existentes).
    glfwSetCursorPosCallback(_window, App::cursorCallback);
    glfwSetFramebufferSizeCallback(
        _window,
        [](GLFWwindow*, int width, int height) {
            glViewport(0, 0, width, height);
        }
    );

    _cameraActive = true;
    _firstMouse = true;
    glfwSetInputMode(_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        glfwDestroyWindow(_window);
        _window = nullptr;
        glfwTerminate();
        return false;
    }

    int fbWidth = 0;
    int fbHeight = 0;
    glfwGetFramebufferSize(_window, &fbWidth, &fbHeight);
    glViewport(0, 0, fbWidth, fbHeight);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_STENCIL_TEST);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(_window, true);
    ImGui_ImplOpenGL3_Init("#version 460");

    _renderer.init();

    auto& pyramids = _scene.pyramidsRef();
    auto& rects = _scene.rectsRef();
    auto& spheres = _scene.spheresRef();

    pyramids.clear();
    rects.clear();
    spheres.clear();
    _scene.selectionRef() = Selection{};

    // ------------------------------------------------------------
    // Objetos originais da Engine Test
    // ------------------------------------------------------------
    Shape pyramid;
    pyramid.type = ShapeType::Pyramid;
    pyramid.id = 3;
    pyramid.scale = glm::vec3(1.0f);
    pyramid.pos = glm::vec3(-3.0f, 0.51f, 0.0f);
    pyramid.mass = 1.0f;
    pyramid.requestedFaceCount = 5;
    pyramid.requestedVertexCount = 5;
    setupPyramidShape(pyramid);
    pyramid.inertia = computeLocalInertiaTensor(pyramid);
    pyramid.invInertia = glm::inverse(pyramid.inertia);
    pyramids.push_back(pyramid);

    Shape rect;
    rect.type = ShapeType::Rect;
    rect.id = 2;
    // Mais a frente para nao nascer diretamente em cima do mech.
    rect.pos = glm::vec3(0.0f, 3.0f, -4.0f);
    rect.scale = glm::vec3(2.0f, 1.0f, 1.0f);
    rect.mass = 3.0f;
    rect.requestedFaceCount = 6;
    rect.requestedVertexCount = 8;
    setupRectShape(rect);
    rect.inertia = computeLocalInertiaTensor(rect);
    rect.invInertia = glm::inverse(rect.inertia);
    rects.push_back(rect);

    Shape sphere;
    sphere.type = ShapeType::Sphere;
    sphere.id = 1;
    sphere.scale = glm::vec3(1.0f);
    sphere.radius = 1.0f;
    sphere.pos = glm::vec3(3.0f, 4.0f, 1.0f);
    sphere.mass = 2.0f;
    setupSphereShape(sphere, sphere.radius);
    sphere.inertia = computeLocalInertiaTensor(sphere);
    sphere.invInertia = glm::inverse(sphere.inertia);
    spheres.push_back(sphere);

    // ------------------------------------------------------------
    // Player: provisoriamente, o mech e um paralelepipedo.
    // Seu transform e controlado por Player, nao pela integracao fisica.
    // ------------------------------------------------------------
    _player.position = glm::vec3(0.0f, 1.0f, 0.0f);
    _player.velocity = glm::vec3(0.0f);
    _player.yaw = 0.0f;

    Shape mech;
    mech.type = ShapeType::Rect;
    mech.id = 100;
    mech.pos = _player.position;
    mech.scale = glm::vec3(1.5f, 2.0f, 1.0f);
    mech.rot = glm::angleAxis(_player.yaw, glm::vec3(0.0f, 1.0f, 0.0f));
    mech.mass = 10.0f;
    mech.isStatic = true;
    mech.useGravity = false;
    mech.vel = glm::vec3(0.0f);
    mech.angularVel = glm::vec3(0.0f);
    mech.requestedFaceCount = 6;
    mech.requestedVertexCount = 8;
    setupRectShape(mech);

    _playerShapeIndex = rects.size();
    rects.push_back(mech);

    // ------------------------------------------------------------
    // Camera terceira pessoa: olhar para o torso, atras do mech.
    // ------------------------------------------------------------
    _camera.yaw = -90.0f;
    _camera.pitch = -12.0f;
    _camera.onMouse(0.0f, 0.0f); // Recalcula front a partir de yaw/pitch.

    _thirdPersonCamera.distance = 8.0f;
    _thirdPersonCamera.height = 0.75f;
    _thirdPersonCamera.update(_camera, _player.position, 1.0f);

    _renderer.setPrevCameraPos(_camera.pos);
    _lastFrame = static_cast<float>(glfwGetTime());
    _deltaTime = 0.0f;
    _dragging = false;
    _rightPrev = false;
    _havePrevTarget = false;
    _adminMode = false;
    _leftPrev = false;
    _adminKeyWasDown = false;
    _savedGameCamera = _camera;
    _firePrev = false;
    _beamRemaining = 0.0f;
    _shotMarks.clear();

    return true;
}


// Screen-space ray using the same FOV and aspect ratio as Renderer::draw().
static Ray makeCursorRay(GLFWwindow* window, const Camera& camera) {
    double mouseX = 0.0, mouseY = 0.0;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    int windowWidth = 1, windowHeight = 1;
    int fbWidth = 1, fbHeight = 1;
    glfwGetWindowSize(window, &windowWidth, &windowHeight);
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);

    windowWidth = std::max(windowWidth, 1);
    windowHeight = std::max(windowHeight, 1);
    fbHeight = std::max(fbHeight, 1);

    const float x = static_cast<float>(2.0 * mouseX / windowWidth - 1.0);
    const float y = static_cast<float>(1.0 - 2.0 * mouseY / windowHeight);
    const float aspect = static_cast<float>(fbWidth) / fbHeight;
    const float halfFov = std::tan(glm::radians(45.0f) * 0.5f);

    const glm::vec3 forward = glm::normalize(camera.front);
    const glm::vec3 right = glm::normalize(glm::cross(forward, camera.up));
    const glm::vec3 up = glm::normalize(glm::cross(right, forward));
    const glm::vec3 direction = glm::normalize(
        forward + right * x * aspect * halfFov + up * y * halfFov
    );
    return Ray{camera.pos, direction};
}

static Shape* selectedShape(Scene& scene, const Selection& selection,
                            std::size_t mechIndex) {
    if (selection.index < 0) return nullptr;
    const std::size_t i = static_cast<std::size_t>(selection.index);
    if (selection.type == ShapeType::Pyramid) {
        auto& shapes = scene.pyramidsRef();
        return i < shapes.size() ? &shapes[i] : nullptr;
    }
    if (selection.type == ShapeType::Sphere) {
        auto& shapes = scene.spheresRef();
        return i < shapes.size() ? &shapes[i] : nullptr;
    }
    if (selection.type == ShapeType::Rect) {
        auto& shapes = scene.rectsRef();
        return i < shapes.size() && i != mechIndex ? &shapes[i] : nullptr;
    }
    return nullptr;
}

static void releaseShape(Shape* shape) {
    if (!shape) return;
    shape->isDragging = false;
    shape->vel = glm::vec3(0.0f);
    shape->angularVel = glm::vec3(0.0f);
    shape->isSleeping = false;
    shape->sleepTimer = 0.0f;
    shape->groundedStableTimer = 0.0f;
    shape->geometryDirty = true;
}

void App::setAdminMode(bool enabled) {
    if (_adminMode == enabled) return;

    releaseShape(selectedShape(_scene, _scene.selectionRef(), _playerShapeIndex));
    _dragging = false;
    _leftPrev = false;
    _rightPrev = false;
    _havePrevTarget = false;
    _scene.selectionRef() = Selection{};

    if (enabled) {
        _savedGameCamera = _camera;
        // Match the editor's yaw/pitch to the actual look vector so the first
        // RMB movement does not abruptly snap the third-person camera angle.
        const glm::vec3 look = glm::normalize(_camera.front);
        _camera.yaw = glm::degrees(std::atan2(look.z, look.x));
        _camera.pitch = glm::degrees(std::asin(std::clamp(look.y, -1.0f, 1.0f)));
        _player.velocity = glm::vec3(0.0f);
        _cameraActive = false;
        glfwSetInputMode(_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    } else {
        _camera = _savedGameCamera;
        _cameraActive = true;
        glfwSetInputMode(_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    }

    _adminMode = enabled;
    _beamRemaining = 0.0f; // No shooting effects in editor mode.
    _firePrev = (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    _firstMouse = true;
    _renderer.setPrevCameraPos(_camera.pos);
}

void App::frame() {
    const float now = static_cast<float>(glfwGetTime());
    _deltaTime = std::clamp(now - _lastFrame, 0.0f, 0.05f);
    _lastFrame = now;

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    processInput();
    _beamRemaining = std::max(0.0f, _beamRemaining - _deltaTime);

    if (!_adminMode) {
        _thirdPersonCamera.update(_camera, _player.position, _deltaTime);
    }

    // F1 is always available. With cursor captured, Tab frees it for the button.
    ImGui::SetNextWindowPos(
        ImVec2(ImGui::GetIO().DisplaySize.x - 230.0f, 15.0f),
        ImGuiCond_Always
    );
    ImGui::SetNextWindowBgAlpha(0.85f);
    const ImGuiWindowFlags modeWindowFlags =
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings;

    ImGui::Begin("Modo da Engine", nullptr, modeWindowFlags);
    ImGui::TextUnformatted(_adminMode ? "MODO ADMIN" : "MODO JOGO");
    if (ImGui::Button(_adminMode ? "Voltar ao jogo (F1)" : "Ativar ADMIN (F1)")) {
        setAdminMode(!_adminMode);
    }
    ImGui::End();

    if (_adminMode) {
        drawConsole(_scene);
    }

    // Mini HUD; not interactive.
    ImGui::SetNextWindowPos(ImVec2(15.0f, 15.0f), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.42f);
    const ImGuiWindowFlags hudFlags =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav |
        ImGuiWindowFlags_NoInputs;
    ImGui::Begin("Mech HUD", nullptr, hudFlags);
    if (_adminMode) {
        ImGui::TextUnformatted("ADMIN | Clique esquerdo: selecionar/arrastar");
        ImGui::TextUnformatted("Roda: profundidade | RMB: olhar");
        ImGui::TextUnformatted("WASD: camera | Q/E: subir/descer | F1: jogo");
    } else {
        ImGui::TextUnformatted("JOGO | WASD: mech | Shift: correr");
        ImGui::TextUnformatted("Mouse: camera | Clique esquerdo: atirar / fragmentar");
        ImGui::TextUnformatted("Tab: cursor | F1: admin");
        ImGui::Text("Pos: %.1f, %.1f, %.1f",
                    _player.position.x, _player.position.y, _player.position.z);
    }
    ImGui::End();

    if (_adminMode) {
        Shape* selected = selectedShape(_scene, _scene.selectionRef(), _playerShapeIndex);

        ImGui::SetNextWindowPos(ImVec2(15.0f, 100.0f), ImGuiCond_FirstUseEver);
        ImGui::Begin("Admin - objetos");
        ImGui::TextUnformatted("Arraste um shape ou ajuste coordenadas abaixo.");
        if (selected) {
            ImGui::Separator();
            ImGui::Text("Objeto ID: %d", selected->id);
            ImGui::Text("Physics: %s | Support: %s",
                selected->isSleeping ? "sleeping" : "awake",
                selected->supportIsStable ? "stable face" : "unstable / none");
            if (ImGui::DragFloat3("Posicao (XYZ)", &selected->pos.x, 0.05f)) {
                selected->vel = glm::vec3(0.0f);
                selected->angularVel = glm::vec3(0.0f);
                selected->geometryDirty = true;
                selected->isSleeping = false;
                selected->sleepTimer = 0.0f;
            }
            bool frozen = selected->isStatic;
            if (ImGui::Checkbox("Congelar no lugar", &frozen)) {
                selected->isStatic = frozen;
                selected->useGravity = !frozen;
                selected->vel = glm::vec3(0.0f);
                selected->angularVel = glm::vec3(0.0f);
                selected->isSleeping = false;
                selected->sleepTimer = 0.0f;
                selected->geometryDirty = true;
            }
            if (ImGui::Button("Soltar e ativar fisica")) {
                selected->isStatic = false;
                selected->useGravity = true;
                releaseShape(selected);
                _dragging = false;
            }
            ImGui::SameLine();
            if (ImGui::Button("Desselecionar")) {
                releaseShape(selected);
                _dragging = false;
                _scene.selectionRef() = Selection{};
            }
        } else {
            ImGui::TextUnformatted("Nenhum shape selecionado.");
        }
        ImGui::Separator();
        ImGui::TextUnformatted("Console: add rect / add sphere / add pyramid");
        ImGui::TextUnformatted("Aperte F1 para retornar ao jogo.");
        ImGui::End();

        const bool leftDown =
            glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        const bool mouseOnUi = ImGui::GetIO().WantCaptureMouse;
        if (leftDown && !_leftPrev && !mouseOnUi) {
            const Ray ray = makeCursorRay(_window, _camera);
            _scene.selectionRef() = pickShape(_scene, ray, 100);
            selected = selectedShape(_scene, _scene.selectionRef(), _playerShapeIndex);
            _dragging = (selected != nullptr);
            if (selected) {
                _dragDistance = std::max(0.5f, _scene.selectionRef().distance);
                _dragOffset = selected->pos - (ray.o + ray.d * _dragDistance);
                selected->isDragging = true;
                selected->isSleeping = false;
                selected->sleepTimer = 0.0f;
                selected->vel = glm::vec3(0.0f);
                selected->angularVel = glm::vec3(0.0f);
            }
        }

        if (leftDown && _dragging) {
            selected = selectedShape(_scene, _scene.selectionRef(), _playerShapeIndex);
            if (selected) {
                if (!mouseOnUi) {
                    _dragDistance = std::max(0.5f,
                        _dragDistance - ImGui::GetIO().MouseWheel * 0.7f);
                }
                const Ray ray = makeCursorRay(_window, _camera);
                selected->pos = ray.o + ray.d * _dragDistance + _dragOffset;
                selected->vel = glm::vec3(0.0f);
                selected->angularVel = glm::vec3(0.0f);
                selected->isSleeping = false;
                selected->geometryDirty = true;
            } else {
                _dragging = false;
            }
        }

        if (!leftDown && _leftPrev) {
            if (_dragging) {
                releaseShape(selectedShape(_scene, _scene.selectionRef(), _playerShapeIndex));
            }
            _dragging = false;
            // Keep selection so the inspector can edit the position.
        }
        _leftPrev = leftDown;
    }

    // Keep the kinematic mech at its player-controlled transform. It must not
    // drift during admin edits or simulate as a dynamic body.
    auto syncMech = [&]() {
        auto& rects = _scene.rectsRef();
        if (_playerShapeIndex >= rects.size()) return;
        Shape& mech = rects[_playerShapeIndex];
        // Velocity of a kinematic platform is needed to transfer motion
        // through its contacts. Yaw is also an angular contact velocity.
        const glm::mat3 oldRotation = glm::mat3_cast(mech.rot);
        const float previousYaw = std::atan2(oldRotation[2].x, oldRotation[2].z);
        const float deltaYaw = std::remainder(_player.yaw - previousYaw,
                                              6.2831853071795864769f);
        mech.pos = _player.position;
        mech.rot = glm::angleAxis(_player.yaw, glm::vec3(0.0f, 1.0f, 0.0f));
        mech.vel = _adminMode ? glm::vec3(0.0f) : _player.velocity;
        mech.vel.y = 0.0f;
        mech.angularVel = _adminMode || _deltaTime <= 0.0f
            ? glm::vec3(0.0f)
            : glm::vec3(0.0f, deltaYaw / _deltaTime, 0.0f);
        mech.isStatic = true;
        mech.useGravity = false;
        mech.geometryDirty = true;
    };
    syncMech();
    if (_deltaTime > 0.0f) updatePhysics(_scene, _deltaTime);
    syncMech();

    // Edge-triggered left click, ignored while operating ImGui / admin mode.
    const bool fireDown =
        glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    if (!_adminMode && _cameraActive && fireDown && !_firePrev &&
        !ImGui::GetIO().WantCaptureMouse) {
        fireShot();
    }
    _firePrev = fireDown;

    // Center-screen reticle exists only in player third-person mode.
    if (!_adminMode) {
        const ImVec2 display = ImGui::GetIO().DisplaySize;
        const ImVec2 center(display.x * 0.5f, display.y * 0.5f);
        ImDrawList* overlay = ImGui::GetForegroundDrawList();
        const ImU32 dark = IM_COL32(12, 12, 12, 230);
        const ImU32 red = IM_COL32(255, 60, 60, 245);
        for (int i = 0; i < 4; ++i) {
            const ImVec2 dir = i == 0 ? ImVec2(1, 0) : i == 1 ? ImVec2(-1, 0) :
                               i == 2 ? ImVec2(0, 1) : ImVec2(0, -1);
            const ImVec2 p0(center.x + dir.x * 6.0f, center.y + dir.y * 6.0f);
            const ImVec2 p1(center.x + dir.x * 13.0f, center.y + dir.y * 13.0f);
            overlay->AddLine(p0, p1, dark, 4.0f);
            overlay->AddLine(p0, p1, red, 2.0f);
        }
        overlay->AddCircleFilled(center, 1.8f, red);
    }

    glClearColor(0.10f, 0.10f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    _renderer.draw(_scene, _camera);
    if (!_adminMode) {
        _renderer.drawShots(_scene, _camera, _shotMarks,
                            _beamRemaining > 0.0f ? &_shotBeam : nullptr);
    }
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(_window);
    glfwPollEvents();
    _renderer.setPrevCameraPos(_camera.pos);
}

bool App::shouldClose() const {
    return !_window || glfwWindowShouldClose(_window);
}

void App::shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    if (_window) {
        glfwDestroyWindow(_window);
        _window = nullptr;
    }
    glfwTerminate();
}

void App::cursorCallback(GLFWwindow* window, double xpos, double ypos) {
    App* self = static_cast<App*>(glfwGetWindowUserPointer(window));
    if (!self) return;

    if (self->_firstMouse) {
        self->_renderer.setLastMouse(static_cast<float>(xpos), static_cast<float>(ypos));
        self->_firstMouse = false;
        return;
    }

    const float dx = static_cast<float>(xpos - self->_renderer.getLastX());
    const float dy = static_cast<float>(self->_renderer.getLastY() - ypos);
    self->_renderer.setLastMouse(static_cast<float>(xpos), static_cast<float>(ypos));

    if (self->_adminMode) {
        // RMB: orbit/turn the editor camera while keeping cursor visible.
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS &&
            !self->_dragging && !ImGui::GetIO().WantCaptureMouse) {
            self->_camera.onMouse(dx, dy);
        }
    } else if (self->_cameraActive) {
        self->_camera.onMouse(dx, dy);
    }
}

void App::processInput() {
    if (glfwGetKey(_window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(_window, true);

    const bool adminDown = glfwGetKey(_window, GLFW_KEY_F1) == GLFW_PRESS;
    if (adminDown && !_adminKeyWasDown)
        setAdminMode(!_adminMode);
    _adminKeyWasDown = adminDown;

    if (_adminMode) {
        if (ImGui::GetIO().WantCaptureKeyboard) return;

        const float speed = (glfwGetKey(_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
            ? 14.0f : 6.0f;
        glm::vec3 direction(0.0f);
        const glm::vec3 forward = glm::normalize(_camera.front);
        const glm::vec3 right = glm::normalize(glm::cross(forward, _camera.up));

        if (glfwGetKey(_window, GLFW_KEY_W) == GLFW_PRESS ||
            glfwGetKey(_window, GLFW_KEY_UP) == GLFW_PRESS) direction += forward;
        if (glfwGetKey(_window, GLFW_KEY_S) == GLFW_PRESS ||
            glfwGetKey(_window, GLFW_KEY_DOWN) == GLFW_PRESS) direction -= forward;
        if (glfwGetKey(_window, GLFW_KEY_D) == GLFW_PRESS ||
            glfwGetKey(_window, GLFW_KEY_RIGHT) == GLFW_PRESS) direction += right;
        if (glfwGetKey(_window, GLFW_KEY_A) == GLFW_PRESS ||
            glfwGetKey(_window, GLFW_KEY_LEFT) == GLFW_PRESS) direction -= right;
        if (glfwGetKey(_window, GLFW_KEY_E) == GLFW_PRESS)
            direction.y += 1.0f;
        if (glfwGetKey(_window, GLFW_KEY_Q) == GLFW_PRESS)
            direction.y -= 1.0f;

        if (glm::dot(direction, direction) > 0.0001f)
            _camera.pos += glm::normalize(direction) * speed * _deltaTime;
        return;
    }

    static bool tabWasDown = false;
    const bool tabDown = glfwGetKey(_window, GLFW_KEY_TAB) == GLFW_PRESS;
    if (tabDown && !tabWasDown) {
        _cameraActive = !_cameraActive;
        glfwSetInputMode(_window, GLFW_CURSOR,
            _cameraActive ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
        _firstMouse = true;
    }
    tabWasDown = tabDown;

    if (!_cameraActive || ImGui::GetIO().WantCaptureKeyboard) {
        _player.velocity = glm::vec3(0.0f);
        return;
    }

    PlayerInput input;
    if (glfwGetKey(_window, GLFW_KEY_W) == GLFW_PRESS) input.forward += 1.0f;
    if (glfwGetKey(_window, GLFW_KEY_S) == GLFW_PRESS) input.forward -= 1.0f;
    if (glfwGetKey(_window, GLFW_KEY_D) == GLFW_PRESS) input.right += 1.0f;
    if (glfwGetKey(_window, GLFW_KEY_A) == GLFW_PRESS) input.right -= 1.0f;
    input.running =
        glfwGetKey(_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(_window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    _player.update(input, _camera, _deltaTime);
}


void App::fireShot() {
    // The reticle represents the exact center ray of Renderer::draw().
    constexpr float maxRange = 90.0f;
    constexpr float beamDuration = 0.14f;
    const glm::vec3 aimDirection = glm::normalize(_camera.front);
    const Ray cameraRay{_camera.pos, aimDirection};
    const RayHit aimedAt = raycastShapes(_scene, cameraRay, maxRange, 100);
    const glm::vec3 aimPoint = aimedAt.hit()
        ? aimedAt.point : cameraRay.o + cameraRay.d * maxRange;

    // Offset barrel to the shoulder's right; no accidental self-hit on mech.
    const glm::vec3 horizontal(aimDirection.x, 0.0f, aimDirection.z);
    const glm::vec3 horizontalForward = glm::dot(horizontal, horizontal) > 1.0e-6f
        ? glm::normalize(horizontal) : glm::vec3(0.0f, 0.0f, -1.0f);
    const glm::vec3 right = glm::normalize(glm::cross(horizontalForward, _camera.up));
    const glm::vec3 muzzle = _player.position + glm::vec3(0.0f, 0.35f, 0.0f)
        + right * 1.0f + horizontalForward * 0.55f;
    const glm::vec3 offset = aimPoint - muzzle;
    if (glm::dot(offset, offset) < 0.0001f) return;

    const float travel = glm::length(offset);
    const Ray shotRay{muzzle, offset / travel};
    const RayHit hit = raycastShapes(_scene, shotRay,
                                     std::min(maxRange, travel + 0.02f), 100);
    _shotBeam = {muzzle, hit.hit() ? hit.point : aimPoint};
    _beamRemaining = beamDuration;

    if (!hit.hit()) return;
    std::vector<Shape>* shapes = nullptr;
    if (hit.selection.type == ShapeType::Rect) shapes = &_scene.rectsRef();
    if (hit.selection.type == ShapeType::Pyramid) shapes = &_scene.pyramidsRef();
    if (hit.selection.type == ShapeType::Sphere) shapes = &_scene.spheresRef();
    if (!shapes || hit.selection.index < 0 ||
        static_cast<std::size_t>(hit.selection.index) >= shapes->size()) return;

    Shape& target = (*shapes)[static_cast<std::size_t>(hit.selection.index)];
    // Correct triangle winding: the visible face must point towards the shooter.
    const glm::vec3 normal = glm::normalize(
        glm::dot(hit.normal, shotRay.d) > 0.0f ? -hit.normal : hit.normal);
    const glm::vec3 inheritedVelocity = target.vel;
    // Impulse affects the struck body, including objects that had gone to sleep.
    if (!target.isStatic && !target.isDragging) {
        const glm::vec3 impulse = shotRay.d * 3.6f;
        target.vel += impulse * getInvMass(target);
        target.angularVel += glm::cross(hit.point - target.pos, impulse) * 0.32f;
        target.isSleeping = false;
        target.sleepTimer = 0.0f;
        target.isGrounded = false;
        target.supportIsStable = false;
        target.geometryDirty = true;
    }
    // Debris already represents a detached piece. Hitting it pushes it further,
    // but does not recursively explode it into hundreds of smaller bodies.
    if (target.isShotFragment) return;
    const Destruction::CarveResult cut = Destruction::carve(target, hit.point, normal);
    if (!cut.changed()) return;
    const glm::quat sourceRotation = target.rot;
    const glm::vec3 sourcePosition = target.pos;

    // Use the project's real pyramid physics for small four-sided pyramids.
    // Eject debris toward the outer surface. The damaged parent now uses its
    // actual cavity triangles for contacts (not the old convex collider).
    constexpr int piecesPerShot = 8;
    constexpr std::size_t maxLivePieces = 32;
    static int nextFragmentId = 2000000;
    auto& debrisList = _scene.pyramidsRef();
    std::size_t livePieces = 0;
    for (const Shape& s : debrisList) if (s.isShotFragment) ++livePieces;
    while (livePieces + piecesPerShot > maxLivePieces) {
        auto it = std::find_if(debrisList.begin(), debrisList.end(),
            [](const Shape& s) { return s.isShotFragment; });
        if (it == debrisList.end()) break;
        const std::size_t erasedIndex = static_cast<std::size_t>(it - debrisList.begin());
        debrisList.erase(it);
        // Selection is only used in the editor, but keep its index valid.
        Selection& selected = _scene.selectionRef();
        if (selected.type == ShapeType::Pyramid) {
            if (selected.index == static_cast<int>(erasedIndex)) selected = Selection{};
            else if (selected.index > static_cast<int>(erasedIndex)) --selected.index;
        }
        --livePieces;
    }

    const glm::vec3 helper = std::abs(normal.y) < 0.92f
        ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 axisU = glm::normalize(glm::cross(normal, helper));
    const glm::vec3 axisV = glm::normalize(glm::cross(normal, axisU));
    constexpr float goldenAngle = 2.399963229728653f;
    for (int i = 0; i < piecesPerShot; ++i) {
        const float f = static_cast<float>(i + 1) / piecesPerShot;
        const float angle = i * goldenAngle;
        const glm::vec3 radial = std::cos(angle) * axisU + std::sin(angle) * axisV;
        const std::size_t sample = std::min(
            cut.removedLocalCenters.size() - 1,
            static_cast<std::size_t>(f * (cut.removedLocalCenters.size() - 1)));
        const glm::vec3 cutLocal = cut.removedLocalCenters[sample];
        const glm::vec3 removedWorldPoint = sourcePosition + sourceRotation * cutLocal;
        const float depthBelowSurface = std::max(
            0.0f, glm::dot(hit.point - removedWorldPoint, normal));
        Shape piece;
        piece.type = ShapeType::Pyramid;
        piece.isShotFragment = true;
        piece.id = nextFragmentId++;
        piece.scale = glm::vec3(0.13f + 0.045f * (i % 3),
                                0.14f + 0.025f * (i % 4),
                                0.13f + 0.035f * ((i + 1) % 3));
        // Eject the fragment from the channel mouth, preventing initial overlap.
        piece.pos = removedWorldPoint + normal *
            (depthBelowSurface + 0.27f + 0.055f * (i % 3));
        piece.rot = glm::angleAxis(angle, glm::normalize(normal + axisU * 0.2f));
        piece.mass = 0.065f;
        piece.useGravity = true;
        piece.isStatic = false;
        piece.isSleeping = false;
        piece.vel = inheritedVelocity + normal * (4.0f + 2.5f * f) +
                    radial * (1.8f + 2.0f * f) + shotRay.d * 0.6f;
        piece.angularVel = axisU * (5.0f + i % 4) +
                           axisV * (3.0f + (i * 3) % 5);
        piece.requestedFaceCount = 5;
        piece.requestedVertexCount = 5;
        setupPyramidShape(piece);
        piece.inertia = computeLocalInertiaTensor(piece);
        piece.invInertia = glm::inverse(piece.inertia);
        piece.geometryDirty = true;
        debrisList.push_back(std::move(piece));
    }
}
