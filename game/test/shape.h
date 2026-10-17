#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>
#include <glm/gtc/quaternion.hpp>
#include "geometry.h"

enum class ShapeType {
    None,
    Pyramid,
    Rect,
    Sphere,
    Capsule,
    ConvexHull
};




// Each hit subtracts a tessellated hemispherical cavity from the original mesh.
struct CavityCut {
    glm::vec3 point{0.0f};
    glm::vec3 normal{0.0f,1.0f,0.0f};
    float radius=0.33f;
    float depth=0.33f;
    float outside=0.08f;
};
struct MeshTriangleBounds {
    glm::vec3 min{0.0f};
    glm::vec3 max{0.0f};
};
struct MeshBVHNode {
    glm::vec3 min{0.0f};
    glm::vec3 max{0.0f};
    int left=-1,right=-1;
    int first=0,count=0; // leaf range in triangleOrder
};
struct MeshDamage {
    std::vector<CavityCut> cuts;
    std::vector<glm::vec3> triangles; // unscaled local-space triangle soup
    std::vector<MeshTriangleBounds> triangleBounds;
    std::vector<int> triangleOrder;
    std::vector<MeshBVHNode> bvh;
    std::uint32_t revision=0;
    bool active() const { return !cuts.empty(); }
};

struct Shape {

    ShapeType type = ShapeType::None;
    MeshDamage damage;
    bool isShotFragment = false;
    int id = -1;
    bool geometryDirty = true;

   
    GeometryData geometry;

    int requestedVertexCount = 0;
    int requestedFaceCount = 0;


    std::array<glm::vec3, MAX_SHAPE_VERTS> worldVertices;
    std::array<Face, MAX_SHAPE_FACES> worldFaces;


    
    glm::vec3 lastGroundedPos = glm::vec3(0.0f);
    glm::quat lastGroundedRot = glm::quat(1, 0, 0, 0);
    float groundedStableTimer = 0.0f;

    float radius = 0.0f;      // sphere/capsule
    float halfHeight = 0.0f;  // capsule

    // transf
    glm::vec3 pos = glm::vec3(0.0f);
    glm::vec3 scale = glm::vec3(1.0f);
    glm::quat rot = glm::quat(1, 0, 0, 0);

    // mov lin
    glm::vec3 vel = glm::vec3(0.0f);
    float mass = 1.0f;
    float invMass = 1.0f;

    // mov rot
    glm::vec3 angularVel = glm::vec3(0.0f);
    glm::mat3 inertia = glm::mat3(1.0f);
    glm::mat3 invInertia = glm::mat3(1.0f);

    bool isStatic = false;
    bool isDragging = false;

    bool useGravity = true;
    bool isGrounded = false;
    bool supportIsStable = false; // stable face support: ground OR resting on another body

    bool isSleeping = false;
    float sleepTimer = 0.0f;
};

// massa e inercia
inline float getInvMass(const Shape& s) {
    if (s.isStatic || s.isDragging) return 0.0f;
    return (s.mass > 0.0f) ? 1.0f / s.mass : 0.0f;
}

struct Contact {
    Shape* a;
    Shape* b;
    glm::vec3 point;
    glm::vec3 normal;
    float penetration;
};




struct Selection {
    ShapeType type = ShapeType::None;
    int index = -1;
    float distance = 0.0f;
};

struct Ray {
    glm::vec3 o; // origin
    glm::vec3 d; // direction 
};
