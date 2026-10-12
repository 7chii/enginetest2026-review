#include "physics.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <vector>
#include <cfloat>
#include <array>
#include <algorithm>
#include <limits>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/vector_angle.hpp>

//----------------------------------------------------------
// CONSTANTES
//----------------------------------------------------------

static const glm::vec3 GRAVITY(0.0f, -9.81f, 0.0f);

static const float restitutionGround = 0.2f;
static const float frictionGround = 0.8f;
static const float sleepTimeThreshold = 0.6f;

static const float WAKE_SPEEDANGVEL = 0.000000001f;

static const float WAKE_SPEEDVEL = 0.000000000001f;



//adicionar motor de friccao que possa ser alterado conforme material  add material ao chao

/*
using BoxVerts = std::array<glm::vec3, 8>;
using PyramidVerts = std::array<glm::vec3, 5>;
using PyramidFaces = std::array<PyramidFace, 5>;
*/
//----------------------------------------------------------
// BROADPHASE - AABB
//----------------------------------------------------------

struct AABB
{
    glm::vec3 min;
    glm::vec3 max;
};

static void projectPoints(
    const std::array<glm::vec3, MAX_SHAPE_VERTS>& vertices,
    int vertexCount,
    const glm::vec3& axis,
    float& minProj,
    float& maxProj
) {
    minProj = std::numeric_limits<float>::max();
    maxProj = std::numeric_limits<float>::lowest();

    for (int i = 0; i < vertexCount; ++i) {
        float projection = glm::dot(vertices[i], axis);

        minProj = std::min(minProj, projection);
        maxProj = std::max(maxProj, projection);
    }
}

template<size_t N>
static void projectShape(
    const Shape& s,
    const glm::vec3& axis,
    float& outMin,
    float& outMax)
{
    outMin = FLT_MAX;
    outMax = -FLT_MAX;

    for (int i = 0; i < s.geometry.vertexCount; ++i)
    {
        float proj = glm::dot(s.geometry.vertices[i], axis);

        outMin = std::min(outMin, proj);
        outMax = std::max(outMax, proj);
    }
}
static bool isEffectivelyStable(const Shape& s)
{
    //printf("-- shape : %d , stabletimer : %lf --",s.id, s.groundedStableTimer);

    if (s.isStatic) return true;
    if (s.isSleeping) return true;
    
    //if (s.vel.value < 0.01f && !s.isSleeping) return true;
    //if (s.isGrounded) return true;
    // pequenas folgas: se j começou a estabilizar (groundedStableTimer) ou est quase dormindo
    if (s.groundedStableTimer > 0.5f) return true;
    if (s.sleepTimer > 0.3f) return true;
    return false;
}

// Retorna vertices em world space de uma box (ShapeType::Rect)
static void computeBoxVertices(Shape& s)
{
    glm::mat3 R = glm::mat3_cast(s.rot);
    glm::vec3 he = s.scale * 0.5f;
    s.geometry.vertexCount = 8; // box

    int i = 0;

    for (int x = -1; x <= 1; x += 2)
        for (int y = -1; y <= 1; y += 2)
            for (int z = -1; z <= 1; z += 2)
                s.geometry.vertices[i++] =
                s.pos + R * glm::vec3(x * he.x, y * he.y, z * he.z);
}
// Retorna vertices em world space da piramide

// Retorna vertices em world space da piramide
static void computePyramidVertices(Shape& p)
{
    // Segurança (estilo console determinístico)
    if (p.geometry.vertexCount < 5)
        return;

    glm::mat3 R = glm::mat3_cast(p.rot);
    p.geometry.vertexCount = 5;
    p.geometry.faceCount = 5;

    const float hx = p.scale.x * 0.5f;
    const float hy = p.scale.y * 0.5f;
    const float hz = p.scale.z * 0.5f;

    // Base (y = -hy)
    p.geometry.vertices[0] = p.pos + R * glm::vec3(-hx, -hy, -hz);
    p.geometry.vertices[1] = p.pos + R * glm::vec3(hx, -hy, -hz);
    p.geometry.vertices[2] = p.pos + R * glm::vec3(hx, -hy, hz);
    p.geometry.vertices[3] = p.pos + R * glm::vec3(-hx, -hy, hz);

    // Apex (y = +hy)
    p.geometry.vertices[4] = p.pos + R * glm::vec3(0.0f, hy, 0.0f);
}
static AABB computeAABB(const Shape& s)
{
    if (s.type == ShapeType::Sphere)
    {
        float r = s.scale.x * s.radius;
        return { s.pos - glm::vec3(r), s.pos + glm::vec3(r) };
    }

    glm::vec3 minV(FLT_MAX);
    glm::vec3 maxV(-FLT_MAX);

    // Usa vertices persistentes
    for (int i = 0; i < s.geometry.vertexCount; ++i)
    {
        const glm::vec3& v = s.geometry.vertices[i];

        minV = glm::min(minV, v);
        maxV = glm::max(maxV, v);
    }

    return { minV, maxV };
}

static bool aabbOverlap(const AABB& a, const AABB& b)
{
    if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
    if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
    if (a.max.z < b.min.z || a.min.z > b.max.z) return false;
    return true;
}
static void computePyramidFaces(Shape& p)
{
    if (p.geometry.vertexCount < 5)
        return;

    auto& faces = p.geometry.faces;
    p.geometry.faceCount = 5;

    const auto& v = p.geometry.vertices;

    // 4 laterais
    for (int i = 0; i < 4; ++i)
    {
        const glm::vec3& a = v[i];
        const glm::vec3& b = v[(i + 1) % 4];
        const glm::vec3& c = v[4];

        glm::vec3 n = glm::normalize(glm::cross(b - a, c - a));

        if (glm::dot(n, p.pos - a) > 0.0f)
            n = -n;

        faces[i].n = n;
        faces[i].d = -glm::dot(n, a);
    }

    // base
    glm::vec3 nBase = glm::normalize(glm::cross(v[1] - v[0], v[3] - v[0]));
    if (glm::dot(nBase, p.pos - v[0]) > 0.0f) nBase = -nBase;
    faces[4].n = nBase;
    faces[4].d = -glm::dot(nBase, v[0]);
}

static void updateShapeWorldGeometry(Shape& s)
{
    switch (s.type)
    {
    case ShapeType::Rect:
        computeBoxVertices(s);
        break;

    case ShapeType::Pyramid:
        computePyramidVertices(s);
        computePyramidFaces(s);
        break;

    default:
        break;
    }
}

// Calcula a matriz de inercia local baseada no tipo e escala do objeto.
glm::mat3 computeLocalInertiaTensor(const Shape& s)
{
    float m = s.mass;
    glm::vec3 size = s.scale;

    if (m <= 0.0f)
        return glm::mat3(0.0f);

    if (s.type == ShapeType::Rect)
    {
        float x2 = size.x * size.x;
        float y2 = size.y * size.y;
        float z2 = size.z * size.z;

        return glm::mat3(
            (1.0f / 12.0f) * m * (y2 + z2), 0, 0,
            0, (1.0f / 12.0f) * m * (x2 + z2), 0,
            0, 0, (1.0f / 12.0f) * m * (x2 + y2)
        );
    }

    if (s.type == ShapeType::Sphere)
    {
        float r = s.radius * std::max({ s.scale.x, s.scale.y, s.scale.z });
        float i = (2.0f / 5.0f) * m * r * r;
        return glm::mat3(i);
    }

    // Para piramide (base quadruada), aproxime como caixa (ajuste se necessrio)
    if (s.type == ShapeType::Pyramid)
    {
        float x2 = size.x * size.x;
        float y2 = size.y * size.y;
        float z2 = size.z * size.z;
        float Ixx = (3.0f / 20.0f) * m * (y2 + z2); // aproximaao
        float Iyy = (3.0f / 20.0f) * m * (x2 + z2);
        float Izz = (3.0f / 20.0f) * m * (x2 + y2);
        return glm::mat3(
            Ixx, 0, 0,
            0, Iyy, 0,
            0, 0, Izz
        );
    }

    return glm::mat3(1.0f);
}

// Calcula matriz de inercia inversa no sistema de mundo
static glm::mat3 computeWorldInvInertia(const Shape& s)
{
    if (s.isStatic || s.isDragging )
        return glm::mat3(0.0f);

    glm::mat3 R = glm::mat3_cast(s.rot);
    return R * s.invInertia * glm::transpose(R);
}

//----------------------------------------------------------
// SLEEP THRESHOLDS dinamicos
//----------------------------------------------------------
static float computeSleepLinearThreshold(const Shape& s) {
    // Linear: proporcional ao maior eixo
    float maxScale = std::max({ s.scale.x, s.scale.y, s.scale.z });
    return std::max(0.01f, std::min(0.2f, 0.05f * maxScale));
}
static float computeSleepAngularThreshold(const Shape& s) {
    // Angular: menos sensivel ao tamanho, pode ser constante ou levemente dependente do scale
    float maxScale = std::max({ s.scale.x, s.scale.y, s.scale.z });
    return std::max(0.001f, 0.01f * maxScale);
}

//----------------------------------------------------------
// Funcoes utilitrias
//----------------------------------------------------------

inline void wake(Shape& s) {
   // if (s.isStatic) return;
    s.isSleeping = false;
    s.sleepTimer = 0.0f;
    //s.groundedStableTimer = 0.0f;
}


static void wakeIfThresholdPassed(Shape& s) {
    wake(s);
}

static float sphereRadius(const Shape& s) {
    return s.radius * std::max({ s.scale.x, s.scale.y, s.scale.z });
}

static glm::vec3 getHalfExtents(const Shape& s) {
    return s.scale * 0.5f;
}

static glm::vec3 getLowestPointOnBox(const Shape& s) {
    glm::mat3 R = glm::mat3_cast(s.rot);
    glm::vec3 he = s.scale * 0.5f;

    glm::vec3 lowest = s.pos;
    float minY = FLT_MAX;

    for (int x = -1; x <= 1; x += 2)
        for (int y = -1; y <= 1; y += 2)
            for (int z = -1; z <= 1; z += 2) {
                glm::vec3 local(x * he.x, y * he.y, z * he.z);
                glm::vec3 world = s.pos + R * local;
                if (world.y < minY) {
                    minY = world.y;
                    lowest = world;
                }
            }
    return lowest;
}


static void getPyramidContactPoints(const Shape& s, std::vector<glm::vec3>& out) {
    //std::vector<glm::vec3> verts;
    out.clear();
    for (int i = 0; i < s.geometry.vertexCount; ++i)
    {
        const glm::vec3& v = s.geometry.vertices[i];

        if (v.y <= 0.002f)
            out.push_back(v);
    }
}

//----------------------------------------------------------
// INTEGRACAO
//----------------------------------------------------------
static void integrateGravity(Shape& s, float dt)
{
    if (s.isStatic || s.isDragging || s.isSleeping) return;

    if (s.useGravity)
        s.vel += GRAVITY * dt;

    s.vel *= std::exp(-0.05f * dt);
    s.angularVel *= std::exp(-0.15f * dt);
}

static void integratePosition(Shape& s, float dt)
{
    if (s.isStatic || s.isDragging || s.isSleeping) return;
    s.pos += s.vel * dt;
    s.geometryDirty = true;
}

static void integrateRotation(Shape& s, float dt)
{
    if (s.isStatic || s.isDragging || s.isSleeping) return;

    float w2 = glm::length2(s.angularVel);
    if (w2 < 1e-10f) {
        s.angularVel = glm::vec3(0.0f);
        return;
    }

    float w = std::sqrt(w2);
    s.rot = glm::normalize(glm::angleAxis(w * dt, s.angularVel / w) * s.rot);
    s.geometryDirty = true;
}
/*
const float WAKE_SPEED2 = 0.0025f; // ~5cm/s
    if (glm::length2(s.vel) > WAKE_SPEED2 ||
        glm::length2(s.angularVel) > WAKE_SPEED2)
    {
        wake(s);
    }*/
//----------------------------------------------------------
// 
// 
// APLICACAO TODO IMPULSO
//----------------------------------------------------------
static void applyImpulse(Shape& s, const glm::vec3& J, const glm::vec3& cp) {
    if (s.isStatic || s.isDragging) return;
    if (s.isSleeping) wake(s);
    s.vel += J * getInvMass(s);

    glm::vec3 r = cp - s.pos;
    glm::vec3 torque = glm::cross(r, J);

    glm::mat3 invInertiaWorld = computeWorldInvInertia(s);
    s.angularVel += invInertiaWorld * torque;
}

//----------------------------------------------------------
// CONTATO C CHAO
//----------------------------------------------------------
// Ground contact manifold. Resolve individual vertices, not one centroid:
// impulses distributed over a flat face have no invented torque.
static void solveGroundContact(Shape& s, const glm::vec3& /*cp*/, float /*dt*/)
{
    if (s.isStatic || s.isDragging || s.isSleeping) return;

    constexpr float skin = 0.002f;
    constexpr float groundMargin = 0.013f;
    constexpr float friction = 0.85f;
    const glm::vec3 normal(0.0f, 1.0f, 0.0f);
    const float invM = getInvMass(s);
    const glm::mat3 invI = computeWorldInvInertia(s);

    glm::vec3 contacts[5];
    int contactCount = 0;
    float lowest = FLT_MAX;
    if (s.type == ShapeType::Sphere) {
        lowest = s.pos.y - sphereRadius(s);
        contacts[contactCount++] = s.pos - glm::vec3(0.0f, sphereRadius(s), 0.0f);
    } else {
        for (int i = 0; i < s.geometry.vertexCount; ++i)
            lowest = std::min(lowest, s.geometry.vertices[i].y);
        for (int i = 0; i < s.geometry.vertexCount && contactCount < 5; ++i) {
            const glm::vec3 v = s.geometry.vertices[i];
            if (v.y <= lowest + 0.012f) contacts[contactCount++] = v;
        }
    }
    if (contactCount == 0 || lowest > groundMargin) return;

    const float correction = glm::clamp((skin - lowest) * 0.7f, 0.0f, 0.075f);
    if (correction > 0.0f) {
        s.pos.y += correction;
        s.geometryDirty = true;
    }

    // Several sequential passes for a resting box on four corners.
    for (int pass = 0; pass < 2; ++pass) {
        for (int i = 0; i < contactCount; ++i) {
            const glm::vec3 cp = contacts[i] + glm::vec3(0, correction, 0);
            const glm::vec3 r = cp - s.pos;
            glm::vec3 v = s.vel + glm::cross(s.angularVel, r);
            const float vn = glm::dot(v, normal);
            if (vn >= -1e-6f) continue;
            const glm::vec3 rxn = glm::cross(r, normal);
            const float denom = invM + glm::dot(invI * rxn, rxn);
            if (denom < 1e-9f) continue;
            const float restitution = (vn < -2.0f) ? 0.025f : 0.0f;
            const float jn = -(1.0f + restitution) * vn / denom;
            const glm::vec3 Jn = jn * normal;
            s.vel += invM * Jn;
            s.angularVel += invI * glm::cross(r, Jn);

            v = s.vel + glm::cross(s.angularVel, r);
            glm::vec3 t = v - glm::dot(v, normal) * normal;
            float speed = glm::length(t);
            if (speed < 1e-7f) continue;
            t /= speed;
            const glm::vec3 rxt = glm::cross(r, t);
            const float denomT = invM + glm::dot(invI * rxt, rxt);
            if (denomT < 1e-9f) continue;
            const float jt = std::min(speed / denomT, friction * jn);
            const glm::vec3 Jt = -jt * t;
            s.vel += invM * Jt;
            s.angularVel += invI * glm::cross(r, Jt);
        }
    }
}

static bool isCOMInsideBoxTopFace(const Shape& box, const Shape& obj)
{
    // eixo local da box
    glm::mat3 R = glm::mat3_cast(box.rot);

    glm::vec3 local = glm::transpose(R) * (obj.pos - box.pos);

    glm::vec3 half = box.scale * 0.5f;

    // testa projeao em X e Z da face superior
    return std::abs(local.x) <= half.x &&
        std::abs(local.z) <= half.z;
}

inline float quatAngularDistance(const glm::quat& q1, const glm::quat& q2) {
    float dot = glm::dot(glm::normalize(q1), glm::normalize(q2));
    dot = std::clamp(std::abs(dot), 0.0f, 1.0f); // protections
    return 2.0f * std::acos(dot);
}


// escolhe a face da pirâmide que melhor corresponde à normal de contato (maior dot)
// retorna true se encontrou e preenche centroid e faceNormal (ambos em world)
static bool findPyramidContactFaceCentroid(
    const Shape& pyr,
    const glm::vec3& contactNormal,
    glm::vec3& outCentroid,
    glm::vec3& outFaceNormal)
{
    const auto& verts = pyr.geometry.vertices;

    // esperamos 5 vértices: 0-3 base, 4 apex
    if (verts.size() < 5)
        return false;

    // faces triangulares:
    // base = 2 triângulos
    // 4 faces laterais
    struct Tri { int a, b, c; };

    Tri faces[6] = {
        {0,1,2}, {0,2,3}, // base
        {0,1,4},
        {1,2,4},
        {2,3,4},
        {3,0,4}
    };

    float bestDot = -FLT_MAX;
    int bestIdx = -1;
    glm::vec3 bestNormal(0.0f);

    // contactNormal aponta de suporte -> objeto
    // queremos a face cuja normal aponte aproximadamente contra contactNormal
    glm::vec3 wanted = -glm::normalize(contactNormal);

    for (int i = 0; i < 6; ++i)
    {
        const glm::vec3& v0 = verts[faces[i].a];
        const glm::vec3& v1 = verts[faces[i].b];
        const glm::vec3& v2 = verts[faces[i].c];

        glm::vec3 n = glm::cross(v1 - v0, v2 - v0);
        float len2 = glm::length2(n);
        if (len2 < 1e-8f)
            continue;

        n = glm::normalize(n);

        // garantir normal apontando para fora
        // usamos centro da pirâmide como referência
        if (glm::dot(n, pyr.pos - v0) > 0.0f)
            n = -n;

        float d = glm::dot(n, wanted);

        if (d > bestDot)
        {
            bestDot = d;
            bestIdx = i;
            bestNormal = n;
        }
    }

    if (bestIdx == -1)
        return false;

    const glm::vec3& v0 = verts[faces[bestIdx].a];
    const glm::vec3& v1 = verts[faces[bestIdx].b];
    const glm::vec3& v2 = verts[faces[bestIdx].c];

    outCentroid = (v0 + v1 + v2) / 3.0f;
    outFaceNormal = bestNormal;

    return true;
}
static void handleGroundContact(Shape& s, float dt)
{
    if (s.isStatic || s.isDragging) {
        s.isGrounded = false;
        s.supportIsStable = false;
        return;
    }
    if (s.geometryDirty) {
        updateShapeWorldGeometry(s);
        s.geometryDirty = false;
    }

    const float r = s.type == ShapeType::Sphere ? sphereRadius(s) : 0.0f;
    float minY = s.type == ShapeType::Sphere ? s.pos.y - r : FLT_MAX;
    if (s.type != ShapeType::Sphere)
        for (int i = 0; i < s.geometry.vertexCount; ++i)
            minY = std::min(minY, s.geometry.vertices[i].y);

    s.isGrounded = minY <= 0.013f;
    // Non-spherical objects require an actual face, rather than an unstable
    // single vertex / edge, before they may go to sleep on the floor.
    int nearCount = 0;
    if (s.type != ShapeType::Sphere)
        for (int i = 0; i < s.geometry.vertexCount; ++i)
            if (s.geometry.vertices[i].y <= minY + 0.012f) ++nearCount;
    s.supportIsStable = s.isGrounded &&
        (s.type == ShapeType::Sphere || nearCount >= 3);

    if (s.isGrounded && !s.isSleeping)
        solveGroundContact(s, glm::vec3(0.0f, minY, 0.0f), dt);
}

static bool isPointInsideBoxTopFace(const Shape& box, const glm::vec3& worldPoint)
{
    glm::mat3 R = glm::mat3_cast(box.rot);
    glm::vec3 local = glm::transpose(R) * (worldPoint - box.pos); // point em espaço local da box
    glm::vec3 half = box.scale * 0.5f;

    // checa projeao X/Z dentro dos half-extents da face superior
    // nAo exigimos que y esteja exatamente no topo; apenas que projete sobre a rea da face 1e-6f
    return (std::abs(local.x) <= half.x + 1e-6f && std::abs(local.z) <= half.z);
}



inline bool guessNormalAndPoint(const glm::vec3& a, const glm::vec3& b,
    glm::vec3& outSupportNormal, glm::vec3& outContactPoint, const Shape& s)
{
    // heuristica: um dos argumentos provavelmente e uma normal unitria (~1.0 length)
    // e o outro e um ponto (distancia razovel ao centro do shape).
    float la = glm::length(a);
    float lb = glm::length(b);

    // preferencias: se um tem comprimento ~1, trat-lo como normal
    if (std::abs(la - 1.0f) < 0.25f && lb > 0.001f) {
        outSupportNormal = glm::normalize(a);
        outContactPoint = b;
        return true;
    }
    if (std::abs(lb - 1.0f) < 0.25f && la > 0.001f) {
        outSupportNormal = glm::normalize(b);
        outContactPoint = a;
        return true;
    }

    // fallback: se um dos vetores est perto do centro do shape, assume ponto
    if (glm::length(a - s.pos) < glm::length(b - s.pos)) {
        outContactPoint = a;
        outSupportNormal = glm::normalize(b);
    }
    else {
        outContactPoint = b;
        outSupportNormal = glm::normalize(a);
    }
    return true;
}

inline void snapBoxToSurfaceA(Shape& s,
    const glm::vec3& argA,
    const glm::vec3& argB)
{
    if (s.isStatic) return;

    // detecta ordem (suporte normal vs contacto)
    glm::vec3 supportNormal, contactPoint;
    guessNormalAndPoint(argA, argB, supportNormal, contactPoint, s);

    // se normal invlida, aborta
    if (glm::length2(supportNormal) < 1e-8f) return;
    supportNormal = glm::normalize(supportNormal);

    // obtem normais das faces em world
    glm::mat3 R = glm::mat3_cast(s.rot);
    std::array<glm::vec3, 6> faceNormals = {
        R[0], -R[0],
        R[1], -R[1],
        R[2], -R[2]
    };

    // escolhe face mais alinhada com supportNormal
    int bestIdx = -1;
    float bestDot = -FLT_MAX;
    for (int i = 0; i < 6; ++i) {
        glm::vec3 fn = faceNormals[i];
        float d = glm::dot(fn, supportNormal);
        if (d > bestDot) {
            bestDot = d;
            bestIdx = i;
        }
    }

    // evita snap se face pouco alinhada
    const float MIN_ALIGN = 0.25f; // ajuste se quiser mais/menos permissivo
    if (bestIdx < 0 || bestDot < MIN_ALIGN) return;

    // se j praticamente alinhado, apenas corrige penetracoes
    glm::vec3 chosenFrom = glm::normalize(faceNormals[bestIdx]);
    const auto& verts = s.geometry.vertices;
    if (bestDot > 0.9995f) {
        // apenas empurra para evitar interpenetraao com o plano definido por contactPoint+supportNormal
        //std::vector<glm::vec3> verts; 
        //BoxVerts verts;
        //getBoxVertices(s, verts);
        
        float minDist = FLT_MAX;
        for (auto& v : verts) {
            float d = glm::dot(v - contactPoint, supportNormal);
            minDist = std::min(minDist, d);
        }
        if (minDist < 0.0f) s.pos -= supportNormal * minDist;
        return;
    }

    // calcula rotaao que leva a face escolhida -> supportNormal
    glm::quat q = glm::rotation(chosenFrom, supportNormal);
    q = glm::normalize(q);

    // rotaciona em torno do ponto de contato (mantem contato onde est)
    glm::vec3 offset = contactPoint - s.pos;
    glm::vec3 rotatedOffset = glm::rotate(q, offset);
    s.pos = contactPoint - rotatedOffset;

    s.rot = glm::normalize(q * s.rot);

    // corrige penetraao pequena após rotaao (garante que vertices nAo fiquem "dentro" do suporte)
    
    float minDist = FLT_MAX;
    for (auto& v : verts) {
        float d = glm::dot(v - contactPoint, supportNormal);
        minDist = std::min(minDist, d);
    }
    if (minDist < 0.0f)
        s.pos -= supportNormal * minDist;
    s.geometryDirty = true;

    // amortecimento para evitar jitter
    s.angularVel *= 0.25f;
    s.vel *= 0.5f;
}


inline void snapBoxToSurface(Shape& s,
    const glm::vec3& supportNormal,
    const glm::vec3& contactPoint)
{
    
    if (s.isStatic) return;

    glm::mat3 R = glm::mat3_cast(s.rot);

    std::array<glm::vec3, 6> faceNormals = {
        R[0], -R[0],
        R[1], -R[1],
        R[2], -R[2]
    };

    int bestIdx = -1;
    float bestDot = -FLT_MAX;

    for (int i = 0; i < 6; ++i)
    {
        float d = glm::dot(glm::normalize(faceNormals[i]), supportNormal);
        if (d > bestDot)
        {
            bestDot = d;
            bestIdx = i;
        }
    }

    const float MIN_ALIGN = 0.4f;
    if (bestDot < MIN_ALIGN)
        return;

    if (bestDot > 0.9995f)
        return;

    glm::vec3 from = glm::normalize(faceNormals[bestIdx]);
    glm::quat q = glm::rotation(from, supportNormal);
    q = glm::normalize(q);

    glm::vec3 offset = contactPoint - s.pos;
    glm::vec3 rotatedOffset = glm::rotate(q, offset);

    s.pos = contactPoint - rotatedOffset;
    s.rot = glm::normalize(q * s.rot);
    s.geometryDirty = true;
    s.angularVel *= 0.2f;
    s.vel *= 0.4f;
}
inline void snapPyramidToSurface(Shape& s,
    const glm::vec3& supportNormal,
    const glm::vec3& contactPoint)
{
    if (s.isStatic) return;

    const auto& verts = s.geometry.vertices;
    if (verts.size() < 5) return;

    struct Tri { int a, b, c; };
    Tri faces[6] = {
        {0,1,2}, {0,2,3},
        {0,1,4},
        {1,2,4},
        {2,3,4},
        {3,0,4}
    };

    int bestFace = -1;
    float bestDot = -FLT_MAX;
    glm::vec3 bestNormal(0);

    for (int i = 0; i < 6; ++i)
    {
        glm::vec3 v0 = verts[faces[i].a];
        glm::vec3 v1 = verts[faces[i].b];
        glm::vec3 v2 = verts[faces[i].c];

        glm::vec3 n = glm::normalize(glm::cross(v1 - v0, v2 - v0));

        if (glm::dot(n, s.pos - v0) < 0.0f)
            n = -n;

        float d = glm::dot(n, supportNormal);

        if (d > bestDot)
        {
            bestDot = d;
            bestFace = i;
            bestNormal = n;
        }
    }

    const float MIN_ALIGN = 0.4f;
    if (bestDot < MIN_ALIGN)
        return;

    glm::quat q = glm::rotation(bestNormal, supportNormal);
    q = glm::normalize(q);

    glm::vec3 offset = contactPoint - s.pos;
    glm::vec3 rotatedOffset = glm::rotate(q, offset);

    s.pos = contactPoint - rotatedOffset;
    s.rot = glm::normalize(q * s.rot);
    s.geometryDirty = true;

    s.angularVel *= 0.2f;
    s.vel *= 0.4f;
}

inline void snapPyramidToSurfaceA(Shape& s,
    const glm::vec3& argA,
    const glm::vec3& argB)
{
    if (s.isStatic) return;

    //--------------------------------------------------
    // Detecta ordem (normal vs ponto)
    //--------------------------------------------------
    glm::vec3 supportNormal, contactPoint;
    guessNormalAndPoint(argA, argB, supportNormal, contactPoint, s);

    if (glm::length2(supportNormal) < 1e-8f)
        return;

    supportNormal = glm::normalize(supportNormal);

    //--------------------------------------------------
    // Obtem vertices da piramide
    //--------------------------------------------------
    const auto& verts = s.geometry.vertices;
    if (verts.size() < 5) return;

    //--------------------------------------------------
    // Faces da piramide (base + 4 laterais)
    //--------------------------------------------------
    struct Tri { int a, b, c; };
    Tri faces[6] =
    {
        {0,1,2}, {0,2,3},   // base (quad dividido)
        {0,1,4},
        {1,2,4},
        {2,3,4},
        {3,0,4}
    };

    //--------------------------------------------------
    // Escolhe face mais alinhada ao suporte
    //--------------------------------------------------
    int bestFace = -1;
    float bestDot = -FLT_MAX;
    glm::vec3 bestNormal(0);

    for (int i = 0; i < 6; ++i)
    {
        glm::vec3 v0 = verts[faces[i].a];
        glm::vec3 v1 = verts[faces[i].b];
        glm::vec3 v2 = verts[faces[i].c];

        glm::vec3 n = glm::normalize(glm::cross(v1 - v0, v2 - v0));

        // garante normal externa
        if (glm::dot(n, s.pos - v0) < 0.0f)
            n = -n;

        float d = glm::dot(n, supportNormal);

        if (d > bestDot)
        {
            bestDot = d;
            bestFace = i;
            bestNormal = n;
        }
    }

    const float MIN_ALIGN = 0.25f;
    if (bestFace < 0 || bestDot < MIN_ALIGN)
        return;

    //--------------------------------------------------
    // Se j praticamente alinhado → só corrige penetraao
    //--------------------------------------------------
    if (bestDot > 0.9995f)
    {
        float minDist = FLT_MAX;
        for (auto& v : verts)
        {
            float d = glm::dot(v - contactPoint, supportNormal);
            minDist = std::min(minDist, d);
        }

        if (minDist < 0.0f)
            s.pos -= supportNormal * minDist;

        return;
    }

    //--------------------------------------------------
    // Rotaao da face escolhida → supportNormal
    //--------------------------------------------------
    glm::quat q = glm::rotation(bestNormal, supportNormal);
    q = glm::normalize(q);

    glm::vec3 offset = contactPoint - s.pos;
    glm::vec3 rotatedOffset = glm::rotate(q, offset);

    s.pos = contactPoint - rotatedOffset;
    s.rot = glm::normalize(q * s.rot);

    //--------------------------------------------------
    // Corrige penetraao residual pós-rotaao
    //--------------------------------------------------
    //verts = s.geometry.vertices;

    float minDist = FLT_MAX;
    for (auto& v : verts)
    {
        float d = glm::dot(v - contactPoint, supportNormal);
        minDist = std::min(minDist, d);
    }

    if (minDist < 0.0f)
        s.pos -= supportNormal * minDist;

    //--------------------------------------------------
    // Amortecimento para evitar jitter
    //--------------------------------------------------
    // trava movimento normal ao plano
    float vn = glm::dot(s.vel, supportNormal);
    if (vn < 0.0f)
        s.vel -= supportNormal * vn;

    // trava rotaao residual pequena
    if (glm::length2(s.angularVel) < 0.0001f)
        s.angularVel = glm::vec3(0);
}
inline bool findSupportOnPlane(const Shape& s,
    const glm::vec3& planePoint,
    const glm::vec3& planeNormal,
    glm::vec3& outContactPoint,
    glm::vec3& outSupportNormal)
{
    const float SNAP_EPS = 0.02f;
    float minDist = FLT_MAX;

    if (s.type == ShapeType::Sphere)
    {
        float radius = s.scale.x * s.radius;
        glm::vec3 bottom = s.pos - planeNormal * radius;

        float dist = glm::dot(bottom - planePoint, planeNormal);

        if (std::abs(dist) <= SNAP_EPS)
        {
            outContactPoint = bottom - planeNormal * dist;
            outSupportNormal = planeNormal;
            return true;
        }
        return false;
    }
    else
    {
        const auto& verts = s.geometry.vertices;

        for (auto& v : verts)
        {
            float dist = glm::dot(v - planePoint, planeNormal);
            minDist = std::min(minDist, dist);
        }

        if (std::abs(minDist) > SNAP_EPS)
            return false;

        glm::vec3 sum(0.0f);
        int count = 0;

        for (auto& v : verts)
        {
            float dist = glm::dot(v - planePoint, planeNormal);
            if (std::abs(dist - minDist) <= 0.02f)
            {
                sum += v;
                count++;
            }
        }

        if (count == 0)
            return false;

        outContactPoint = sum / (float)count;
        outSupportNormal = planeNormal;
        return true;
    }

    return false;
}
inline bool findLocalSupport(const Shape& s,
    glm::vec3& outContactPoint,
    glm::vec3& outSupportNormal)
{
    const float GROUND_EPS = 0.01f;
    
    if (s.type == ShapeType::Sphere)
    {
        float radius = s.scale.x * s.radius;
        outContactPoint = s.pos - glm::vec3(0, radius, 0);
        outSupportNormal = glm::vec3(0, 1, 0);

        return outContactPoint.y <= GROUND_EPS;
    }

    else
    {
        float minY = FLT_MAX;
        const auto& verts = s.geometry.vertices;
        for (auto& v : verts)
            minY = std::min(minY, v.y);

        if (minY > GROUND_EPS)
            return false;

        glm::vec3 sum(0.0f);
        int count = 0;

        for (auto& v : verts)
        {
            if (v.y <= minY + 0.02f)
            {
                sum += v;
                count++;
            }
        }

        outContactPoint = sum / (float)count;
        outSupportNormal = glm::vec3(0, 1, 0);
        return true;
    }

    return false;
}
//----------------------------------------------------------
// SOLUaO DE CONTATO ENTRE CORPOS (CORRIGIDA)
//----------------------------------------------------------
// --- helpers para estabilidade entre box <-> pyramid ---------------------------------

// Contact manifold (rect/rect, rect/pyramid, pyramid/pyramid).
// SAT gives a separation normal; clipping the incident polygon against
// the reference polygon gives the actual face contact patch (up to 4 points).
struct ContactPolygon {
    glm::vec3 vertices[8];
    int count = 0;
    glm::vec3 normal = glm::vec3(0.0f);
};

static void sortPolygon(ContactPolygon& p) {
    if (p.count < 3) return;
    glm::vec3 center(0.0f);
    for (int i = 0; i < p.count; ++i) center += p.vertices[i];
    center /= static_cast<float>(p.count);
    const glm::vec3 seed = std::abs(p.normal.y) < 0.85f
        ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
    const glm::vec3 u = glm::normalize(glm::cross(seed, p.normal));
    const glm::vec3 v = glm::cross(p.normal, u);
    std::sort(p.vertices, p.vertices + p.count, [&](glm::vec3 a, glm::vec3 b) {
        return std::atan2(glm::dot(a - center, v), glm::dot(a - center, u)) <
               std::atan2(glm::dot(b - center, v), glm::dot(b - center, u));
    });
}

static ContactPolygon supportingFace(const Shape& s, const glm::vec3& direction,
                                     float* outAlignment = nullptr) {
    ContactPolygon p;
    float bestAlignment = -FLT_MAX;
    if (s.type == ShapeType::Rect) {
        const glm::mat3 R = glm::mat3_cast(s.rot);
        int axis = 0;
        float sign = 1.0f;
        for (int i = 0; i < 3; ++i)
            for (float side : { -1.0f, 1.0f }) {
                const float score = glm::dot(side * R[i], direction);
                if (score > bestAlignment) {
                    bestAlignment = score;
                    axis = i;
                    sign = side;
                }
            }
        p.normal = sign * R[axis];
        const glm::vec3 half = s.scale * 0.5f;
        for (int i = 0; i < 8; ++i) {
            const glm::vec3 local((i & 4) ? half.x : -half.x,
                                  (i & 2) ? half.y : -half.y,
                                  (i & 1) ? half.z : -half.z);
            if ((sign > 0.0f) == (local[axis] > 0.0f))
                p.vertices[p.count++] = s.pos + R * local;
        }
    } else if (s.type == ShapeType::Pyramid) {
        constexpr int faceSizes[5] = {3, 3, 3, 3, 4};
        constexpr int faceIDs[5][4] = {
            {0,1,4,0}, {1,2,4,0}, {2,3,4,0}, {3,0,4,0}, {0,1,2,3}
        };
        int face = -1;
        for (int i = 0; i < 5; ++i) {
            const float score = glm::dot(s.geometry.faces[i].n, direction);
            if (score > bestAlignment) {
                bestAlignment = score;
                face = i;
            }
        }
        if (face >= 0) {
            p.normal = s.geometry.faces[face].n;
            p.count = faceSizes[face];
            for (int i = 0; i < p.count; ++i)
                p.vertices[i] = s.geometry.vertices[faceIDs[face][i]];
        }
    }
    if (outAlignment) *outAlignment = bestAlignment;
    sortPolygon(p);
    return p;
}

static int contactPatch(const Contact& c, glm::vec3 (&points)[4]) {
    points[0] = c.point;
    if ((c.a->type != ShapeType::Rect && c.a->type != ShapeType::Pyramid) ||
        (c.b->type != ShapeType::Rect && c.b->type != ShapeType::Pyramid)) return 1;

    float alignA, alignB;
    ContactPolygon refA = supportingFace(*c.a, c.normal, &alignA);
    ContactPolygon refB = supportingFace(*c.b, -c.normal, &alignB);
    if (std::max(alignA, alignB) < 0.92f) return 1; // edge-edge
    const bool referenceIsA = alignA >= alignB;
    ContactPolygon reference = referenceIsA ? refA : refB;
    ContactPolygon incident = supportingFace(referenceIsA ? *c.b : *c.a,
                                            -reference.normal);
    if (reference.count < 3 || incident.count < 3) return 1;

    // Clip against four/three planes of the reference face.
    glm::vec3 clipped[16];
    int count = incident.count;
    for (int i = 0; i < count; ++i) clipped[i] = incident.vertices[i];
    for (int edge = 0; edge < reference.count && count; ++edge) {
        glm::vec3 output[16];
        int outCount = 0;
        const glm::vec3 a = reference.vertices[edge];
        const glm::vec3 b = reference.vertices[(edge + 1) % reference.count];
        const glm::vec3 inside = glm::cross(reference.normal, b - a);
        for (int i = 0; i < count; ++i) {
            const glm::vec3 p0 = clipped[i];
            const glm::vec3 p1 = clipped[(i + 1) % count];
            const float d0 = glm::dot(inside, p0 - a);
            const float d1 = glm::dot(inside, p1 - a);
            if ((d0 >= 0.0f) != (d1 >= 0.0f)) {
                const float fraction = d0 / (d0 - d1);
                if (outCount < 16) output[outCount++] = glm::mix(p0, p1, fraction);
            }
            if (d1 >= -1e-5f && outCount < 16) output[outCount++] = p1;
        }
        count = outCount;
        for (int i = 0; i < count; ++i) clipped[i] = output[i];
    }

    glm::vec3 candidates[16];
    int n = 0;
    for (int i = 0; i < count; ++i) {
        const float separation = glm::dot(reference.normal,
                                          clipped[i] - reference.vertices[0]);
        if (separation > 0.025f) continue;
        glm::vec3 midpoint = clipped[i] - 0.5f * separation * reference.normal;
        bool duplicate = false;
        for (int k = 0; k < n; ++k)
            if (glm::length2(candidates[k] - midpoint) < 1e-6f) duplicate = true;
        if (!duplicate && n < 16) candidates[n++] = midpoint;
    }
    if (!n) return 1;
    if (n <= 4) {
        for (int i = 0; i < n; ++i) points[i] = candidates[i];
        return n;
    }

    // Pick four spread-out points from a larger clipped polygon.
    glm::vec3 centroid(0.0f);
    for (int i = 0; i < n; ++i) centroid += candidates[i];
    centroid /= static_cast<float>(n);
    const glm::vec3 seed = std::abs(reference.normal.y) < 0.85f
        ? glm::vec3(0,1,0) : glm::vec3(1,0,0);
    const glm::vec3 u = glm::normalize(glm::cross(seed, reference.normal));
    const glm::vec3 v = glm::cross(reference.normal, u);
    const glm::vec3 dirs[4] = {u, v, -u, -v};
    int selected = 0;
    for (glm::vec3 direction : dirs) {
        float best = -FLT_MAX;
        int bestIdx = -1;
        for (int i = 0; i < n; ++i) {
            bool used = false;
            for (int k = 0; k < selected; ++k)
                if (glm::length2(points[k] - candidates[i]) < 1e-6f) used = true;
            const float score = glm::dot(candidates[i] - centroid, direction);
            if (!used && score > best) { best = score; bestIdx = i; }
        }
        if (bestIdx >= 0) points[selected++] = candidates[bestIdx];
    }
    return selected > 0 ? selected : 1;
}

static bool centerOverSupportFace(const Shape& base, const Shape& top) {
    if (base.type == ShapeType::Sphere) return false;
    float align;
    ContactPolygon face = supportingFace(base, glm::vec3(0,1,0), &align);
    if (align < 0.85f || face.count < 3) return false;
    // Point-in-polygon on horizontal plane: COM must project inside footprint.
    // Being supported by an edge alone does not qualify for sleep.
    bool inside = false;
    const float x = top.pos.x, z = top.pos.z;
    for (int i = 0, j = face.count - 1; i < face.count; j = i++) {
        const glm::vec3 a = face.vertices[i], b = face.vertices[j];
        const bool crosses = ((a.z > z) != (b.z > z)) &&
            (x < (b.x - a.x) * (z - a.z) / (b.z - a.z) + a.x);
        if (crosses) inside = !inside;
    }
    return inside;
}

static bool stationarySupport(const Shape& s) {
    return s.isStatic || s.isSleeping ||
           (s.isGrounded && glm::length2(s.vel) < 0.15f*0.15f &&
            glm::length2(s.angularVel) < 0.12f*0.12f);
}

static void markSupport(const Contact& c, const glm::vec3 (&points)[4], int contactCount) {
    if (std::abs(c.normal.y) < 0.62f || c.penetration < -0.01f) return;
    Shape& top = c.normal.y > 0.0f ? *c.b : *c.a;
    const Shape& base = c.normal.y > 0.0f ? *c.a : *c.b;
    if (top.isStatic || top.isDragging) return;
    top.isGrounded = true;
    // A wide support polygon is necessary for a sleeping box/pyramid.
    // Three nearly-collinear contacts at a balancing edge are NOT a face.
    float maxArea2 = 0.0f;
    for (int i = 0; i < contactCount; ++i)
        for (int j = i + 1; j < contactCount; ++j)
            for (int k = j + 1; k < contactCount; ++k)
                maxArea2 = std::max(maxArea2, glm::length(
                    glm::cross(points[j]-points[i], points[k]-points[i])));
    float topAlignment = 1.0f;
    if (top.type != ShapeType::Sphere)
        (void)supportingFace(top, glm::vec3(0,-1,0), &topAlignment);
    const bool faceContact = top.type == ShapeType::Sphere ||
        (contactCount >= 3 && maxArea2 > 0.003f && topAlignment > 0.985f);
    if (stationarySupport(base) && faceContact && centerOverSupportFace(base, top))
        top.supportIsStable = true;
}

static void solveContact(Contact& c, float /*dt*/)
{
    Shape& A = *c.a;
    Shape& B = *c.b;
    const float n2 = glm::length2(c.normal);
    if (n2 < 1e-10f || c.penetration < 0.0f) return;
    const glm::vec3 n = c.normal / std::sqrt(n2);

    glm::vec3 points[4];
    const int pointCount = contactPatch(c, points);
    markSupport(c, points, pointCount);

    // A resting body must remain dynamic. Wake it for a real impact, a
    // substantially displaced support, or contact with a moving platform.
    const glm::vec3 rvCenter = B.vel - A.vel;
    const float closing = glm::dot(rvCenter, n);
    const float relativeSpeed2 = glm::length2(rvCenter);
    const bool movingKinematicSupport =
        ((A.isStatic || A.isDragging) && glm::length2(A.vel) > 0.10f*0.10f) ||
        ((B.isStatic || B.isDragging) && glm::length2(B.vel) > 0.10f*0.10f);
    if (closing < -0.24f || c.penetration > 0.035f ||
        (movingKinematicSupport && relativeSpeed2 > 0.10f*0.10f)) {
        if (A.isSleeping && !A.isStatic && !A.isDragging) wake(A);
        if (B.isSleeping && !B.isStatic && !B.isDragging) wake(B);
    }

    // Treat a sleeping body as immovable for tiny resting impulses. Gravity
    // and integration resume after wake, so an anchored tower doesn't buzz.
    const float invMA = A.isSleeping ? 0.0f : getInvMass(A);
    const float invMB = B.isSleeping ? 0.0f : getInvMass(B);
    if (invMA + invMB < 1e-9f) return;
    const glm::mat3 invIA = A.isSleeping ? glm::mat3(0.0f) : computeWorldInvInertia(A);
    const glm::mat3 invIB = B.isSleeping ? glm::mat3(0.0f) : computeWorldInvInertia(B);

    // Position correction only once per manifold, not at each vertex.
    // Sleeping bodies keep their exact transforms until explicitly woken.
    constexpr float slop = 0.002f;
    constexpr float percent = 0.35f;
    const float correction = std::min(0.06f,
        percent * std::max(0.0f, c.penetration - slop) / (invMA + invMB));
    if (correction > 0.0f) {
        if (invMA > 0.0f) { A.pos -= correction * invMA * n; A.geometryDirty = true; }
        if (invMB > 0.0f) { B.pos += correction * invMB * n; B.geometryDirty = true; }
    }

    // Sequential impulses across the whole contact patch. Multiple contact
    // points counteract artificial spin from one arbitrarily chosen centroid.
    for (int pass = 0; pass < 2; ++pass) {
        for (int k = 0; k < pointCount; ++k) {
            const glm::vec3 cp = points[k];
            const glm::vec3 rA = cp - A.pos;
            const glm::vec3 rB = cp - B.pos;
            glm::vec3 vA = A.vel + glm::cross(A.angularVel, rA);
            glm::vec3 vB = B.vel + glm::cross(B.angularVel, rB);
            glm::vec3 rv = vB - vA;
            const float vn = glm::dot(rv, n);
            if (vn >= -1e-6f) continue;
            const glm::vec3 rAxn = glm::cross(rA, n);
            const glm::vec3 rBxn = glm::cross(rB, n);
            const float denomN = invMA + invMB +
                glm::dot(invIA * rAxn, rAxn) + glm::dot(invIB * rBxn, rBxn);
            if (denomN < 1e-9f) continue;
            const float e = vn < -2.0f ? 0.035f : 0.0f;
            const float jn = -(1.0f + e) * vn / denomN;
            const glm::vec3 Jn = jn * n;
            A.vel -= invMA * Jn;
            B.vel += invMB * Jn;
            A.angularVel -= invIA * glm::cross(rA, Jn);
            B.angularVel += invIB * glm::cross(rB, Jn);

            vA = A.vel + glm::cross(A.angularVel, rA);
            vB = B.vel + glm::cross(B.angularVel, rB);
            rv = vB - vA;
            glm::vec3 t = rv - glm::dot(rv, n) * n;
            const float vt = glm::length(t);
            if (vt < 1e-7f) continue;
            t /= vt;
            const glm::vec3 rAt = glm::cross(rA, t);
            const glm::vec3 rBt = glm::cross(rB, t);
            const float denomT = invMA + invMB +
                glm::dot(invIA * rAt, rAt) + glm::dot(invIB * rBt, rBt);
            if (denomT < 1e-9f) continue;
            const float jt = std::min(vt / denomT, 0.85f * jn);
            const glm::vec3 Jt = -jt * t;
            A.vel -= invMA * Jt;
            B.vel += invMB * Jt;
            A.angularVel -= invIA * glm::cross(rA, Jt);
            B.angularVel += invIB * glm::cross(rB, Jt);
        }
    }
}

// --- Sphere x Pyramid (face contact robusta) ---
// Closest point on a triangle (Ericson, Real-Time Collision Detection).
static glm::vec3 closestPointTriangle(const glm::vec3& p,
                                     const glm::vec3& a,
                                     const glm::vec3& b,
                                     const glm::vec3& c) {
    const glm::vec3 ab = b-a, ac = c-a, ap = p-a;
    const float d1 = glm::dot(ab,ap), d2 = glm::dot(ac,ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const glm::vec3 bp = p-b;
    const float d3 = glm::dot(ab,bp), d4 = glm::dot(ac,bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const float vc = d1*d4 - d3*d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + (d1/(d1-d3))*ab;
    const glm::vec3 cp = p-c;
    const float d5 = glm::dot(ab,cp), d6 = glm::dot(ac,cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const float vb = d5*d2 - d1*d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + (d2/(d2-d6))*ac;
    const float va = d3*d6 - d5*d4;
    if (va <= 0 && (d4-d3) >= 0 && (d5-d6) >= 0)
        return b + ((d4-d3)/((d4-d3)+(d5-d6)))*(c-b);
    const float denom = 1.0f/(va+vb+vc);
    return a + ab*(vb*denom) + ac*(vc*denom);
}

static bool spherePyramid(Shape& sphere, Shape& pyr, Contact& c)
{
    const float r = sphereRadius(sphere);
    // Six triangles cover pyramid's base and four lateral faces.
    constexpr int triangles[6][3] = {
        {0,1,2}, {0,2,3}, {0,1,4}, {1,2,4}, {2,3,4}, {3,0,4}
    };
    glm::vec3 closest(0.0f);
    float dist2 = FLT_MAX;
    for (auto const& tri : triangles) {
        const glm::vec3 q = closestPointTriangle(sphere.pos,
            pyr.geometry.vertices[tri[0]], pyr.geometry.vertices[tri[1]],
            pyr.geometry.vertices[tri[2]]);
        const float d2 = glm::length2(q - sphere.pos);
        if (d2 < dist2) { dist2 = d2; closest = q; }
    }
    bool inside = true;
    float nearestExit = -FLT_MAX;
    glm::vec3 exitNormal(0,1,0);
    for (int i = 0; i < pyr.geometry.faceCount; ++i) {
        const float distance = glm::dot(pyr.geometry.faces[i].n, sphere.pos)
                               + pyr.geometry.faces[i].d;
        if (distance > 0) inside = false;
        if (distance > nearestExit) {
            nearestExit = distance;
            exitNormal = pyr.geometry.faces[i].n;
        }
    }
    if (!inside && dist2 > r*r) return false;
    c.a = &pyr;
    c.b = &sphere;
    if (inside) {
        c.normal = exitNormal;
        c.penetration = r - nearestExit;
        c.point = closest;
    } else {
        const float dist = std::sqrt(dist2);
        c.normal = dist > 1e-6f ? (sphere.pos - closest)/dist : exitNormal;
        c.penetration = r - dist;
        c.point = closest;
    }
    return true;
}
//----------------------------------------------------------
// DETECaO DE COLISÕES
//----------------------------------------------------------
static bool sphereSphere(Shape& a, Shape& b, Contact& c) {
    float ra = sphereRadius(a);
    float rb = sphereRadius(b);

    glm::vec3 d = b.pos - a.pos;
    float dist2 = glm::length2(d);
    float r = ra + rb;

    if (dist2 > r * r) return false;

    float dist = std::sqrt(dist2);
    glm::vec3 n = (dist > 1e-6f) ? d / dist : glm::vec3(0, 1, 0);

    c = { &a, &b, a.pos + n * ra, n, r - dist };
    return true;
}
static glm::vec3 averageExtremeVertices(const Shape& s, const glm::vec3& dir)
{
    float maxProjection = -FLT_MAX;
    for (int i = 0; i < s.geometry.vertexCount; ++i)
        maxProjection = std::max(maxProjection, glm::dot(s.geometry.vertices[i], dir));

    glm::vec3 sum(0.0f);
    int count = 0;
    for (int i = 0; i < s.geometry.vertexCount; ++i) {
        if (glm::dot(s.geometry.vertices[i], dir) >= maxProjection - 0.015f) {
            sum += s.geometry.vertices[i];
            ++count;
        }
    }
    return count ? sum / static_cast<float>(count) : s.pos;
}

// ---- BOX-BOX SAT ----
static bool boxBox(const Shape& a, const Shape& b, Contact& c)
{
    const glm::mat3 Ra = glm::mat3_cast(a.rot);
    const glm::mat3 Rb = glm::mat3_cast(b.rot);
    float minOverlap = FLT_MAX;
    glm::vec3 bestAxis(0.0f);

    auto testAxis = [&](glm::vec3 axis) -> bool {
        const float len2 = glm::length2(axis);
        if (len2 < 1e-8f) return true;
        axis /= std::sqrt(len2); // SAT penetration must use unit axes.

        float minA, maxA, minB, maxB;
        projectPoints(a.geometry.vertices, a.geometry.vertexCount, axis, minA, maxA);
        projectPoints(b.geometry.vertices, b.geometry.vertexCount, axis, minB, maxB);
        const float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap < 0.0f) return false;
        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = glm::dot(b.pos - a.pos, axis) >= 0.0f ? axis : -axis;
        }
        return true;
    };

    for (int i = 0; i < 3; ++i) if (!testAxis(Ra[i])) return false;
    for (int i = 0; i < 3; ++i) if (!testAxis(Rb[i])) return false;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (!testAxis(glm::cross(Ra[i], Rb[j]))) return false;

    if (glm::length2(bestAxis) < 0.5f) return false;
    c.a = const_cast<Shape*>(&a);
    c.b = const_cast<Shape*>(&b);
    c.normal = bestAxis;
    c.penetration = minOverlap;
    // Real face-oriented support point, not the average of both body centers.
    c.point = 0.5f * (averageExtremeVertices(a, bestAxis) +
                       averageExtremeVertices(b, -bestAxis));
    return true;
}

// ---- BOX-PYRAMID SAT ----
static bool boxPyramid(const Shape& box, const Shape& pyr, Contact& c)
{
    const glm::mat3 R = glm::mat3_cast(box.rot);
    float minOverlap = FLT_MAX;
    glm::vec3 bestAxis(0.0f);

    auto testAxis = [&](glm::vec3 axis) -> bool {
        const float len2 = glm::length2(axis);
        if (len2 < 1e-8f) return true;
        axis /= std::sqrt(len2);

        float minA, maxA, minB, maxB;
        projectPoints(box.geometry.vertices, box.geometry.vertexCount, axis, minA, maxA);
        projectPoints(pyr.geometry.vertices, pyr.geometry.vertexCount, axis, minB, maxB);
        const float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap < 0.0f) return false;
        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = glm::dot(pyr.pos - box.pos, axis) >= 0.0f ? axis : -axis;
        }
        return true;
    };

    for (int i = 0; i < 3; ++i) if (!testAxis(R[i])) return false;
    for (int i = 0; i < pyr.geometry.faceCount; ++i)
        if (!testAxis(pyr.geometry.faces[i].n)) return false;

    // Valid SAT cross axes are box edges crossed with pyramid edges.
    constexpr int edges[8][2] = {
        {0,1}, {1,2}, {2,3}, {3,0},
        {0,4}, {1,4}, {2,4}, {3,4}
    };
    for (int i = 0; i < 3; ++i) {
        for (const auto& e : edges) {
            const glm::vec3 edge =
                pyr.geometry.vertices[e[1]] - pyr.geometry.vertices[e[0]];
            if (!testAxis(glm::cross(R[i], edge))) return false;
        }
    }

    if (glm::length2(bestAxis) < 0.5f) return false;
    c.a = const_cast<Shape*>(&box);
    c.b = const_cast<Shape*>(&pyr);
    c.normal = bestAxis;
    c.penetration = minOverlap;
    c.point = 0.5f * (averageExtremeVertices(box, bestAxis) +
                       averageExtremeVertices(pyr, -bestAxis));
    return true;
}

static bool pyramidPyramid(const Shape& a, const Shape& b, Contact& c)
{
    float minOverlap = FLT_MAX;
    glm::vec3 bestAxis(0.0f);
    auto testAxis = [&](glm::vec3 axis) -> bool {
        const float len2 = glm::length2(axis);
        if (len2 < 1e-8f) return true;
        axis /= std::sqrt(len2);
        float minA,maxA,minB,maxB;
        projectPoints(a.geometry.vertices, a.geometry.vertexCount, axis, minA,maxA);
        projectPoints(b.geometry.vertices, b.geometry.vertexCount, axis, minB,maxB);
        const float overlap = std::min(maxA,maxB)-std::max(minA,minB);
        if (overlap < 0.0f) return false;
        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = glm::dot(b.pos-a.pos,axis) >= 0 ? axis : -axis;
        }
        return true;
    };
    for (int i=0;i<a.geometry.faceCount;++i)
        if (!testAxis(a.geometry.faces[i].n)) return false;
    for (int i=0;i<b.geometry.faceCount;++i)
        if (!testAxis(b.geometry.faces[i].n)) return false;
    constexpr int edges[8][2] = {
        {0,1}, {1,2}, {2,3}, {3,0}, {0,4}, {1,4}, {2,4}, {3,4}
    };
    for (const auto& ea : edges) for (const auto& eb : edges) {
        const glm::vec3 da = a.geometry.vertices[ea[1]]-a.geometry.vertices[ea[0]];
        const glm::vec3 db = b.geometry.vertices[eb[1]]-b.geometry.vertices[eb[0]];
        if (!testAxis(glm::cross(da,db))) return false;
    }
    if (glm::length2(bestAxis) < 0.5f) return false;
    c.a = const_cast<Shape*>(&a);
    c.b = const_cast<Shape*>(&b);
    c.normal = bestAxis;
    c.penetration = minOverlap;
    c.point = 0.5f * (averageExtremeVertices(a,bestAxis) +
                      averageExtremeVertices(b,-bestAxis));
    return true;
}

static bool sphereBox(Shape& sphere, Shape& box, Contact& c)
{
    // Resolve the closest point in the oriented box's LOCAL coordinate space.
    const glm::mat3 R = glm::mat3_cast(box.rot);
    const glm::vec3 local = glm::transpose(R) * (sphere.pos - box.pos);
    const glm::vec3 h = getHalfExtents(box);
    const glm::vec3 clamped = glm::clamp(local, -h, h);
    const glm::vec3 delta = local - clamped;
    const float r = sphereRadius(sphere);
    const float dist2 = glm::length2(delta);
    if (dist2 > r * r) return false;

    glm::vec3 normalLocal(0.0f);
    glm::vec3 contactLocal = clamped;
    float penetration;
    if (dist2 > 1e-10f) {
        const float dist = std::sqrt(dist2);
        normalLocal = delta / dist;
        penetration = r - dist;
    } else {
        // The sphere center is inside/on the box. Pick the nearest EXIT
        // face instead of inventing an upward normal, which caused popping.
        const glm::vec3 faceDist = h - glm::abs(local);
        int axis = 0;
        if (faceDist.y < faceDist[axis]) axis = 1;
        if (faceDist.z < faceDist[axis]) axis = 2;
        normalLocal[axis] = local[axis] >= 0.0f ? 1.0f : -1.0f;
        contactLocal = local;
        contactLocal[axis] = normalLocal[axis] * h[axis];
        penetration = r + faceDist[axis];
    }

    c.a = &box;
    c.b = &sphere;
    c.normal = glm::normalize(R * normalLocal);
    c.point = box.pos + R * contactLocal;
    c.penetration = penetration;
    return true;
}

// Leave sleeping dynamic bodies dynamic. They wake from contacts; unlike the
// previous sleepCheck(), this never sets isStatic=true permanently.
static void updateSleepState(Shape& s, float dt)
{
    if (s.isStatic || s.isDragging) return;
    if (s.isSleeping) {
        // Moving a support away (or losing balance) wakes a resting body.
        if (!s.isGrounded || !s.supportIsStable) wake(s);
        return;
    }
    const float linearThreshold = 0.085f;
    const float angularThreshold = 0.075f;
    const bool quiet = glm::length2(s.vel) < linearThreshold*linearThreshold &&
                       glm::length2(s.angularVel) < angularThreshold*angularThreshold;
    if (!quiet || !s.isGrounded || !s.supportIsStable) {
        s.sleepTimer = 0.0f;
        return;
    }
    s.sleepTimer += dt;
    if (s.sleepTimer >= 0.65f) {
        s.isSleeping = true;
        s.vel = glm::vec3(0.0f);
        s.angularVel = glm::vec3(0.0f);
    }
}

// One bounded physics step. Collision pairs are regenerated after positional
// correction so that the SAT never uses stale world-space vertices.
static void physicsSubstep(Scene& scene, float dt)
{
    auto& spheres = scene.spheresRef();
    auto& boxes = scene.rectsRef();
    auto& pyramids = scene.pyramidsRef();

    auto applyToAll = [&](auto fn) {
        for (auto& s : spheres) fn(s);
        for (auto& s : boxes) fn(s);
        for (auto& s : pyramids) fn(s);
    };
    auto updateGeometry = [&](Shape& s) {
        if (s.geometryDirty) {
            updateShapeWorldGeometry(s);
            s.geometryDirty = false;
        }
    };
    applyToAll([&](Shape& s) {
        integrateGravity(s, dt);
        integratePosition(s, dt);
        integrateRotation(s, dt);
    });

    // Contacts must be recomputed as shapes move, including all floors and
    // pyramid/pyramid pairs. Previously only 3 solver passes were used.
    for (int iteration = 0; iteration < 8; ++iteration) {
        applyToAll(updateGeometry);
        applyToAll([&](Shape& s) { handleGroundContact(s, dt); });
        applyToAll(updateGeometry);

        std::vector<AABB> sphereAABB(spheres.size());
        std::vector<AABB> boxAABB(boxes.size());
        std::vector<AABB> pyramidAABB(pyramids.size());
        for (size_t i = 0; i < spheres.size(); ++i)
            sphereAABB[i] = computeAABB(spheres[i]);
        for (size_t i = 0; i < boxes.size(); ++i)
            boxAABB[i] = computeAABB(boxes[i]);
        for (size_t i = 0; i < pyramids.size(); ++i)
            pyramidAABB[i] = computeAABB(pyramids[i]);

        Contact c;
        for (size_t i = 0; i < spheres.size(); ++i)
            for (size_t j = i + 1; j < spheres.size(); ++j)
                if (aabbOverlap(sphereAABB[i], sphereAABB[j]) &&
                    sphereSphere(spheres[i], spheres[j], c)) solveContact(c, dt);

        for (size_t i = 0; i < spheres.size(); ++i)
            for (size_t j = 0; j < boxes.size(); ++j)
                if (aabbOverlap(sphereAABB[i], boxAABB[j]) &&
                    sphereBox(spheres[i], boxes[j], c)) solveContact(c, dt);

        for (size_t i = 0; i < boxes.size(); ++i)
            for (size_t j = i + 1; j < boxes.size(); ++j)
                if (aabbOverlap(boxAABB[i], boxAABB[j]) &&
                    boxBox(boxes[i], boxes[j], c)) solveContact(c, dt);

        for (size_t i = 0; i < boxes.size(); ++i)
            for (size_t j = 0; j < pyramids.size(); ++j)
                if (aabbOverlap(boxAABB[i], pyramidAABB[j]) &&
                    boxPyramid(boxes[i], pyramids[j], c)) solveContact(c, dt);

        for (size_t i = 0; i < spheres.size(); ++i)
            for (size_t j = 0; j < pyramids.size(); ++j)
                if (aabbOverlap(sphereAABB[i], pyramidAABB[j]) &&
                    spherePyramid(spheres[i], pyramids[j], c)) solveContact(c, dt);

        for (size_t i = 0; i < pyramids.size(); ++i)
            for (size_t j = i + 1; j < pyramids.size(); ++j)
                if (aabbOverlap(pyramidAABB[i], pyramidAABB[j]) &&
                    pyramidPyramid(pyramids[i], pyramids[j], c)) solveContact(c, dt);
    }
    applyToAll([&](Shape& s) { updateSleepState(s, dt); });
}

void updatePhysics(Scene& scene, float dt)
{
    if (dt <= 0.0f) return;
    // Bound the accumulated frame time rather than integrating a huge step
    // after alt-tab, a breakpoint, or a stalled frame.
    dt = std::min(dt, 1.0f / 20.0f);
    const int substeps = std::clamp(static_cast<int>(std::ceil(dt / (1.0f / 120.0f))), 1, 8);
    const float h = dt / static_cast<float>(substeps);
    for (int i = 0; i < substeps; ++i) physicsSubstep(scene, h);
}
