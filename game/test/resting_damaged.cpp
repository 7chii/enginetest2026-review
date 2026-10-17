// Run from project root:
// g++ -std=c++17 -O2 -I. tests/resting_damaged.cpp physics.cpp physics_geometry.cpp -o build/resting_damaged
// ./build/resting_damaged
#include <cmath>
#include <iostream>
#include <glm/gtc/matrix_inverse.hpp>
#include "physics.h"
#include "destruction.h"

static Shape makeBox(int id, glm::vec3 p, glm::vec3 size, bool fixed) {
    Shape s;
    s.id=id;
    s.pos=p;
    s.scale=size;
    s.mass=1.0f;
    s.isStatic=fixed;
    s.useGravity=!fixed;
    setupRectShape(s);
    s.invInertia=glm::inverse(computeLocalInertiaTensor(s));
    return s;
}
static Shape makePyramid(int id, glm::vec3 p, glm::vec3 size) {
    Shape s;
    s.id=id; s.pos=p; s.scale=size;
    s.mass=1.0f; s.useGravity=true;
    setupPyramidShape(s);
    s.invInertia=glm::inverse(computeLocalInertiaTensor(s));
    return s;
}
static void run(Scene& sc, float seconds) {
    for (int i=0;i<int(seconds*120.0f);++i)
        updatePhysics(sc,1.0f/120.0f);
}
static bool result(const char* name, bool pass) {
    std::cout<<(pass?"PASS ":"FAIL ")<<name<<'\n';
    return pass;
}
static void stateIfFailed(const char* label, const Shape& s, bool passed) {
    if (passed) return;
    std::cout << "  " << label
              << " pos=(" << s.pos.x << "," << s.pos.y << "," << s.pos.z << ")"
              << " vel=(" << s.vel.x << "," << s.vel.y << "," << s.vel.z << ")"
              << " omega=(" << s.angularVel.x << "," << s.angularVel.y << ","
              << s.angularVel.z << ")"
              << " grounded=" << s.isGrounded
              << " stable=" << s.supportIsStable
              << " timer=" << s.sleepTimer << '\n';
}
int main() {
    bool ok=true;
    // The damaged shape is the UPPER object. Its bottom face must be able to
    // rest on the UNDAMAGED static base; previously support was marked backwards.
    {
        Scene sc;
        Shape base=makeBox(10,{0,1,0},{4,2,4},true);
        Shape top=makeBox(11,{0.9f,2.8f,0},{0.8f,0.8f,0.8f},false);
        ok &= result("Carve upper box",Destruction::carve(top,{0.9f,3.2f,0},{0,1,0},0.15f,0.25f).changed());
        sc.rectsRef().push_back(base); sc.rectsRef().push_back(top);
        run(sc,1.8f);
        const Shape& settled=sc.rectsRef().back();
        ok &= result("Drilled top rests on intact base",
                     std::abs(settled.pos.y-2.4f)<0.15f);
        ok &= result("Drilled top enters sleep",settled.isSleeping);
    }
    // Both shapes have holes; the upper is placed on a surviving flat patch.
    {
        Scene sc;
        Shape base=makeBox(12,{0,1,0},{4,2,4},true);
        Shape top=makeBox(13,{1.0f,2.8f,0},{0.8f,0.8f,0.8f},false);
        Destruction::carve(base,{0,2,0},{0,1,0},0.28f,0.5f);
        Destruction::carve(top,{1.0f,3.2f,0},{0,1,0},0.15f,0.2f);
        sc.rectsRef().push_back(base); sc.rectsRef().push_back(top);
        run(sc,1.8f);
        const Shape& settled=sc.rectsRef().back();
        const bool sleep=settled.isSleeping;
        ok &= result("Two drilled bodies settle",sleep);
        stateIfFailed("drilled pair",settled,sleep);
        // A spurious side-wall contact on the large base used to push the
        // small box sideways until it fell, even with COM above a wide face.
        const bool centered=std::abs(settled.pos.x-1.0f)<0.28f &&
                            std::abs(settled.pos.z)<0.28f;
        ok &= result("No false horizontal impulse from drilled support",centered);
        stateIfFailed("lateral drift",settled,centered);
    }
    // A drilled pyramid must still rest on its wide horizontal base.
    {
        Scene sc;
        Shape base=makeBox(18,{0,1,0},{4,2,4},true);
        Shape top=makePyramid(19,{0.85f,2.8f,0},{0.8f,0.8f,0.8f});
        ok &= result("Carve pyramid side",Destruction::carve(
            top,{1.05f,2.8f,0},{1,0,0},0.09f,0.18f).changed());
        sc.rectsRef().push_back(base);sc.pyramidsRef().push_back(top);
        run(sc,1.8f);
        const Shape& settled=sc.pyramidsRef().back();
        const bool sleep=settled.isSleeping;
        ok &= result("Drilled pyramid reaches sleep on base",sleep);
        stateIfFailed("pyramid",settled,sleep);
    }
    // A resting child must wake when the support moves away: sleep is not glue.
    {
        Scene sc;
        Shape base=makeBox(14,{0,1,0},{4,2,4},true);
        Shape top=makeBox(15,{0.8f,2.8f,0},{0.8f,0.8f,0.8f},false);
        Destruction::carve(top,{0.8f,3.2f,0},{0,1,0},0.15f,0.2f);
        sc.rectsRef().push_back(base);sc.rectsRef().push_back(top);
        run(sc,1.8f);
        const bool beforeSleep=sc.rectsRef().back().isSleeping;
        ok &= result("Sleep before removal",beforeSleep);
        stateIfFailed("before removal",sc.rectsRef().back(),beforeSleep);
        sc.rectsRef()[0].pos.x+=8.0f;
        sc.rectsRef()[0].geometryDirty=true;
        run(sc,0.25f);
        ok &= result("Loss of support wakes top",!sc.rectsRef().back().isSleeping);
    }
    // Overhanging centre of mass: a tiny corner patch is NOT stable support.
    {
        Scene sc;
        Shape base=makeBox(16,{0,1,0},{2,2,2},true);
        Shape top=makeBox(17,{1.45f,2.65f,0},{1,1,1},false);
        Destruction::carve(top,{1.45f,3.15f,0},{0,1,0},0.15f,0.20f);
        sc.rectsRef().push_back(base);sc.rectsRef().push_back(top);
        run(sc,0.4f);
        ok &= result("Overhang does not gain stable support",
                     !sc.rectsRef().back().supportIsStable ||
                     sc.rectsRef().back().pos.y<2.3f);
    }
    return ok?0:1;
}
