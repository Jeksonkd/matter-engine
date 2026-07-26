// Headless checks for p2d's rigid joints (Joint.hpp): DistanceJoint,
// RevoluteJoint, WeldJoint, PrismaticJoint. These are genuinely bilateral
// (equality) velocity constraints solved by World's sequential-impulse
// solver -- unlike SpringJoint, they hold firmly rather than stretching
// under load.

#include "p2d/World.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace p2d;

namespace {

int g_failures = 0;

void check(bool condition, const char* description) {
    if (condition) {
        std::printf("[PASS] %s\n", description);
    } else {
        std::printf("[FAIL] %s\n", description);
        ++g_failures;
    }
}

} // namespace

int main() {
    const float dt = 1.0f / 120.0f;

    // --- DistanceJoint: holds its length under gravity, doesn't sag -------
    {
        World world;
        Body* anchor = world.createCircle({0.0f, 5.0f}, 0.1f, BodyType::Static);
        Body* bob = world.createCircle({0.0f, 0.0f}, 0.3f, BodyType::Dynamic);
        world.createDistanceJoint(anchor, bob, anchor->position, bob->position);

        for (int i = 0; i < 600; ++i) world.step(dt); // 5s, well past settling

        float dist = (bob->position - anchor->position).length();
        check(std::fabs(dist - 5.0f) < 0.1f,
              "DistanceJoint holds its 5m length under gravity (settled distance within 0.1m)");
    }

    // --- DistanceJoint holds even under a hard pull ------------------------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({-2.5f, 0.0f}, 0.3f, BodyType::Dynamic);
        Body* b = world.createCircle({2.5f, 0.0f}, 0.3f, BodyType::Dynamic);
        world.createDistanceJoint(a, b, a->position, b->position); // length = 5
        a->velocity = Vec2(-3.0f, 0.0f); // pulling apart hard
        b->velocity = Vec2(3.0f, 0.0f);

        for (int i = 0; i < 120; ++i) world.step(dt); // 1s

        float dist = (b->position - a->position).length();
        check(dist < 6.0f, "DistanceJoint resists being pulled far past its rest length (stays under 6m, not 11m)");
    }

    // --- RevoluteJoint: anchor points stay coincident under gravity -------
    {
        World world;
        Body* anchor = world.createCircle({0.0f, 5.0f}, 0.1f, BodyType::Static);
        Body* arm = world.createBox({2.0f, 5.0f}, 1.0f, 0.2f, BodyType::Dynamic); // extends to the right of the pivot
        world.createRevoluteJoint(anchor, arm, anchor->position); // pinned at anchor's own center

        for (int i = 0; i < 300; ++i) world.step(dt); // 2.5s -- swings like a pendulum

        // The arm's center of mass is 2m from the pivot at creation (pivot
        // at the arm's local (-1,0)) -- a rigid pin keeps it swinging on
        // that same radius rather than drifting toward or away from it.
        float distFromPivot = (arm->position - anchor->position).length();
        check(std::fabs(distFromPivot - 2.0f) < 0.15f,
              "RevoluteJoint keeps the arm at a constant ~2m radius from its pivot (swings, doesn't drift)");
    }

    // --- RevoluteJoint: relative rotation is free ---------------------------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({0.0f, 0.0f}, 0.5f, BodyType::Static);
        Body* b = world.createBox({1.0f, 0.0f}, 1.0f, 0.2f, BodyType::Dynamic);
        world.createRevoluteJoint(a, b, {0.0f, 0.0f});
        b->applyTorque(50.0f);

        for (int i = 0; i < 60; ++i) world.step(dt); // 0.5s

        check(std::fabs(b->angularVelocity) > 0.1f,
              "RevoluteJoint allows free relative rotation -- a torque actually spins the body");
        // And the pin should still hold despite the spin.
        Vec2 rB = rotate(Vec2(-1.0f, 0.0f), b->rotation); // b's local anchor was its own left edge, at creation
        float pinError = (b->position + rB - a->position).length();
        check(pinError < 0.2f, "RevoluteJoint's pin still holds while the body spins freely around it");
    }

    // --- WeldJoint: locks relative rotation too -----------------------------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({0.0f, 0.0f}, 0.5f, BodyType::Static);
        Body* b = world.createBox({1.0f, 0.0f}, 1.0f, 0.2f, BodyType::Dynamic);
        world.createWeldJoint(a, b, {0.0f, 0.0f});
        b->applyTorque(50.0f);

        for (int i = 0; i < 60; ++i) world.step(dt); // 0.5s

        check(std::fabs(b->rotation - a->rotation) < 0.1f,
              "WeldJoint keeps the relative angle fixed even when torque is applied (unlike Revolute)");
        check(std::fabs(b->angularVelocity) < 0.1f,
              "WeldJoint's angle lock stops the torqued body from actually spinning (A is Static/immovable)");
    }

    // --- PrismaticJoint: slides along its axis, doesn't drift sideways -----
    {
        World world;
        Body* a = world.createBox({0.0f, 5.0f}, 2.0f, 0.2f, BodyType::Static); // a horizontal rail
        Body* b = world.createBox({0.0f, 5.0f}, 0.3f, 0.3f, BodyType::Dynamic); // a slider hanging on it
        world.createPrismaticJoint(a, b, {0.0f, 5.0f}, {0.0f, -1.0f}); // slides straight down

        for (int i = 0; i < 120; ++i) world.step(dt); // 1s of falling along the rail

        check(b->position.y < 4.5f, "PrismaticJoint lets the slider move along its axis (falls under gravity)");
        check(std::fabs(b->position.x - 0.0f) < 0.05f,
              "PrismaticJoint keeps the slider from drifting perpendicular to its axis");
        check(std::fabs(b->rotation - a->rotation) < 0.05f,
              "PrismaticJoint locks relative rotation (slider doesn't tumble while sliding)");
    }

    // --- Removing a body drops any joint touching it (no dangling ptr) -----
    {
        World world;
        Body* a = world.createCircle({0.0f, 0.0f}, 0.3f, BodyType::Static);
        Body* b = world.createCircle({1.0f, 0.0f}, 0.3f, BodyType::Dynamic);
        world.createDistanceJoint(a, b, a->position, b->position);
        check(world.distanceJoints().size() == 1, "sanity: the joint was created");
        world.removeBody(b);
        check(world.distanceJoints().empty(), "removing a body drops any joint touching it");
        // Would crash/UB if a dangling Body* were dereferenced here.
        for (int i = 0; i < 10; ++i) world.step(dt);
        check(true, "stepping after removing a jointed body doesn't crash");
    }

    if (g_failures == 0) {
        std::printf("\nAll joint tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d joint test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
