// Headless checks for the newer per-body properties: linear/angular
// damping, gravity scale, fixed rotation, and sensors. These are real
// physics behaviors (not just Inspector cosmetics), so they're worth
// verifying numerically the same way the core solver is.

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

    // --- linearDamping: slows a coasting body with no other forces -------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* b = world.createCircle({0.0f, 0.0f}, 0.5f, BodyType::Dynamic);
        b->velocity = {10.0f, 0.0f};
        b->linearDamping = 5.0f;

        for (int i = 0; i < 240; ++i) world.step(dt); // 2 simulated seconds

        check(b->velocity.length() < 1.0f, "linearDamping substantially slows a coasting body over 2 seconds");
        check(b->velocity.x >= 0.0f, "damping decays velocity without reversing its direction");
    }

    // --- gravityScale: 0 floats, 1 falls normally, 2 falls faster ---------
    {
        World world;
        world.gravity = {0.0f, -10.0f};

        Body* zeroG = world.createCircle({-3.0f, 5.0f}, 0.3f, BodyType::Dynamic);
        zeroG->gravityScale = 0.0f;
        Body* normalG = world.createCircle({0.0f, 5.0f}, 0.3f, BodyType::Dynamic);
        normalG->gravityScale = 1.0f;
        Body* doubleG = world.createCircle({3.0f, 5.0f}, 0.3f, BodyType::Dynamic);
        doubleG->gravityScale = 2.0f;

        for (int i = 0; i < 60; ++i) world.step(dt); // 0.5s, no ground -> pure free fall

        check(std::fabs(zeroG->position.y - 5.0f) < 0.01f, "gravityScale = 0 body does not fall");
        check(normalG->position.y < 4.9f, "gravityScale = 1 body falls normally");
        check(doubleG->position.y < normalG->position.y, "gravityScale = 2 body falls further than gravityScale = 1");
    }

    // --- fixedRotation: torque/off-center force never spins the body ------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* b = world.createCircle({0.0f, 0.0f}, 0.5f, BodyType::Dynamic);
        b->fixedRotation = true;
        b->computeMass(); // required after changing fixedRotation, per its documented contract

        b->applyForceAtPoint(Vec2(10.0f, 0.0f), Vec2(0.0f, 0.3f)); // off-center -> would normally torque it
        for (int i = 0; i < 60; ++i) world.step(dt);

        check(b->angularVelocity == 0.0f, "fixedRotation body has exactly zero angular velocity");
        check(b->rotation == 0.0f, "fixedRotation body never rotates");
        check(b->velocity.x > 0.0f, "fixedRotation body still translates normally from the same force");
    }

    // --- isSensor: detects contact but applies no collision response ------
    {
        bool sensorContactFired = false;
        World sensorWorld;
        sensorWorld.gravity = {0.0f, 0.0f};
        Body* s1 = sensorWorld.createCircle({-0.1f, 0.0f}, 0.5f, BodyType::Dynamic);
        Body* s2 = sensorWorld.createCircle({0.1f, 0.0f}, 0.5f, BodyType::Dynamic);
        s1->isSensor = true;
        sensorWorld.onContact = [&](const Contact&) { sensorContactFired = true; };

        for (int i = 0; i < 60; ++i) sensorWorld.step(dt);
        float sensorDist = (s2->position - s1->position).length();

        check(sensorContactFired, "sensor overlap still fires World::onContact");
        check(sensorDist < 0.3f, "a sensor contact does not push the overlapping bodies apart");

        World normalWorld;
        normalWorld.gravity = {0.0f, 0.0f};
        Body* n1 = normalWorld.createCircle({-0.1f, 0.0f}, 0.5f, BodyType::Dynamic);
        Body* n2 = normalWorld.createCircle({0.1f, 0.0f}, 0.5f, BodyType::Dynamic);
        for (int i = 0; i < 60; ++i) normalWorld.step(dt);
        float normalDist = (n2->position - n1->position).length();

        check(normalDist > sensorDist, "the same overlap without isSensor separates further than the sensor case");
        check(normalDist >= 0.95f, "non-sensor bodies fully resolve penetration (sum of radii ~= 1.0)");
    }

    if (g_failures == 0) {
        std::printf("\nAll property tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d property test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
