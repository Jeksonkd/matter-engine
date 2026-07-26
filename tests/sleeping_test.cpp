// Headless checks for sleeping bodies: a settled body should actually go to
// sleep (stop being simulated), stay perfectly still while asleep, and wake
// up both from an explicit force and from being hit by another body.

#include "p2d/World.hpp"

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

    // --- A settled body falls asleep, stays put, and wakes on force ------
    {
        World world;
        Body* ground = world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);
        Body* ball = world.createCircle({0.0f, 3.0f}, 0.5f, BodyType::Dynamic);
        ball->restitution = 0.05f;

        bool wentToSleep = false;
        Vec2 posAtSleep;
        for (int i = 0; i < 600 && !wentToSleep; ++i) { // up to 5s
            world.step(dt);
            if (!ball->isAwake) {
                wentToSleep = true;
                posAtSleep = ball->position;
            }
        }
        check(wentToSleep, "a settled ball falls asleep within 5 simulated seconds");
        check(ball->velocity.length() == 0.0f, "a sleeping body has exactly zero velocity");
        check(ball->angularVelocity == 0.0f, "a sleeping body has exactly zero angular velocity");

        for (int i = 0; i < 120; ++i) world.step(dt); // 1 more second
        check((ball->position - posAtSleep).length() < 1e-5f,
              "a sleeping body's position does not drift while asleep");
        check(!ball->isAwake, "the body is still asleep after more steps with nothing disturbing it");

        ball->applyForce(Vec2(0.0f, 100.0f));
        check(ball->isAwake, "applyForce() wakes a sleeping body immediately");
    }

    // --- A sleeping body wakes when something falls on it -----------------
    {
        World world;
        world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);
        Body* resting = world.createCircle({0.0f, 3.0f}, 0.5f, BodyType::Dynamic);
        resting->restitution = 0.05f;

        bool slept = false;
        for (int i = 0; i < 600 && !slept; ++i) {
            world.step(dt);
            if (!resting->isAwake) slept = true;
        }
        check(slept, "the resting ball falls asleep before the incoming ball is dropped");

        Body* incoming = world.createCircle({0.05f, 6.0f}, 0.5f, BodyType::Dynamic);
        incoming->restitution = 0.05f;

        bool woke = false;
        for (int i = 0; i < 300 && !woke; ++i) { // up to 2.5s to fall and land
            world.step(dt);
            if (resting->isAwake) woke = true;
        }
        check(woke, "the sleeping ball wakes up when the incoming ball lands on it");
    }

    if (g_failures == 0) {
        std::printf("\nAll sleeping tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d sleeping test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
