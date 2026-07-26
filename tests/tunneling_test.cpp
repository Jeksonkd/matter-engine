// Headless check that adaptive substepping actually prevents tunneling:
// a fast bullet fired at a thin wall must be caught when substepping is
// enabled (the default), and -- to prove the test itself is meaningful --
// must actually tunnel through when substepping is disabled.

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

// Fires a fast circle at a thin static wall and returns its final x
// position. Per-call World::step(dt) displacement is exactly 1.0m
// (velocity 60 * dt 1/60), chosen together with the start position so the
// *un-substepped* sample points (-4.5, -3.5, ..., -0.5, +0.5, ...) never
// land inside the wall's [-0.45, 0.45] combined-with-bullet-radius overlap
// region -- i.e. disabling substepping is guaranteed to tunnel by
// construction, not by luck.
float fireBulletAt(World& world) {
    world.gravity = {0.0f, 0.0f};
    world.createBox({0.0f, 0.0f}, 0.3f, 3.0f, BodyType::Static); // 0.6m-thick wall
    Body* bullet = world.createCircle({-4.5f, 0.0f}, 0.15f, BodyType::Dynamic);
    bullet->velocity = Vec2(60.0f, 0.0f);

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 10; ++i) world.step(dt);
    return bullet->position.x;
}

} // namespace

int main() {
    World substepped;
    float finalXWithSubsteps = fireBulletAt(substepped);
    check(finalXWithSubsteps < 0.0f,
          "with adaptive substepping enabled (default), a fast bullet is caught by a thin wall, not tunneling through");

    World coarse;
    coarse.continuousDisplacementFraction = 1.0e9f; // effectively forces substeps = 1
    float finalXCoarse = fireBulletAt(coarse);
    check(finalXCoarse > 0.0f,
          "confirms the test scenario: with substepping disabled, the same bullet tunnels straight through");

    if (g_failures == 0) {
        std::printf("\nAll tunneling tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d tunneling test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
