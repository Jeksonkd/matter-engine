// Verifies the fix for a real reported bug: bodies spawned deeply
// overlapping another body (e.g. a script placing shapes without checking
// what's already there) used to separate almost instantly -- correctly
// bounded in principle (a fixed 20% of the remaining penetration is
// resolved per step), but with no cap on the ABSOLUTE amount, 20% of a
// large initial penetration is still large, and with up to 8 substeps
// running per rendered frame in full-accuracy mode, that fraction compounds
// within a single visible frame into what looks like an explosion. See
// World::maxLinearCorrection's doc comment and correctPositions().

#include "p2d/World.hpp"

#include <algorithm>
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
    World world;
    world.gravity = {0.0f, 0.0f}; // isolate from gravity -- any motion here is purely separation

    // Two large, same-size, dead-centered circles: the deepest possible
    // overlap for this shape (penetration == the full combined radius).
    Body* a = world.createCircle({0.0f, 0.0f}, 2.0f, BodyType::Dynamic);
    Body* b = world.createCircle({0.0f, 0.0f}, 2.0f, BodyType::Dynamic);

    const float dt = 1.0f / 60.0f;
    float prevSeparation = 0.0f;
    float maxSeparationDeltaInOneStep = 0.0f;
    float maxSpeedSeen = 0.0f;

    for (int i = 0; i < 180; ++i) { // 3 simulated seconds
        world.step(dt);

        float separation = (b->position - a->position).length();
        maxSeparationDeltaInOneStep = std::max(maxSeparationDeltaInOneStep, separation - prevSeparation);
        prevSeparation = separation;

        maxSpeedSeen = std::max({maxSpeedSeen, a->velocity.length(), b->velocity.length()});
    }

    std::printf("final separation: %.3f (combined radius: 4.0)\n", prevSeparation);
    check(prevSeparation > 3.5f,
          "two fully-overlapping circles do eventually separate to about their combined radius");

    // The actual fix: no single step should resolve a large chunk of the
    // overlap at once. Without the cap, the first step alone would have
    // resolved 20% of ~4.0m = ~0.8m; with it, every step is bounded at
    // World::maxLinearCorrection (0.2m default) -- checked with a small
    // margin rather than the exact constant, so this doesn't over-couple
    // the test to one specific tuning value.
    std::printf("largest single-step separation jump: %.4f m\n", maxSeparationDeltaInOneStep);
    check(maxSeparationDeltaInOneStep < 0.25f,
          "even fully-overlapping bodies separate gradually, not in one explosive snap");

    // Positional correction is a direct position nudge, not a velocity
    // change (see correctPositions()'s comment) -- with zero gravity and
    // zero initial relative velocity, nothing here should ever accelerate
    // either body. A nonzero velocity would mean the "explosion" is
    // leaking into the velocity solver too, not just position.
    std::printf("max speed observed: %.4f m/s\n", maxSpeedSeen);
    check(maxSpeedSeen < 0.05f, "separating deeply-overlapping bodies doesn't inject velocity, just position");

    check(std::isfinite(a->position.x) && std::isfinite(b->position.x),
          "positions stay finite throughout the separation");

    if (g_failures == 0) {
        std::printf("\nAll spawn-overlap tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d spawn-overlap test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
