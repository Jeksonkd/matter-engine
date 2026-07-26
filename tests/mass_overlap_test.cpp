// Verifies a real reported bug: spawning a large batch of bodies directly
// on top of an already-settled large batch (e.g. clicking "Spawn 1000"
// twice without it accounting for what's already there) caused velocities
// to grow *indefinitely* instead of settling -- and got WORSE with MORE
// solver iterations, which is the signature of a genuine correctness bug
// rather than insufficient convergence.
//
// Root cause: World::solveVelocities() computed each contact's restitution
// bias (`pc.velocityBias`) from the bodies' CURRENT relative velocity, but
// warm-started impulses for EARLIER contacts in the same pass were already
// being applied before LATER contacts computed their own bias -- fine for
// a handful of simultaneous contacts (an ordinary stack), but for a body
// touching many neighbors at once (this scenario), that interleaving fed a
// feedback loop: contaminated relative velocity -> spurious restitution
// bias -> a bigger warm-started impulse carried into next step -> more
// contamination. Fixed by computing every contact's velocityBias in one
// pass, strictly before applying any warm-started impulses in a second
// pass (see solveVelocities()'s comments).

#include "p2d/World.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

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

// Deterministic pseudo-random float in [0, 1) -- same generator used by
// broadphase_test.cpp, for a reproducible but varied scene.
float pseudoRandom(int& state) {
    state = state * 1103515245 + 12345;
    return static_cast<float>((state >> 8) & 0xFFFFFF) / static_cast<float>(0x1000000);
}

std::vector<Body*> spawnGrid(World& world, int cols, int rows, float spacing, float startY, int& rngState) {
    std::vector<Body*> bodies;
    bodies.reserve(static_cast<size_t>(cols) * static_cast<size_t>(rows));
    float startX = -(static_cast<float>(cols) - 1.0f) * spacing * 0.5f;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            float x = startX + static_cast<float>(col) * spacing + (pseudoRandom(rngState) - 0.5f) * 0.05f;
            float y = startY + static_cast<float>(row) * spacing + (pseudoRandom(rngState) - 0.5f) * 0.05f;
            Body* c = world.createCircle({x, y}, 0.15f, BodyType::Dynamic);
            c->restitution = 0.2f;
            c->friction = 0.4f;
            bodies.push_back(c);
        }
    }
    return bodies;
}

float maxSpeed(const std::vector<Body*>& bodies) {
    float m = 0.0f;
    for (Body* b : bodies) m = std::max(m, b->velocity.length());
    return m;
}

} // namespace

int main() {
    World world;
    world.gravity = {0.0f, -9.81f};
    world.createBox({0.0f, 0.0f}, 20.0f, 0.5f, BodyType::Static);
    world.createBox({-20.5f, 15.0f}, 0.5f, 15.0f, BodyType::Static);
    world.createBox({20.5f, 15.0f}, 0.5f, 15.0f, BodyType::Static);

    const float dt = 1.0f / 60.0f;
    int rngState = 12345;

    // Batch 1: let it fall and settle into a dense pile.
    std::vector<Body*> batch1 = spawnGrid(world, 40, 25, 0.35f, 10.0f, rngState);
    for (int i = 0; i < 600; ++i) world.step(dt); // 10 simulated seconds

    check(maxSpeed(batch1) < 2.0f, "batch 1 settles to a low velocity on its own before batch 2 spawns");

    // Batch 2: spawned at the SAME grid coordinates batch 1 started at --
    // i.e. directly overlapping wherever batch 1 has since settled,
    // reproducing "Spawn 1000" clicked again without checking what's
    // already in the scene.
    std::vector<Body*> batch2 = spawnGrid(world, 40, 25, 0.35f, 10.0f, rngState);

    std::vector<Body*> all = batch1;
    all.insert(all.end(), batch2.begin(), batch2.end());

    bool stayedFinite = true;
    float earlyPeakSpeed = 0.0f;  // within the first 2 simulated seconds after the overlap
    for (int i = 0; i < 120; ++i) {
        world.step(dt);
        for (Body* b : all) {
            if (!std::isfinite(b->position.x) || !std::isfinite(b->position.y)) stayedFinite = false;
        }
        earlyPeakSpeed = std::max(earlyPeakSpeed, maxSpeed(all));
    }
    check(stayedFinite, "positions stay finite throughout the overlapping spawn and its aftermath");
    std::printf("peak speed in the first 2s after the overlapping spawn: %.2f m/s\n", earlyPeakSpeed);

    // The actual bug: this used to keep climbing indefinitely (still
    // ~20-25 m/s and rising after several more seconds). A brief, bounded
    // burst while genuinely-overlapping bodies find room is expected and
    // fine; what must NOT happen is the speed still being this high (or
    // higher) many seconds later.
    for (int i = 0; i < 1080; ++i) world.step(dt); // 18 more simulated seconds
    float lateSpeed = maxSpeed(all);
    std::printf("max speed 20s after the overlapping spawn: %.2f m/s\n", lateSpeed);
    check(lateSpeed < 1.5f, "velocity decays back down and settles, rather than growing indefinitely");
    check(lateSpeed <= earlyPeakSpeed,
          "velocity trends downward from its post-spawn peak, not upward (the bug's exact signature)");

    if (g_failures == 0) {
        std::printf("\nAll mass-overlap tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d mass-overlap test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
