// Verifies the spatial hash grid broadphase produces exactly the same
// candidate pairs as a brute-force O(n^2) AABB scan (the thing it
// replaced), then stress-tests a large, *densely clustered* scene: the
// specific scenario (many bodies genuinely piled up in a small area, not
// just "a lot of bodies") that degrades badly with either a naive O(n^2)
// scan or a poorly-sized broadphase.

#include "p2d/BroadPhase.hpp"
#include "p2d/World.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <utility>
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

// Deterministic pseudo-random float in [0, 1) -- no <random>/Math.random()
// needed, just enough spread to build a varied test scene reproducibly.
float pseudoRandom(int& state) {
    state = state * 1103515245 + 12345;
    return static_cast<float>((state >> 8) & 0xFFFFFF) / static_cast<float>(0x1000000);
}

} // namespace

int main() {
    // --- Equivalence: SAP candidate pairs == brute-force AABB overlap pairs
    {
        std::vector<AABB> boxes;
        int rngState = 12345;
        for (int i = 0; i < 250; ++i) {
            float x = pseudoRandom(rngState) * 40.0f - 20.0f;
            float y = pseudoRandom(rngState) * 40.0f - 20.0f;
            float hw = 0.1f + pseudoRandom(rngState) * 0.9f;
            float hh = 0.1f + pseudoRandom(rngState) * 0.9f;
            boxes.push_back(AABB{Vec2(x - hw, y - hh), Vec2(x + hw, y + hh)});
        }

        std::set<std::pair<int, int>> bruteForcePairs;
        for (size_t i = 0; i < boxes.size(); ++i) {
            for (size_t j = i + 1; j < boxes.size(); ++j) {
                if (collision::aabbOverlap(boxes[i], boxes[j])) {
                    bruteForcePairs.insert({static_cast<int>(i), static_cast<int>(j)});
                }
            }
        }

        std::set<std::pair<int, int>> gridPairs;
        SpatialHashGrid grid;
        grid.computePairs(boxes, 2.0f, [&](int a, int b) {
            if (a > b) std::swap(a, b);
            gridPairs.insert({a, b});
        });

        check(gridPairs == bruteForcePairs,
              "the spatial hash grid finds exactly the same candidate pairs as brute-force AABB overlap");
        check(!bruteForcePairs.empty(), "the random test scene actually has some overlapping pairs to check");
    }

    // --- Equivalence again with a tiny cell size, forcing lots of bodies
    // through the oversized-body path (since many boxes exceed a 0.05 cell)
    // and heavy same-cell/neighbor traffic -- exercises those code paths.
    {
        std::vector<AABB> boxes;
        int rngState = 54321;
        for (int i = 0; i < 150; ++i) {
            float x = pseudoRandom(rngState) * 5.0f - 2.5f;
            float y = pseudoRandom(rngState) * 5.0f - 2.5f;
            float hw = 0.05f + pseudoRandom(rngState) * 0.3f;
            float hh = 0.05f + pseudoRandom(rngState) * 0.3f;
            boxes.push_back(AABB{Vec2(x - hw, y - hh), Vec2(x + hw, y + hh)});
        }

        std::set<std::pair<int, int>> bruteForcePairs;
        for (size_t i = 0; i < boxes.size(); ++i) {
            for (size_t j = i + 1; j < boxes.size(); ++j) {
                if (collision::aabbOverlap(boxes[i], boxes[j])) {
                    bruteForcePairs.insert({static_cast<int>(i), static_cast<int>(j)});
                }
            }
        }

        std::set<std::pair<int, int>> gridPairs;
        SpatialHashGrid grid;
        grid.computePairs(boxes, 0.05f, [&](int a, int b) {
            if (a > b) std::swap(a, b);
            gridPairs.insert({a, b});
        });

        check(gridPairs == bruteForcePairs,
              "equivalence holds with a tiny cell size too (heavy oversized-body + dense-cell traffic)");
    }

    // --- Performance: a large, clustered (worst-case-ish) scene ----------
    // Walled in on both sides so bodies can't be jostled off the edge of an
    // open floor and free-fall forever -- that's not a broadphase problem,
    // it's just "there's no floor there", and it previously masked the real
    // signal by making per-step cost look like it grows without bound (a
    // pile of bodies endlessly falling through open space never stops being
    // "active", which isn't what a settled, contained pile does).
    {
        World world;
        world.gravity = {0.0f, -10.0f};
        world.createBox({0.0f, 0.0f}, 20.0f, 0.5f, BodyType::Static);
        world.createBox({-20.5f, 15.0f}, 0.5f, 15.0f, BodyType::Static);
        world.createBox({20.5f, 15.0f}, 0.5f, 15.0f, BodyType::Static);

        const int bodyCount = 2000;
        int rngState = 999;
        for (int i = 0; i < bodyCount; ++i) {
            // Deliberately clustered (not spread across the whole floor) --
            // the failure mode this broadphase needed to survive.
            float x = -15.0f + pseudoRandom(rngState) * 30.0f;
            float y = 1.0f + static_cast<float>(i) * 0.05f;
            Body* b = world.createCircle({x, y}, 0.1f, BodyType::Dynamic);
            b->restitution = 0.1f;
        }

        const float dt = 1.0f / 60.0f;
        auto timeWindow = [&](int steps) {
            auto t0 = std::chrono::steady_clock::now();
            for (int i = 0; i < steps; ++i) world.step(dt);
            auto t1 = std::chrono::steady_clock::now();
            return std::chrono::duration<double, std::milli>(t1 - t0).count();
        };

        double early = timeWindow(300); // first 5 simulated seconds (settling)
        double later = timeWindow(300); // next 5 simulated seconds (should be settled)

        std::printf("2000-body clustered, contained: first 5s window = %.1fms, later 5s window = %.1fms\n", early,
                    later);

        // The real regression this guards against: per-step cost growing
        // without bound as a dense cluster keeps existing (it should
        // stabilize once bodies settle and sleep, not keep climbing).
        check(later < early * 3.0 + 200.0,
              "per-step cost does not blow up once a dense, contained cluster has had time to settle");
        // Generous absolute bound too -- a smoke check against catastrophic
        // regression, not a tight perf gate (hardware varies).
        check(early < 20000.0 && later < 20000.0,
              "2000-body clustered scene stays within a generous time budget in both windows");
    }

    // --- Performance: a small, stable population that keeps *moving* over
    // a long run (bouncing around, not settling into one place). This is
    // the scenario that actually caught a real regression: a broadphase
    // grid that only clears bucket *contents* per step (to avoid malloc
    // churn) but never removes now-unused cell *keys* accumulates a
    // steadily growing map of stale entries as bodies wander to new cells
    // over time -- even with population flat, since both pair-generation
    // passes iterate every map entry regardless of whether its bucket is
    // still occupied. A bacteria-simulation test doing exactly this went
    // from ~0.5s to ~14s before this was fixed.
    {
        World world;
        world.gravity = {0.0f, 0.0f}; // no settling -- keep everything moving
        world.createBox({0.0f, 0.0f}, 20.0f, 20.0f, BodyType::Static);

        const int bodyCount = 150;
        int rngState = 555;
        std::vector<Body*> bodies;
        for (int i = 0; i < bodyCount; ++i) {
            float x = -18.0f + pseudoRandom(rngState) * 36.0f;
            float y = -18.0f + pseudoRandom(rngState) * 36.0f;
            Body* b = world.createCircle({x, y}, 0.2f, BodyType::Dynamic);
            b->restitution = 1.0f;
            b->linearDamping = 0.0f;
            b->velocity = Vec2((pseudoRandom(rngState) - 0.5f) * 6.0f, (pseudoRandom(rngState) - 0.5f) * 6.0f);
            bodies.push_back(b);
        }

        const float dt = 1.0f / 60.0f;
        auto timeWindow = [&](int steps) {
            auto t0 = std::chrono::steady_clock::now();
            for (int i = 0; i < steps; ++i) world.step(dt);
            auto t1 = std::chrono::steady_clock::now();
            return std::chrono::duration<double, std::milli>(t1 - t0).count();
        };

        // warm up (let the map reach whatever steady-state size it will)
        timeWindow(600); // 10 simulated seconds

        double early = timeWindow(600);  // seconds 10-20
        double later = timeWindow(1200); // seconds 20-40 -- twice as long

        std::printf("150 wandering bodies over 40s: 10-20s window = %.1fms, 20-40s window (2x steps) = %.1fms\n",
                    early, later);

        // later covers 2x the steps of early; if per-step cost were stable
        // (the fix), later should be roughly 2x early, not many times more.
        check(later < early * 2.5 + 100.0,
              "per-step cost stays flat for a small population that keeps wandering over a long run");
    }

    if (g_failures == 0) {
        std::printf("\nAll broadphase tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d broadphase test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
