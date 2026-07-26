// Headless checks for Body::matterKind -- the simulation-fidelity dial that
// lets an ordinary rigidbody opt into "Matter"/"OptiMatter" treatment
// without losing any rigidbody capability (rotation, arbitrary shape,
// scripting, UI-Element hosting all keep working; this is purely a solver
// knob, not a different kind of object -- contrast p2d::Matter, the
// separate lightweight particle class covered by matter_test.cpp).

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

    // --- MatterKind::Rigidbody is the default and leaves a Body untouched -
    {
        Body* b = nullptr;
        World world;
        b = world.createCircle({0.0f, 0.0f}, 0.5f);
        check(b->matterKind == MatterKind::Rigidbody, "a freshly created Body defaults to MatterKind::Rigidbody");
    }

    // --- A Body with matterKind = OptiMatter can still rotate/be torqued --
    // (proves this is a solver fidelity flag, not a stripped-down object --
    // unlike p2d::Matter, which structurally has no rotation at all).
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* box = world.createBox({0.0f, 0.0f}, 1.0f, 1.0f, BodyType::Dynamic);
        box->matterKind = MatterKind::OptiMatter;
        Body* bullet = world.createCircle({-3.0f, 0.9f}, 0.3f); // off-center hit
        bullet->velocity = Vec2(20.0f, 0.0f);
        bullet->restitution = 0.3f;

        for (int i = 0; i < 60; ++i) world.step(dt); // 0.5s

        check(std::fabs(box->angularVelocity) > 0.01f,
              "an OptiMatter Body still rotates/gains angular velocity from an off-center hit");
    }

    // --- MatterKind::OptiMatter sleeps sooner than Rigidbody under identical
    // conditions, same World-wide settings -------------------------------
    {
        World world;
        world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);
        Body* rigidOne = world.createCircle({-3.0f, 3.0f}, 0.4f);
        Body* optiOne = world.createCircle({3.0f, 3.0f}, 0.4f);
        optiOne->matterKind = MatterKind::OptiMatter;
        rigidOne->restitution = 0.05f;
        optiOne->restitution = 0.05f;

        int rigidSleepStep = -1;
        int optiSleepStep = -1;
        for (int i = 0; i < 600; ++i) {
            world.step(dt);
            if (rigidSleepStep < 0 && !rigidOne->isAwake) rigidSleepStep = i;
            if (optiSleepStep < 0 && !optiOne->isAwake) optiSleepStep = i;
            if (rigidSleepStep >= 0 && optiSleepStep >= 0) break;
        }

        check(rigidSleepStep >= 0 && optiSleepStep >= 0, "both bodies fall asleep within 5 simulated seconds");
        check(optiSleepStep >= 0 && rigidSleepStep >= 0 && optiSleepStep < rigidSleepStep,
              "the OptiMatter body, dropped identically, falls asleep sooner than the Rigidbody one");
    }

    // --- MatterKind::OptiMatter never forces extra substeps -> tunnels ----
    // through a thin wall under settings that catch an otherwise-identical
    // Rigidbody body (same construction as tests/tunneling_test.cpp).
    {
        auto fireAt = [](World& world, MatterKind kind) {
            world.gravity = {0.0f, 0.0f};
            world.createBox({0.0f, 0.0f}, 0.3f, 3.0f, BodyType::Static); // 0.6m-thick wall
            Body* bullet = world.createCircle({-4.5f, 0.0f}, 0.15f);
            bullet->matterKind = kind;
            bullet->velocity = Vec2(60.0f, 0.0f);
            const float stepDt = 1.0f / 60.0f;
            for (int i = 0; i < 10; ++i) world.step(stepDt);
            return bullet->position.x;
        };

        World rigidWorld;
        check(fireAt(rigidWorld, MatterKind::Rigidbody) < 0.0f,
              "a Rigidbody bullet is caught by a thin wall under default (substepping-enabled) settings");

        World optiWorld;
        check(fireAt(optiWorld, MatterKind::OptiMatter) > 0.0f,
              "an otherwise-identical OptiMatter bullet tunnels through the same wall -- it doesn't force "
              "the substeps that caught the Rigidbody one");
    }

    // --- MatterKind::OptiMatter gets a larger position-correction cap -----
    // Deep overlap (large radius, tiny separation) so the uncapped 20%
    // resolution would exceed BOTH caps, making the two clamped outcomes
    // actually diverge (see matter_test.cpp's Matter-Matter analogue).
    {
        auto overlapResolve = [](World& world, MatterKind kind) {
            world.gravity = {0.0f, 0.0f};
            Body* a = world.createCircle({-0.05f, 0.0f}, 2.5f, BodyType::Static);
            Body* b = world.createCircle({0.05f, 0.0f}, 2.5f, BodyType::Dynamic);
            b->matterKind = kind;
            world.step(1.0f / 120.0f);
            return b->position.x;
        };

        World rigidWorld;
        float rigidX = overlapResolve(rigidWorld, MatterKind::Rigidbody);
        World optiWorld;
        float optiX = overlapResolve(optiWorld, MatterKind::OptiMatter);

        check(optiX > rigidX,
              "an OptiMatter body resolves a deep overlap by a larger step than a Rigidbody under identical "
              "penetration (larger optiMatterMaxLinearCorrection cap)");

        World matterWorld;
        float matterX = overlapResolve(matterWorld, MatterKind::Matter);
        check(matterX < rigidX && matterX > 0.05f,
              "a Matter body resolves the SAME deep overlap by a smaller step than a Rigidbody -- its "
              "tighter matterMaxLinearCorrection cap is more gradual/accurate, not rougher");
    }

    // --- MatterKind::Matter is MORE realistic than Rigidbody, not just a ---
    // label: tighter sleep threshold + longer timeToSleep means a Matter
    // body keeps simulating (stays awake) longer than an identically-dropped
    // Rigidbody one before settling.
    {
        World world;
        world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);
        Body* rigidOne = world.createCircle({-3.0f, 3.0f}, 0.4f);
        Body* matterOne = world.createCircle({3.0f, 3.0f}, 0.4f);
        matterOne->matterKind = MatterKind::Matter;
        rigidOne->restitution = 0.05f;
        matterOne->restitution = 0.05f;

        int rigidSleepStep = -1;
        int matterSleepStep = -1;
        for (int i = 0; i < 1200; ++i) { // up to 10s
            world.step(dt);
            if (rigidSleepStep < 0 && !rigidOne->isAwake) rigidSleepStep = i;
            if (matterSleepStep < 0 && !matterOne->isAwake) matterSleepStep = i;
            if (rigidSleepStep >= 0 && matterSleepStep >= 0) break;
        }

        check(rigidSleepStep >= 0 && matterSleepStep >= 0, "both bodies fall asleep within 10 simulated seconds");
        check(matterSleepStep >= 0 && rigidSleepStep >= 0 && matterSleepStep > rigidSleepStep,
              "a Matter body, dropped identically, stays awake/accurate LONGER than a Rigidbody one before "
              "sleeping");
    }

    // --- MatterKind::Matter forces more substeps than Rigidbody at the -----
    // same speed (tighter matterContinuousDisplacementFraction) -- checked
    // directly via onPreSubstep's per-substep callback count rather than an
    // indirect tunneling scenario, which turned out to depend heavily on
    // World::maxSubsteps (a single World-wide cap both kinds share, so at
    // extreme speeds they saturate to the SAME substep count and a
    // tunneling comparison stops being able to tell them apart at all).
    // Counting substeps directly sidesteps that entirely.
    {
        auto countSubsteps = [](MatterKind kind, float speed) {
            World world;
            world.gravity = {0.0f, 0.0f};
            Body* b = world.createCircle({0.0f, 0.0f}, 0.5f);
            b->matterKind = kind;
            b->velocity = Vec2(speed, 0.0f);
            int count = 0;
            world.onPreSubstep = [&](float) { ++count; };
            world.step(1.0f / 60.0f);
            return count;
        };

        int rigidSubsteps = countSubsteps(MatterKind::Rigidbody, 40.0f);
        int matterSubsteps = countSubsteps(MatterKind::Matter, 40.0f);

        check(rigidSubsteps > 1 && rigidSubsteps < 8,
              "sanity: this speed makes a Rigidbody need multiple substeps without hitting the World-wide cap");
        check(matterSubsteps > rigidSubsteps,
              "an identically-moving Matter body needs strictly MORE substeps than a Rigidbody one -- its "
              "tighter matterContinuousDisplacementFraction demands finer resolution");
    }

    if (g_failures == 0) {
        std::printf("\nAll Body matterKind tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d Body matterKind test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
