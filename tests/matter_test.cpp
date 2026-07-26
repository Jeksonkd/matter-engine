// Headless checks for p2d::Matter -- a point-mass particle, genuinely
// separate from p2d::Body (rigidbody): no rotation/torque/inertia at all,
// always a circle, always simulated. Verifies it actually collides with
// both other Matter and ordinary Body rigidbodies, sleeps, respects its own
// MatterKind fidelity dial (Matter vs OptiMatter), and that mass/density
// behave the same way a Body's do.

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

    // --- computeMass(): mass scales with density * area, same convention as Body
    {
        Matter m;
        m.radius = 0.5f;
        m.density = 2.0f;
        m.computeMass();
        float expected = 2.0f * 3.14159265358979323846f * 0.5f * 0.5f;
        check(std::fabs(m.mass - expected) < 1e-4f, "Matter::computeMass() gives density * circle area");
        check(std::fabs(m.invMass - 1.0f / expected) < 1e-4f, "Matter::computeMass() sets invMass = 1/mass");
    }

    // --- Matter falls under gravity and is caught by a static Body floor --
    {
        World world;
        world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);
        Matter* m = world.createMatter({0.0f, 3.0f}, 0.4f);
        m->restitution = 0.05f;

        for (int i = 0; i < 300; ++i) world.step(dt); // up to 2.5s to fall and settle
        check(m->position.y > 0.3f && m->position.y < 1.0f,
              "a Matter particle dropped onto a static Body floor comes to rest on top of it, not through it");
    }

    // --- Matter falls asleep, same as a Body would ------------------------
    {
        World world;
        world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);
        Matter* m = world.createMatter({0.0f, 3.0f}, 0.4f);
        m->restitution = 0.05f;

        bool slept = false;
        for (int i = 0; i < 600 && !slept; ++i) { // up to 5s
            world.step(dt);
            if (!m->isAwake) slept = true;
        }
        check(slept, "a settled Matter particle falls asleep within 5 simulated seconds");
        check(m->velocity.length() == 0.0f, "a sleeping Matter particle has exactly zero velocity");
    }

    // --- Matter-vs-Matter: two particles collide and separate -------------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Matter* a = world.createMatter({-1.0f, 0.0f}, 0.5f);
        Matter* b = world.createMatter({1.0f, 0.0f}, 0.5f);
        a->velocity = Vec2(5.0f, 0.0f);
        a->restitution = 0.8f;
        b->restitution = 0.8f;

        for (int i = 0; i < 60; ++i) world.step(dt); // 0.5s

        check(b->velocity.x > 0.1f, "an incoming Matter particle transfers momentum to a stationary one it hits");
        check((b->position - a->position).length() >= 0.99f,
              "two colliding Matter particles separate (don't pass through or stay overlapped)");
    }

    // --- Matter-vs-Body: an off-center hit torques the Body, but the ------
    // Matter particle itself has no rotation to gain at all (no such field)
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* box = world.createBox({0.0f, 0.0f}, 1.0f, 1.0f, BodyType::Dynamic);
        // Aimed at the box's upper edge (off-center), not its middle -- a
        // purely central hit wouldn't torque it at all, so this specifically
        // exercises the moment-arm term on the Body's side of the contact.
        Matter* m = world.createMatter({-3.0f, 0.9f}, 0.3f);
        m->velocity = Vec2(20.0f, 0.0f);
        m->restitution = 0.3f;

        for (int i = 0; i < 60; ++i) world.step(dt); // 0.5s

        check(std::fabs(box->angularVelocity) > 0.01f,
              "an off-center Matter impact imparts angular velocity to the Body it hits (moment-arm term works)");
    }

    // --- MatterKind::OptiMatter sleeps sooner than Matter under identical
    // conditions, same World-wide settings -------------------------------
    {
        World world;
        world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);
        Matter* matterOne = world.createMatter({-3.0f, 3.0f}, 0.4f, MatterKind::Matter);
        Matter* optiOne = world.createMatter({3.0f, 3.0f}, 0.4f, MatterKind::OptiMatter);
        matterOne->restitution = 0.05f;
        optiOne->restitution = 0.05f;

        int matterSleepStep = -1;
        int optiSleepStep = -1;
        for (int i = 0; i < 600; ++i) {
            world.step(dt);
            if (matterSleepStep < 0 && !matterOne->isAwake) matterSleepStep = i;
            if (optiSleepStep < 0 && !optiOne->isAwake) optiSleepStep = i;
            if (matterSleepStep >= 0 && optiSleepStep >= 0) break;
        }

        check(matterSleepStep >= 0 && optiSleepStep >= 0, "both particles fall asleep within 5 simulated seconds");
        check(optiSleepStep >= 0 && matterSleepStep >= 0 && optiSleepStep < matterSleepStep,
              "the OptiMatter particle, dropped identically, falls asleep sooner than the Matter one");
    }

    // --- MatterKind::OptiMatter never forces extra substeps -> tunnels ----
    // through a thin wall under settings that catch an otherwise-identical
    // Matter particle (same construction as tests/tunneling_test.cpp).
    {
        auto fireAt = [](World& world, MatterKind kind) {
            world.gravity = {0.0f, 0.0f};
            world.createBox({0.0f, 0.0f}, 0.3f, 3.0f, BodyType::Static); // 0.6m-thick wall
            Matter* bullet = world.createMatter({-4.5f, 0.0f}, 0.15f, kind);
            bullet->velocity = Vec2(60.0f, 0.0f);
            const float stepDt = 1.0f / 60.0f;
            for (int i = 0; i < 10; ++i) world.step(stepDt);
            return bullet->position.x;
        };

        World matterWorld;
        check(fireAt(matterWorld, MatterKind::Matter) < 0.0f,
              "a Matter bullet is caught by a thin wall under default (substepping-enabled) settings");

        World optiWorld;
        check(fireAt(optiWorld, MatterKind::OptiMatter) > 0.0f,
              "an otherwise-identical OptiMatter bullet tunnels through the same wall -- it doesn't force "
              "the substeps that caught the Matter one");
    }

    if (g_failures == 0) {
        std::printf("\nAll Matter tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d Matter test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
