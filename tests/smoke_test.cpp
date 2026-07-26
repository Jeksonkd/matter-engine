// Headless correctness smoke test for the p2d physics engine. No graphics
// dependencies -- exercises circle-circle, circle-polygon and
// polygon-polygon collision plus the impulse solver, and fails loudly
// (non-zero exit code) if anything behaves unphysically.

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

bool isFinite(const Vec2& v) { return std::isfinite(v.x) && std::isfinite(v.y); }

} // namespace

int main() {
    // --- Test 1: circle settling on a static box ground ------------------
    {
        World world;
        Body* ground = world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);
        ground->name = "ground";

        Body* ball = world.createCircle({0.0f, 5.0f}, 0.5f, BodyType::Dynamic);
        ball->name = "ball";
        ball->restitution = 0.1f;
        ball->friction = 0.4f;

        const float dt = 1.0f / 120.0f;
        for (int i = 0; i < 960; ++i) world.step(dt);

        check(isFinite(ball->position) && isFinite(ball->velocity), "circle: finite state after simulation");
        check(std::fabs(ball->position.y - 1.0f) < 0.05f,
              "circle: settles to rest on top of the ground (y ~= radius + groundTop)");
        check(ball->velocity.length() < 0.1f, "circle: comes to rest (low residual velocity)");
        check(std::fabs(ball->position.x) < 0.2f, "circle: does not drift sideways with no horizontal force");
    }

    // --- Test 2: box dropped with initial spin settles flat ---------------
    {
        World world;
        Body* ground = world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);
        ground->name = "ground";

        Body* box = world.createBox({3.0f, 5.0f}, 0.5f, 0.5f, BodyType::Dynamic);
        box->name = "box";
        box->rotation = 0.35f;
        box->restitution = 0.1f;
        box->friction = 0.5f;

        const float dt = 1.0f / 120.0f;
        for (int i = 0; i < 1200; ++i) world.step(dt);

        check(isFinite(box->position) && isFinite(box->velocity), "box: finite state after simulation");
        check(std::fabs(box->position.y - 1.0f) < 0.08f,
              "box: settles flat on the ground (y ~= halfHeight + groundTop)");
        check(box->velocity.length() < 0.1f, "box: comes to rest (low residual velocity)");
        check(std::fabs(box->angularVelocity) < 0.1f, "box: stops spinning");
    }

    // --- Test 3: circle-circle collision does not tunnel/explode ----------
    {
        World world;
        world.gravity = {0.0f, 0.0f};

        Body* a = world.createCircle({-2.0f, 0.0f}, 0.5f, BodyType::Dynamic);
        Body* b = world.createCircle({2.0f, 0.0f}, 0.5f, BodyType::Dynamic);
        a->restitution = b->restitution = 0.8f;
        a->velocity = {4.0f, 0.0f};
        b->velocity = {-4.0f, 0.0f};

        const float dt = 1.0f / 120.0f;
        for (int i = 0; i < 240; ++i) world.step(dt);

        float dist = (b->position - a->position).length();
        check(isFinite(a->position) && isFinite(b->position), "circle-circle: finite state after collision");
        check(dist >= 0.95f, "circle-circle: bodies do not interpenetrate after bouncing apart");
        check(a->velocity.x < 0.0f && b->velocity.x > 0.0f, "circle-circle: bodies bounced apart (reversed direction)");
    }

    // --- Test 4: polygon-polygon (two boxes) does not tunnel/explode ------
    {
        World world;
        world.gravity = {0.0f, 0.0f};

        Body* a = world.createBox({-1.5f, 0.0f}, 0.5f, 0.5f, BodyType::Dynamic);
        Body* b = world.createBox({1.5f, 0.0f}, 0.5f, 0.5f, BodyType::Dynamic);
        a->velocity = {3.0f, 0.0f};
        b->velocity = {-3.0f, 0.0f};

        const float dt = 1.0f / 120.0f;
        for (int i = 0; i < 240; ++i) world.step(dt);

        float dist = (b->position - a->position).length();
        check(isFinite(a->position) && isFinite(b->position), "box-box: finite state after collision");
        check(dist >= 0.9f, "box-box: bodies do not interpenetrate after colliding");
    }

    if (g_failures == 0) {
        std::printf("\nAll smoke tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d smoke test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
