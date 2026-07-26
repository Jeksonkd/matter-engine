// Headless check that a stack of boxes settles cleanly: minimal
// interpenetration/sinking and no sideways sliding. This exercises the
// whole rewritten solver together (warm-starting, fixed-tangent friction,
// accumulated/clamped impulses) -- the classic failure mode of a naive
// impulse solver is a stack that either sinks noticeably into itself or
// never quite stops jittering/sliding.

#include "p2d/World.hpp"

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

} // namespace

int main() {
    World world;
    Body* ground = world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);
    ground->name = "ground";

    const int stackHeight = 6;
    std::vector<Body*> boxes;
    for (int i = 0; i < stackHeight; ++i) {
        // A small starting gap avoids a fully-overlapping spawn; the solver
        // should still pull them into a tight, stable stack.
        Body* b = world.createBox({0.0f, 1.0f + static_cast<float>(i) * 1.02f}, 0.5f, 0.5f, BodyType::Dynamic);
        b->friction = 0.6f;
        b->restitution = 0.0f;
        boxes.push_back(b);
    }

    const float dt = 1.0f / 120.0f;
    for (int i = 0; i < 600; ++i) world.step(dt); // 5 simulated seconds to settle

    for (int i = 0; i < stackHeight; ++i) {
        Body& b = *boxes[static_cast<size_t>(i)];
        float expectedY = 1.0f + static_cast<float>(i) * 1.0f;

        char msg[128];
        std::snprintf(msg, sizeof(msg), "box %d settles at its expected stacked height (no sinking)", i);
        check(std::fabs(b.position.y - expectedY) < 0.08f, msg);

        std::snprintf(msg, sizeof(msg), "box %d does not slide sideways", i);
        check(std::fabs(b.position.x) < 0.1f, msg);

        std::snprintf(msg, sizeof(msg), "box %d is at rest (or asleep)", i);
        check(b.velocity.length() < 0.05f, msg);

        std::snprintf(msg, sizeof(msg), "box %d has not been knocked over (near-zero rotation)", i);
        check(std::fabs(b.rotation) < 0.1f, msg);
    }

    if (g_failures == 0) {
        std::printf("\nAll stacking tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d stacking test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
