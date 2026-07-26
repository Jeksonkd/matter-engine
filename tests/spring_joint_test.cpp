// Verifies p2d::SpringJoint: a general-purpose spring connecting any two
// existing bodies (not an auto-generated particle cluster like SoftBody) --
// this is what lets a script or the editor "tie objects together" while
// each object stays itself (its own shape, its own BodyType).

#include "p2d/SpringJoint.hpp"
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
    // --- A spring pulls two different-shaped bodies toward its rest length
    {
        World world;
        world.gravity = {0.0f, 0.0f};

        Body* box = world.createBox({-3.0f, 0.0f}, 0.5f, 0.5f, BodyType::Dynamic);
        Body* circle = world.createCircle({3.0f, 0.0f}, 0.4f, BodyType::Dynamic);
        box->name = "anchor_box";
        circle->name = "anchor_circle";

        SpringJoint joint{box, circle, 2.0f, 40.0f, 3.0f}; // rest length 2.0, currently 6.0 apart
        world.onPreSubstep = [&joint](float) { joint.applyForce(); };

        const float dt = 1.0f / 60.0f;
        bool stayedFinite = true;
        for (int i = 0; i < 300; ++i) { // 5 simulated seconds
            world.step(dt);
            if (!std::isfinite(box->position.x) || !std::isfinite(circle->position.x)) {
                stayedFinite = false;
                break;
            }
        }
        check(stayedFinite, "a spring joint between a box and a circle never produces NaN/inf");

        float finalDist = (circle->position - box->position).length();
        std::printf("final distance: %.3f (rest length 2.0, started at 6.0)\n", finalDist);
        check(std::fabs(finalDist - 2.0f) < 0.3f,
              "the spring pulls two different-shaped bodies to roughly its rest length");

        check(box->shape.type == ShapeType::Polygon && circle->shape.type == ShapeType::Circle,
              "connecting two bodies with a spring doesn't change either one's own shape");
    }

    // --- A spring can hold a body up against gravity (like a cloth's pin,
    // but between two ordinary bodies instead of a generated particle grid)
    {
        World world;
        world.gravity = {0.0f, -10.0f};

        Body* anchor = world.createBox({0.0f, 5.0f}, 0.3f, 0.3f, BodyType::Static);
        Body* hanging = world.createCircle({0.0f, 3.0f}, 0.3f, BodyType::Dynamic);
        anchor->name = "anchor";
        hanging->name = "hanging";

        SpringJoint joint{anchor, hanging, 2.0f, 60.0f, 4.0f};
        world.onPreSubstep = [&joint](float) { joint.applyForce(); };

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 600; ++i) world.step(dt); // 10 simulated seconds to settle

        check(std::isfinite(hanging->position.y), "a gravity-loaded spring joint stays finite while settling");
        float finalDist = (hanging->position - anchor->position).length();
        std::printf("hanging body settled %.3f from its Static anchor (rest length 2.0)\n", finalDist);
        // Under gravity a spring settles somewhat stretched past its rest
        // length (the spring force balances gravity at equilibrium, not at
        // exactly rest length) -- just check it's in a sane bounded range,
        // not still falling freely or flung away.
        check(finalDist > 1.5f && finalDist < 4.0f,
              "the hanging body settles a bounded distance from its anchor, not falling freely or flung away");
        check(anchor->position.y > 4.9f, "the Static anchor itself never moves");

        float speed = hanging->velocity.length();
        std::printf("hanging body speed after settling: %.3f m/s\n", speed);
        check(speed < 1.0f, "the hanging body comes to rest rather than oscillating forever");
    }

    if (g_failures == 0) {
        std::printf("\nAll spring joint tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d spring joint test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
