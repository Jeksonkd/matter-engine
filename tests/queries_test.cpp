// Headless checks for World::raycastClosest()/raycastAll()/queryPoint() and
// Body/Matter collision filtering (collisionCategory/collisionMask).

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
    // --- Raycast vs a circle ------------------------------------------------
    {
        World world;
        Body* circle = world.createCircle({5.0f, 0.0f}, 1.0f, BodyType::Static);

        World::RaycastResult hit;
        bool got = world.raycastClosest({0.0f, 0.0f}, {10.0f, 0.0f}, hit);
        check(got, "raycastClosest hits a circle directly in the ray's path");
        check(hit.body == circle, "raycastClosest reports the correct body");
        check(std::fabs(hit.point.x - 4.0f) < 1e-3f, "raycastClosest hits the circle's near edge (x=4)");
        check(std::fabs(hit.normal.x - (-1.0f)) < 1e-3f, "raycastClosest's normal faces back toward the ray origin");
        check(hit.fraction > 0.0f && hit.fraction < 1.0f, "raycastClosest's fraction is between 0 and 1");
    }

    // --- Raycast that starts inside a shape reports no hit for it ----------
    {
        World world;
        world.createCircle({0.0f, 0.0f}, 1.0f, BodyType::Static);
        World::RaycastResult hit;
        bool got = world.raycastClosest({0.0f, 0.0f}, {10.0f, 0.0f}, hit);
        check(!got, "a ray starting inside a circle does not report a hit against it");
    }

    // --- Raycast vs a box (polygon) ------------------------------------------
    {
        World world;
        Body* box = world.createBox({5.0f, 0.0f}, 1.0f, 1.0f, BodyType::Static);
        World::RaycastResult hit;
        bool got = world.raycastClosest({0.0f, 0.0f}, {10.0f, 0.0f}, hit);
        check(got && hit.body == box, "raycastClosest hits a box directly in the ray's path");
        check(std::fabs(hit.point.x - 4.0f) < 1e-3f, "raycastClosest hits the box's near face (x=4)");
        check(std::fabs(hit.normal.x - (-1.0f)) < 1e-3f, "raycastClosest's normal faces back out of the box's face");
    }

    // --- Raycast picks the CLOSEST of several hits --------------------------
    {
        World world;
        Body* nearBody = world.createCircle({3.0f, 0.0f}, 0.5f, BodyType::Static);
        world.createCircle({6.0f, 0.0f}, 0.5f, BodyType::Static);
        World::RaycastResult hit;
        bool got = world.raycastClosest({0.0f, 0.0f}, {10.0f, 0.0f}, hit);
        check(got && hit.body == nearBody, "raycastClosest reports the nearer of two hits, not the farther");
    }

    // --- raycastAll returns every hit, sorted by fraction -------------------
    {
        World world;
        Body* far = world.createCircle({8.0f, 0.0f}, 0.5f, BodyType::Static);
        Body* near = world.createCircle({3.0f, 0.0f}, 0.5f, BodyType::Static);
        auto hits = world.raycastAll({0.0f, 0.0f}, {10.0f, 0.0f});
        check(hits.size() == 2, "raycastAll finds both bodies along the ray");
        if (hits.size() == 2) {
            check(hits[0].body == near && hits[1].body == far,
                  "raycastAll sorts hits by fraction, nearest first");
        }
    }

    // --- Raycast misses something off to the side ---------------------------
    {
        World world;
        world.createCircle({5.0f, 5.0f}, 1.0f, BodyType::Static); // well off the ray's path
        World::RaycastResult hit;
        bool got = world.raycastClosest({0.0f, 0.0f}, {10.0f, 0.0f}, hit);
        check(!got, "raycastClosest reports no hit for a body off to the side of the ray");
    }

    // --- Raycast + Matter particle ------------------------------------------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Matter* m = world.createMatter({5.0f, 0.0f}, 0.5f);
        World::RaycastResult hit;
        bool got = world.raycastClosest({0.0f, 0.0f}, {10.0f, 0.0f}, hit);
        check(got && hit.matter == m && hit.body == nullptr, "raycastClosest can hit a Matter particle too");
    }

    // --- queryPoint ----------------------------------------------------------
    {
        World world;
        Body* box = world.createBox({0.0f, 0.0f}, 1.0f, 1.0f, BodyType::Static);
        check(world.queryPoint({0.0f, 0.0f}) == box, "queryPoint finds the box containing the origin");
        check(world.queryPoint({5.0f, 5.0f}) == nullptr, "queryPoint returns nullptr where nothing is");
    }
    {
        World world;
        world.createCircle({0.0f, 0.0f}, 1.0f, BodyType::Static)->name = "bottom";
        Body* top = world.createCircle({0.0f, 0.0f}, 1.0f, BodyType::Static);
        top->name = "top";
        check(world.queryPoint({0.0f, 0.0f}) == top,
              "queryPoint returns the topmost (last-created) match when shapes overlap");
    }

    // --- Collision filtering: Body vs Body ----------------------------------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({-1.0f, 0.0f}, 0.6f, BodyType::Dynamic);
        Body* b = world.createCircle({1.0f, 0.0f}, 0.6f, BodyType::Dynamic);
        a->collisionCategory = 0x0002;
        a->collisionMask = 0x0002; // only collides with its own category
        b->collisionCategory = 0x0001;
        b->collisionMask = 0x0001;
        a->velocity = Vec2(5.0f, 0.0f);

        for (int i = 0; i < 30; ++i) world.step(1.0f / 60.0f); // 0.5s, should pass straight through

        check(a->position.x > 1.5f, "filtered-out bodies pass through each other instead of colliding");
    }
    {
        // Same setup, but WITHOUT filtering -- sanity check they normally would collide.
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({-1.0f, 0.0f}, 0.6f, BodyType::Dynamic);
        world.createCircle({1.0f, 0.0f}, 0.6f, BodyType::Dynamic);
        a->velocity = Vec2(5.0f, 0.0f);

        for (int i = 0; i < 30; ++i) world.step(1.0f / 60.0f);

        check(a->position.x < 1.5f, "sanity: without filtering, the same two bodies DO collide/deflect");
    }

    // --- Collision filtering: Matter vs Body --------------------------------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Matter* m = world.createMatter({-1.0f, 0.0f}, 0.4f);
        Body* wall = world.createBox({1.0f, 0.0f}, 0.3f, 3.0f, BodyType::Static);
        m->collisionCategory = 0x0002;
        wall->collisionMask = 0x0001; // wall no longer accepts category 0x0002
        m->velocity = Vec2(5.0f, 0.0f);

        for (int i = 0; i < 30; ++i) world.step(1.0f / 60.0f);

        // Wall spans x in [0.7, 1.3] (halfWidth 0.3 at x=1); starting at
        // x=-1 with vx=5 for 0.5s reaches x=1.5 unobstructed -- past the
        // wall's far face, which a filtered (pass-through) particle should
        // reach with room to spare.
        check(m->position.x > 1.4f, "a filtered-out Matter particle passes straight through a Body wall");
    }

    // --- Raycast respects the mask parameter --------------------------------
    {
        World world;
        Body* body = world.createCircle({5.0f, 0.0f}, 1.0f, BodyType::Static);
        body->collisionCategory = 0x0002;
        World::RaycastResult hit;
        bool gotFiltered = world.raycastClosest({0.0f, 0.0f}, {10.0f, 0.0f}, hit, 0x0001);
        check(!gotFiltered, "raycastClosest's mask parameter filters out a non-matching category");
        bool gotUnfiltered = world.raycastClosest({0.0f, 0.0f}, {10.0f, 0.0f}, hit, 0x0002);
        check(gotUnfiltered, "raycastClosest's mask parameter still allows a matching category through");
    }

    if (g_failures == 0) {
        std::printf("\nAll query/filtering tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d query/filtering test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
