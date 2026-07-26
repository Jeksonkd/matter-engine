// Headless checks for World::enableCcd -- the opt-in swept-circle
// time-of-impact continuous collision detection layered on top of adaptive
// substepping (see World.hpp's doc comment for the full picture).

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

// Same construction as tests/tunneling_test.cpp's fireBulletAt(): a fast
// circle vs. a thin wall, at a speed/wall-thickness combination chosen so
// it tunnels through under ordinary (non-CCD) settings.
float fireBulletAt(World& world, bool ccd, MatterKind kind = MatterKind::Rigidbody) {
    world.gravity = {0.0f, 0.0f};
    world.enableCcd = ccd;
    world.createBox({0.0f, 0.0f}, 0.3f, 3.0f, BodyType::Static); // 0.6m-thick wall
    Body* bullet = world.createCircle({-4.5f, 0.0f}, 0.15f, BodyType::Dynamic);
    bullet->matterKind = kind;
    bullet->velocity = Vec2(1500.0f, 0.0f); // fast enough to outrun maxSubsteps entirely
    const float stepDt = 1.0f / 60.0f;
    for (int i = 0; i < 5; ++i) world.step(stepDt);
    return bullet->position.x;
}

} // namespace

int main() {
    // --- Sanity: this speed tunnels through the wall by default -----------
    {
        World world;
        float x = fireBulletAt(world, /*ccd=*/false);
        check(x > 0.0f, "sanity: at this extreme speed, the bullet tunnels through the wall by default");
    }

    // --- enableCcd catches the exact case the sanity check just proved ----
    {
        World world;
        float x = fireBulletAt(world, /*ccd=*/true);
        check(x < 0.0f, "World::enableCcd catches a bullet that would otherwise tunnel clean through");
    }

    // --- OptiMatter is exempt from CCD even when it's turned on -----------
    {
        World world;
        float x = fireBulletAt(world, /*ccd=*/true, MatterKind::OptiMatter);
        check(x > 0.0f, "an OptiMatter bullet still tunnels with CCD on -- it's exempt, same as forced substeps");
    }

    // --- CCD also works for circle-vs-circle, not just circle-vs-polygon --
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        world.enableCcd = true;
        world.createCircle({0.0f, 0.0f}, 0.5f, BodyType::Static); // a "wall" made of one big circle
        Body* bullet = world.createCircle({-4.5f, 0.0f}, 0.1f, BodyType::Dynamic);
        bullet->velocity = Vec2(1500.0f, 0.0f);
        for (int i = 0; i < 5; ++i) world.step(1.0f / 60.0f);
        check(bullet->position.x < 0.0f, "CCD also catches a fast circle vs. a circular target");
    }

    // --- CCD works for Matter particles too, not just Body circles --------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        world.enableCcd = true;
        world.createBox({0.0f, 0.0f}, 0.3f, 3.0f, BodyType::Static);
        Matter* bullet = world.createMatter({-4.5f, 0.0f}, 0.15f, MatterKind::Matter);
        bullet->velocity = Vec2(1500.0f, 0.0f);
        for (int i = 0; i < 5; ++i) world.step(1.0f / 60.0f);
        check(bullet->position.x < 0.0f, "CCD also catches a fast Matter particle, not just Body circles");
    }

    // --- Disabled by default -- an ordinary World never applies it --------
    {
        World world;
        check(!world.enableCcd, "World::enableCcd is false by default (opt-in, not automatic)");
    }

    // --- Collision filtering is still respected during a CCD sweep --------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        world.enableCcd = true;
        Body* wall = world.createBox({0.0f, 0.0f}, 0.3f, 3.0f, BodyType::Static);
        wall->collisionMask = 0x0001;
        Body* bullet = world.createCircle({-4.5f, 0.0f}, 0.15f, BodyType::Dynamic);
        bullet->collisionCategory = 0x0002; // wall's mask doesn't include this category
        bullet->velocity = Vec2(1500.0f, 0.0f);
        for (int i = 0; i < 5; ++i) world.step(1.0f / 60.0f);
        check(bullet->position.x > 0.0f, "a CCD sweep still respects collision filtering -- a filtered pair passes through");
    }

    if (g_failures == 0) {
        std::printf("\nAll CCD tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d CCD test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
