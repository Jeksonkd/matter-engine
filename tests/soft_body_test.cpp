// Verifies the new SoftBody helper (mass-spring ring of particle Bodies,
// see p2d/SoftBody.hpp): it should settle onto a floor without exploding,
// keep roughly its ring shape (not collapse flat, not fly apart), and every
// particle position/velocity should stay finite throughout. Also checks
// that a regular polygon (triangle) built via ShapeData::MakePolygon --
// the same construction the editor's unified Spawn tool now uses for
// Triangle/Pentagon/Hexagon -- collides and settles normally, since that
// path previously only ever got exercised via the box-specific helper.

#include "p2d/SoftBody.hpp"
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

bool allFinite(const World& world) {
    for (auto& b : world.bodies()) {
        if (!std::isfinite(b->position.x) || !std::isfinite(b->position.y)) return false;
        if (!std::isfinite(b->velocity.x) || !std::isfinite(b->velocity.y)) return false;
    }
    return true;
}

} // namespace

int main() {
    // --- Soft body ring settles on a floor without exploding -------------
    {
        World world;
        world.gravity = {0.0f, -10.0f};
        world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);

        const float ringRadius = 1.0f;
        SoftBody sb = makeSoftBodyRing(world, {0.0f, 5.0f}, ringRadius, 10);
        check(sb.particles.size() == 10, "makeSoftBodyRing creates the requested particle count");
        check(!sb.springs.empty(), "makeSoftBodyRing creates spring constraints between particles");

        // Spring forces MUST be recomputed every internal substep, not just
        // once before world.step() -- see World::onPreSubstep's doc comment.
        // An earlier version of this test called sb.applySpringForces()
        // once per outer step() call instead and caught a real bug: once
        // particles moved fast enough to trigger step()'s internal
        // substepping, the spring force (set once, consumed on only the
        // first substep) was silently under-applied on every later
        // substep, and the ring gained energy indefinitely instead of
        // settling.
        world.onPreSubstep = [&sb](float) { sb.applySpringForces(); };

        const float dt = 1.0f / 60.0f;
        bool stayedFinite = true;
        for (int i = 0; i < 600; ++i) { // 10 simulated seconds
            world.step(dt);
            if (!allFinite(world)) {
                stayedFinite = false;
                break;
            }
        }
        check(stayedFinite, "soft body ring never produces NaN/inf position or velocity while settling");

        // Shape check: average particle distance from the ring's centroid
        // should still be in the same ballpark as the original radius --
        // neither collapsed flat (too small) nor blown apart (too large).
        Vec2 centroid;
        for (Body* p : sb.particles) centroid += p->position;
        centroid *= (1.0f / static_cast<float>(sb.particles.size()));

        float avgDist = 0.0f;
        for (Body* p : sb.particles) avgDist += (p->position - centroid).length();
        avgDist /= static_cast<float>(sb.particles.size());

        std::printf("ring centroid after settling: (%.2f, %.2f), avg particle distance from centroid: %.2f "
                    "(original radius %.2f)\n",
                    centroid.x, centroid.y, avgDist, ringRadius);
        check(avgDist > ringRadius * 0.4f && avgDist < ringRadius * 2.0f,
              "soft body ring roughly keeps its shape (doesn't collapse flat or fly apart) after settling");

        // It should have come to rest on the floor, not be in freefall or
        // still bouncing wildly after 10 simulated seconds.
        float maxSpeed = 0.0f;
        for (Body* p : sb.particles) maxSpeed = std::max(maxSpeed, p->velocity.length());
        std::printf("max particle speed after settling: %.3f m/s\n", maxSpeed);
        check(maxSpeed < 2.0f, "soft body ring settles to a low velocity, not perpetual bouncing/energy gain");
    }

    // --- Soft body jelly: a ring plus a center hub -- settles without
    // exploding, and the hub stays roughly centered (proving the spokes are
    // actually holding it in the middle of the blob, not off to one side)
    {
        World world;
        world.gravity = {0.0f, -10.0f};
        world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);

        const float radius = 1.0f;
        const int segments = 10;
        SoftBody jelly = makeSoftBodyJelly(world, {0.0f, 5.0f}, radius, segments);
        check(jelly.particles.size() == static_cast<size_t>(segments + 1),
              "makeSoftBodyJelly creates the ring particles plus one center hub");
        check(jelly.springs.size() > static_cast<size_t>(segments),
              "makeSoftBodyJelly adds a spoke spring from the hub to every ring particle");

        world.onPreSubstep = [&jelly](float) { jelly.applySpringForces(); };

        const float dt = 1.0f / 60.0f;
        bool stayedFinite = true;
        for (int i = 0; i < 600; ++i) { // 10 simulated seconds
            world.step(dt);
            if (!allFinite(world)) {
                stayedFinite = false;
                break;
            }
        }
        check(stayedFinite, "soft body jelly never produces NaN/inf position or velocity while settling");

        Body* hub = jelly.particles[0];
        Vec2 centroid;
        for (Body* p : jelly.particles) centroid += p->position;
        centroid *= (1.0f / static_cast<float>(jelly.particles.size()));

        float hubOffsetFromCentroid = (hub->position - centroid).length();
        std::printf("jelly hub position: (%.2f, %.2f), offset from centroid: %.3f\n", hub->position.x,
                    hub->position.y, hubOffsetFromCentroid);
        check(hubOffsetFromCentroid < radius * 0.5f,
              "the jelly's hub stays roughly centered in the blob (spokes don't let it get shoved aside)");

        float maxSpeed = 0.0f;
        for (Body* p : jelly.particles) maxSpeed = std::max(maxSpeed, p->velocity.length());
        std::printf("jelly max particle speed after settling: %.3f m/s\n", maxSpeed);
        check(maxSpeed < 2.0f, "soft body jelly settles to a low velocity, not perpetual bouncing/energy gain");
    }

    // --- Cloth: a pinned top row should hold still while the rest hangs
    // and drapes below it, without exploding
    {
        World world;
        world.gravity = {0.0f, -10.0f};
        // No floor -- this specifically tests hanging from the pinned row,
        // not landing on something.

        const int cols = 6, rows = 5;
        SoftBody cloth = makeSoftBodyCloth(world, {-1.0f, 5.0f}, cols, rows, 0.4f);
        check(cloth.particles.size() == static_cast<size_t>(cols * rows),
              "makeSoftBodyCloth creates a full cols x rows grid of particles");

        Body* pinned = cloth.particles[0]; // top-left, row 0
        Vec2 pinnedStart = pinned->position;
        check(pinned->type == BodyType::Static, "makeSoftBodyCloth's top row defaults to pinned (Static)");

        world.onPreSubstep = [&cloth](float) { cloth.applySpringForces(); };

        const float dt = 1.0f / 60.0f;
        bool stayedFinite = true;
        for (int i = 0; i < 600; ++i) { // 10 simulated seconds
            world.step(dt);
            if (!allFinite(world)) {
                stayedFinite = false;
                break;
            }
        }
        check(stayedFinite, "cloth never produces NaN/inf position or velocity while hanging/settling");
        check((pinned->position - pinnedStart).length() < 1e-4f,
              "a pinned (Static) top-row particle never moves, regardless of spring forces pulling on it");

        // The lowest particle should have fallen well below the pinned row
        // -- i.e. it's actually hanging/draping under gravity, not frozen
        // in its initial flat grid shape.
        float pinnedY = pinned->position.y;
        float lowestY = pinnedY;
        for (Body* p : cloth.particles) lowestY = std::min(lowestY, p->position.y);
        std::printf("cloth pinned row y=%.2f, lowest particle y=%.2f\n", pinnedY, lowestY);
        check(lowestY < pinnedY - 0.5f, "the cloth's unpinned rows hang down below the pinned top row");

        float maxSpeed = 0.0f;
        for (Body* p : cloth.particles) maxSpeed = std::max(maxSpeed, p->velocity.length());
        std::printf("cloth max particle speed after settling: %.3f m/s\n", maxSpeed);
        check(maxSpeed < 3.0f, "cloth settles to a low velocity after hanging");
    }

    // --- Regular polygon (triangle) collides/settles like any other shape
    {
        World world;
        world.gravity = {0.0f, -10.0f};
        world.createBox({0.0f, 0.0f}, 10.0f, 0.5f, BodyType::Static);

        std::vector<Vec2> tri = {{1.0f, -0.6f}, {0.0f, 0.8f}, {-1.0f, -0.6f}};
        Body* triangle = world.createBody(ShapeData::MakePolygon(tri), {0.3f, 5.0f}, BodyType::Dynamic);
        check(triangle->mass > 0.0f, "a hand-built triangle polygon gets a valid positive mass");

        const float dt = 1.0f / 60.0f;
        bool stayedFinite = true;
        for (int i = 0; i < 300; ++i) {
            world.step(dt);
            if (!std::isfinite(triangle->position.y)) {
                stayedFinite = false;
                break;
            }
        }
        check(stayedFinite, "a triangle body never produces NaN/inf while falling and settling");
        check(triangle->position.y > 0.0f && triangle->position.y < 5.0f,
              "the triangle falls and comes to rest above the floor, not through it or still falling");
    }

    if (g_failures == 0) {
        std::printf("\nAll soft body tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d soft body test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
