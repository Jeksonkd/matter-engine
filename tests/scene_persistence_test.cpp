// Headless test for scene save/load (app/src/ScenePersistence.cpp): the
// JSON encode/decode logic behind autosave-to-disk and reload-on-project-
// open, verified against a plain p2d::World the same way as every other
// engine-level test -- this deliberately has no SFML/ImGui/ScriptEngine
// dependency (see ScenePersistence.hpp's comment), which is what makes it
// possible to test headlessly at all.

#include "ScenePersistence.hpp"

#include "p2d/SoftBody.hpp"
#include "p2d/SpringJoint.hpp"
#include "p2d/World.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace p2d;
using namespace p2d::app;

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
    // --- Round-trip: bodies (circle + polygon), properties, and a script path
    {
        World world;
        world.gravity = {0.0f, -12.0f};

        Body* circle = world.createCircle({1.0f, 2.0f}, 0.4f, BodyType::Dynamic);
        circle->name = "ball";
        circle->rotation = 0.3f;
        circle->velocity = Vec2(1.5f, -2.0f);
        circle->angularVelocity = 0.5f;
        circle->density = 2.0f;
        circle->computeMass();
        circle->restitution = 0.7f;
        circle->friction = 0.6f;
        circle->linearDamping = 0.1f;
        circle->angularDamping = 0.2f;
        circle->gravityScale = 0.5f;
        circle->isSensor = true;
        circle->scriptPath = "scripts/bouncy_ball.lua";
        circle->texturePath = "textures/ball.png";

        std::vector<Vec2> tri = {{1.0f, -0.5f}, {0.0f, 0.5f}, {-1.0f, -0.5f}};
        Body* triangle = world.createBody(ShapeData::MakePolygon(tri), {-3.0f, 4.0f}, BodyType::Static);
        triangle->name = "wedge";
        triangle->fixedRotation = true;
        triangle->computeMass();

        std::vector<SoftBody> softBodies;
        std::vector<SpringJoint> springJoints;
        nlohmann::json saved = saveScene(world, softBodies, springJoints);

        World loaded;
        std::vector<SoftBody> loadedSoftBodies;
        std::vector<SpringJoint> loadedSpringJoints;
        std::vector<std::pair<std::string, std::string>> attachedScripts;
        bool ok = loadScene(saved, loaded, loadedSoftBodies, loadedSpringJoints,
                            [&](Body& b, const std::string& path) { attachedScripts.emplace_back(b.name, path); });

        check(ok, "loadScene() succeeds for a freshly-saved scene");
        check(std::fabs(loaded.gravity.y - (-12.0f)) < 1e-4f, "world gravity round-trips");
        check(loaded.bodies().size() == 2, "both bodies round-trip");

        Body* loadedBall = loaded.findByName("ball");
        check(loadedBall != nullptr, "the circle body is found by its saved name");
        if (loadedBall) {
            check(loadedBall->shape.type == ShapeType::Circle, "shape type (circle) round-trips");
            check(std::fabs(loadedBall->shape.radius - 0.4f) < 1e-4f, "circle radius round-trips");
            check(std::fabs(loadedBall->position.x - 1.0f) < 1e-4f &&
                      std::fabs(loadedBall->position.y - 2.0f) < 1e-4f,
                  "position round-trips");
            check(std::fabs(loadedBall->rotation - 0.3f) < 1e-4f, "rotation round-trips");
            check(std::fabs(loadedBall->velocity.x - 1.5f) < 1e-4f && std::fabs(loadedBall->velocity.y + 2.0f) < 1e-4f,
                  "velocity round-trips");
            check(std::fabs(loadedBall->angularVelocity - 0.5f) < 1e-4f, "angular velocity round-trips");
            check(std::fabs(loadedBall->density - 2.0f) < 1e-4f, "density round-trips");
            check(loadedBall->mass > 0.0f, "mass is recomputed from the round-tripped density/shape");
            check(std::fabs(loadedBall->restitution - 0.7f) < 1e-4f, "restitution round-trips");
            check(std::fabs(loadedBall->friction - 0.6f) < 1e-4f, "friction round-trips");
            check(std::fabs(loadedBall->linearDamping - 0.1f) < 1e-4f, "linear damping round-trips");
            check(std::fabs(loadedBall->angularDamping - 0.2f) < 1e-4f, "angular damping round-trips");
            check(std::fabs(loadedBall->gravityScale - 0.5f) < 1e-4f, "gravity scale round-trips");
            check(loadedBall->isSensor, "is_sensor round-trips");
            check(loadedBall->type == BodyType::Dynamic, "body type round-trips");
            check(loadedBall->texturePath == "textures/ball.png", "texture path round-trips");
        }

        Body* loadedWedge = loaded.findByName("wedge");
        check(loadedWedge != nullptr, "the polygon body is found by its saved name");
        if (loadedWedge) {
            check(loadedWedge->shape.type == ShapeType::Polygon, "shape type (polygon) round-trips");
            check(loadedWedge->shape.vertices.size() == 3, "polygon vertex count round-trips");
            check(loadedWedge->fixedRotation, "fixed_rotation round-trips");
            check(loadedWedge->type == BodyType::Static, "Static body type round-trips");
            check(loadedWedge->texturePath.empty(), "a body with no texture assigned loads with an empty path");
        }

        check(attachedScripts.size() == 1 && attachedScripts[0].first == "ball" &&
                  attachedScripts[0].second == "scripts/bouncy_ball.lua",
              "the script path is passed to attachScript for exactly the body that had one");
    }

    // --- Round-trip: a soft body ring and a standalone spring joint
    {
        World world;
        world.gravity = {0.0f, -9.81f};
        SoftBody ring = makeSoftBodyRing(world, {0.0f, 5.0f}, 1.0f, 6);
        for (size_t i = 0; i < ring.particles.size(); ++i) ring.particles[i]->name = "ring_p" + std::to_string(i);

        Body* anchor = world.createBox({-4.0f, 6.0f}, 0.3f, 0.3f, BodyType::Static);
        anchor->name = "anchor";
        Body* weight = world.createCircle({-4.0f, 4.0f}, 0.3f, BodyType::Dynamic);
        weight->name = "weight";

        std::vector<SoftBody> softBodies = {ring};
        std::vector<SpringJoint> springJoints = {{anchor, weight, 1.5f, 40.0f, 2.0f}};

        nlohmann::json saved = saveScene(world, softBodies, springJoints);

        World loaded;
        std::vector<SoftBody> loadedSoftBodies;
        std::vector<SpringJoint> loadedSpringJoints;
        bool ok = loadScene(saved, loaded, loadedSoftBodies, loadedSpringJoints, nullptr);

        check(ok, "loadScene() succeeds for a scene with a soft body and a spring joint");
        check(loadedSoftBodies.size() == 1, "the soft body round-trips as one group");
        if (loadedSoftBodies.size() == 1) {
            check(loadedSoftBodies[0].particles.size() == ring.particles.size(),
                  "the soft body's particle count round-trips");
            check(loadedSoftBodies[0].springs.size() == ring.springs.size(),
                  "the soft body's spring count round-trips");
            bool allResolved = true;
            for (Body* p : loadedSoftBodies[0].particles) allResolved = allResolved && (p != nullptr);
            check(allResolved, "every soft body particle is re-resolved to a real (non-null) Body");
        }

        check(loadedSpringJoints.size() == 1, "the standalone spring joint round-trips");
        if (loadedSpringJoints.size() == 1) {
            const SpringJoint& j = loadedSpringJoints[0];
            check(j.a != nullptr && j.b != nullptr, "the spring joint's bodies are re-resolved (non-null)");
            check(j.a && j.a->name == "anchor" && j.b && j.b->name == "weight",
                  "the spring joint connects the correct two bodies by name");
            check(std::fabs(j.restLength - 1.5f) < 1e-4f && std::fabs(j.stiffness - 40.0f) < 1e-4f &&
                      std::fabs(j.damping - 2.0f) < 1e-4f,
                  "the spring joint's restLength/stiffness/damping round-trip");
        }
    }

    // --- A malformed/unexpected JSON value is rejected, not crashed on
    {
        World world;
        world.createCircle({0.0f, 0.0f}, 1.0f, BodyType::Dynamic);
        std::vector<SoftBody> softBodies;
        std::vector<SpringJoint> springJoints;

        nlohmann::json notAScene = nlohmann::json::array({1, 2, 3}); // an array, not an object
        World loaded;
        std::vector<SoftBody> loadedSoftBodies;
        std::vector<SpringJoint> loadedSpringJoints;
        bool ok = loadScene(notAScene, loaded, loadedSoftBodies, loadedSpringJoints, nullptr);
        check(!ok, "loadScene() rejects a JSON value that isn't a scene object, instead of crashing");
    }

    if (g_failures == 0) {
        std::printf("\nAll scene persistence tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d scene persistence test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
