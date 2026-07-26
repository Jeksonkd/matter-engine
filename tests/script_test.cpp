// Headless test for the Lua scripting layer: verifies attach/on_start,
// on_update being driven by World::step-adjacent calls, and that a script
// runtime error is reported through the callback instead of crashing.

#ifndef TESTS_LUA_DIR
#define TESTS_LUA_DIR "."
#endif

#include "p2d/World.hpp"
#include "p2d/script/ScriptEngine.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace p2d;
using namespace p2d::script;

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
    world.gravity = {0.0f, 0.0f};

    ScriptEngine engine;
    std::string lastError;
    engine.onError = [&](const std::string& msg) { lastError = msg; };
    engine.onLog = [](const std::string& msg) { std::printf("[lua log] %s\n", msg.c_str()); };
    engine.bindWorld(world);

    Body* body = world.createCircle({0.0f, 0.0f}, 0.5f, BodyType::Dynamic);
    body->name = "scripted_ball";

    engine.attachScript(*body, std::string(TESTS_LUA_DIR) + "/test_body_script.lua");

    check(engine.hasScript(*body), "script attaches successfully");
    check(std::fabs(body->restitution - 0.42f) < 1e-4f, "on_start mutated the body (restitution set from Lua)");
    check(std::fabs(body->linearDamping - 1.5f) < 1e-4f, "on_start set linear_damping via Lua");
    check(std::fabs(body->angularDamping - 0.5f) < 1e-4f, "on_start set angular_damping via Lua");
    check(std::fabs(body->gravityScale - 0.0f) < 1e-4f, "on_start set gravity_scale via Lua");
    check(body->isSensor, "on_start set is_sensor via Lua");
    check(body->fixedRotation, "on_start set fixed_rotation via Lua");
    check(body->invInertia == 0.0f, "setting fixed_rotation via Lua triggers computeMass (invInertia -> 0)");

    for (int i = 0; i < 5; ++i) {
        engine.update(world, 1.0f / 60.0f);
        world.step(1.0f / 60.0f);
    }

    check(body->velocity.x > 0.0f, "on_update applied a force to the body via Vec2/apply_force binding");

    float velocityYBeforeGui = body->velocity.y;
    engine.updateGui(world);
    check(body->velocity.y > velocityYBeforeGui, "updateGui() calls on_gui, independent of on_update/world.step");

    // Attaching a broken script must not crash and must report an error.
    Body* body2 = world.createCircle({2.0f, 0.0f}, 0.5f, BodyType::Dynamic);
    body2->name = "broken";
    engine.attachScript(*body2, std::string(TESTS_LUA_DIR) + "/broken_script.lua");
    check(!lastError.empty(), "runtime error in on_start is reported via onError, not a crash");

    engine.detachScript(*body);
    check(!engine.hasScript(*body), "detachScript removes the attachment");

    // world:bodies() lets a script inspect the whole scene, not just create
    // new bodies blind to what's already there (used by
    // spawn_1000_button.lua to spawn above existing balls). Add a static
    // box and another dynamic circle so the count can distinguish "only
    // dynamic circles" from "everything".
    world.createBox({0.0f, -5.0f}, 5.0f, 0.5f, BodyType::Static)->name = "floor";
    world.createCircle({1.0f, 2.0f}, 0.3f, BodyType::Dynamic)->name = "extra_ball";

    sol::protected_function_result countResult = engine.lua().script(
        "local n = 0 "
        "for _, b in ipairs(world:bodies()) do "
        "  if b.type == BodyType.Dynamic and b.radius > 0 then n = n + 1 end "
        "end "
        "return n");
    check(countResult.valid(), "world:bodies() iteration script runs without error");
    // scripted_ball + broken + extra_ball are Dynamic circles; floor is a
    // Static box, so it must NOT be counted.
    check(countResult.get<int>() == 3, "world:bodies() lets Lua filter to just dynamic circles ('balls')");

    // world:create_polygon() -- the same regular-N-gon path the editor's
    // unified Spawn tool uses for Triangle/Pentagon/Hexagon, now available
    // to scripts too (covers "every type of object" with one function
    // rather than one per shape).
    sol::protected_function_result polyResult =
        engine.lua().script("return world:create_polygon(3.0, 3.0, 5, 0.6, BodyType.Dynamic)");
    check(polyResult.valid(), "world:create_polygon() runs without error");
    Body* pentagon = polyResult.get<Body*>();
    check(pentagon != nullptr && pentagon->mass > 0.0f, "world:create_polygon() creates a body with valid mass");
    check(pentagon->shape.vertices.size() == 5, "world:create_polygon(sides=5) creates a 5-vertex polygon");

    // Body property/method expansion: density (recomputes mass), readonly
    // mass/inertia, is_awake, apply_torque, wake().
    float massBeforeDensityChange = pentagon->mass;
    pentagon->name = "pentagon_for_density_test";
    sol::protected_function_result densityScript = engine.lua().script(
        "local p = world:find_body('pentagon_for_density_test') "
        "p.density = p.density * 2.0 "
        "return p.mass");
    check(densityScript.valid(), "setting Body.density from Lua runs without error");
    check(densityScript.get<float>() > massBeforeDensityChange * 1.9f,
          "setting Body.density from Lua recomputes mass (doubling density roughly doubles mass)");

    sol::protected_function_result torqueScript = engine.lua().script(
        "local p = world:find_body('pentagon_for_density_test') "
        "p:apply_torque(5.0) "
        "return p.is_awake");
    check(torqueScript.valid() && torqueScript.get<bool>(),
          "Body:apply_torque() and Body.is_awake are usable from Lua");

    // Body.type: like density, wrapped so a Lua assignment recomputes mass
    // immediately (Static/Kinematic always get infinite mass) rather than
    // leaving stale mass data around. Restores Dynamic afterward so later
    // checks below (which expect a nonzero, recomputable mass) aren't left
    // dealing with a permanently-static body.
    sol::protected_function_result typeScript = engine.lua().script(
        "local p = world:find_body('pentagon_for_density_test') "
        "p.type = BodyType.Static "
        "local staticMass = p.mass "
        "p.type = BodyType.Dynamic "
        "return staticMass");
    check(typeScript.valid() && typeScript.get<float>() == 0.0f,
          "setting Body.type via Lua recomputes mass (Static -> mass 0)");

    // Body.matter_kind -- the Rigidbody/Matter/OptiMatter solver fidelity
    // dial, read/write from Lua.
    sol::protected_function_result matterKindScript = engine.lua().script(
        "local p = world:find_body('pentagon_for_density_test') "
        "p.matter_kind = MatterKind.OptiMatter "
        "return p.matter_kind == MatterKind.OptiMatter");
    check(matterKindScript.valid() && matterKindScript.get<bool>(),
          "Body.matter_kind is settable/readable from Lua");

    // Shape-kind conversions: set_circle/set_box/set_polygon rebuild the
    // shape and recompute mass, not just resize within the same kind.
    sol::protected_function_result setCircleScript = engine.lua().script(
        "local p = world:find_body('pentagon_for_density_test') "
        "p:set_circle(1.5) "
        "return p.radius, p.mass");
    check(setCircleScript.valid(), "Body:set_circle() runs without error");
    if (setCircleScript.valid()) {
        float radius = setCircleScript.get<float>(0);
        float mass = setCircleScript.get<float>(1);
        check(std::fabs(radius - 1.5f) < 1e-4f, "Body:set_circle() converts a polygon into a circle of that radius");
        check(mass > 0.0f, "Body:set_circle() recomputes a nonzero mass");
    }

    sol::protected_function_result radiusSetScript = engine.lua().script(
        "local p = world:find_body('pentagon_for_density_test') "
        "local before = p.mass "
        "p.radius = 3.0 "
        "return p.radius, p.mass ~= before");
    check(radiusSetScript.valid(), "Body.radius assignment (already a circle) runs without error");
    if (radiusSetScript.valid()) {
        float radius = radiusSetScript.get<float>(0);
        bool massChanged = radiusSetScript.get<bool>(1);
        check(std::fabs(radius - 3.0f) < 1e-4f, "Body.radius assignment resizes an existing circle");
        check(massChanged, "Body.radius assignment recomputes mass");
    }

    sol::protected_function_result setBoxScript = engine.lua().script(
        "local p = world:find_body('pentagon_for_density_test') "
        "p:set_box(0.5, 0.25) "
        "return p.radius"); // 0 for a non-circle shape
    check(setBoxScript.valid() && setBoxScript.get<float>() == 0.0f,
          "Body:set_box() converts to a box shape (radius reads back 0)");

    sol::protected_function_result radiusIgnoredScript = engine.lua().script(
        "local p = world:find_body('pentagon_for_density_test') "
        "p.radius = 9.0 "
        "return p.radius");
    check(radiusIgnoredScript.valid() && radiusIgnoredScript.get<float>() == 0.0f,
          "Body.radius assignment on a non-circle shape is silently ignored, not a crash/exception");

    sol::protected_function_result setPolygonScript = engine.lua().script(
        "local p = world:find_body('pentagon_for_density_test') "
        "p:set_polygon(6, 1.0) "
        "return p.radius"); // 0 for a non-circle shape
    check(setPolygonScript.valid() && setPolygonScript.get<float>() == 0.0f,
          "Body:set_polygon() converts to a polygon shape (radius reads back 0)");

    // Appearance: color_r/g/b are plain read/write, set_color() is the
    // one-call convenience for all three at once.
    sol::protected_function_result colorScript = engine.lua().script(
        "local p = world:find_body('pentagon_for_density_test') "
        "p.color_r = 10 "
        "p:set_color(200, 100, 50) "
        "return p.color_r, p.color_g, p.color_b");
    check(colorScript.valid(), "Body color_r/set_color() run without error");
    if (colorScript.valid()) {
        int r = colorScript.get<int>(0);
        int g = colorScript.get<int>(1);
        int b = colorScript.get<int>(2);
        check(r == 200 && g == 100 && b == 50, "Body:set_color() overwrites all three channels at once");
    }

    // World tracking helpers: count/find bodies by name, BodyType, and
    // MatterKind -- distinct from find_body() (exact name, first match
    // only) since names aren't required to be unique.
    world.createCircle({5.0f, 5.0f}, 0.2f, BodyType::Dynamic)->name = "tracked";
    world.createCircle({5.0f, 6.0f}, 0.2f, BodyType::Dynamic)->name = "tracked";
    Body* trackedStatic = world.createCircle({5.0f, 7.0f}, 0.2f, BodyType::Static);
    trackedStatic->name = "tracked";

    sol::protected_function_result countByNameScript =
        engine.lua().script("return world:count_by_name('tracked')");
    check(countByNameScript.valid() && countByNameScript.get<int>() == 3,
          "world:count_by_name() counts every body sharing that name, not just the first");

    sol::protected_function_result findByNameScript =
        engine.lua().script("return #world:find_bodies_by_name('tracked')");
    check(findByNameScript.valid() && findByNameScript.get<int>() == 3,
          "world:find_bodies_by_name() returns a table with every matching body");

    sol::protected_function_result countByTypeScript =
        engine.lua().script("return world:count_by_type(BodyType.Static)");
    check(countByTypeScript.valid() && countByTypeScript.get<int>() == 2, // floor + trackedStatic
          "world:count_by_type() counts bodies of a given BodyType");

    sol::protected_function_result countByMatterKindScript =
        engine.lua().script("return world:count_by_matter_kind(MatterKind.OptiMatter)");
    check(countByMatterKindScript.valid() && countByMatterKindScript.get<int>() == 1,
          "world:count_by_matter_kind() counts bodies with a given MatterKind (only the pentagon set earlier)");

    sol::protected_function_result findByMatterKindScript =
        engine.lua().script("return #world:find_bodies_by_matter_kind(MatterKind.OptiMatter)");
    check(findByMatterKindScript.valid() && findByMatterKindScript.get<int>() == 1,
          "world:find_bodies_by_matter_kind() returns a table with every matching body");

    // Matter (the standalone particle class): radius setter recomputes
    // mass, color/set_color, and the count/find-by-kind tracking helpers.
    Matter* m1 = world.createMatter({8.0f, 0.0f}, 0.4f, MatterKind::Matter);
    world.createMatter({8.5f, 0.0f}, 0.4f, MatterKind::Matter);
    world.createMatter({9.0f, 0.0f}, 0.4f, MatterKind::OptiMatter);
    m1->name = "m1";

    sol::protected_function_result matterRadiusScript = engine.lua().script(
        "local m = world:find_matter('m1') "
        "local before = m.mass "
        "m.radius = 1.0 "
        "return m.mass ~= before");
    check(matterRadiusScript.valid() && matterRadiusScript.get<bool>(),
          "Matter.radius assignment from Lua recomputes mass");

    sol::protected_function_result matterColorScript = engine.lua().script(
        "local m = world:find_matter('m1') "
        "m:set_color(1, 2, 3) "
        "return m.color_r, m.color_g, m.color_b");
    check(matterColorScript.valid(), "Matter:set_color() runs without error");
    if (matterColorScript.valid()) {
        int r = matterColorScript.get<int>(0);
        int g = matterColorScript.get<int>(1);
        int b = matterColorScript.get<int>(2);
        check(r == 1 && g == 2 && b == 3, "Matter:set_color() sets all three channels");
    }

    sol::protected_function_result countMatterByKindScript =
        engine.lua().script("return world:count_matter_by_kind(MatterKind.Matter)");
    check(countMatterByKindScript.valid() && countMatterByKindScript.get<int>() == 2,
          "world:count_matter_by_kind() counts Matter particles of a given kind");

    sol::protected_function_result findMatterByKindScript =
        engine.lua().script("return #world:find_matter_by_kind(MatterKind.OptiMatter)");
    check(findMatterByKindScript.valid() && findMatterByKindScript.get<int>() == 1,
          "world:find_matter_by_kind() returns a table with every matching particle");

    if (g_failures == 0) {
        std::printf("\nAll script tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d script test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
