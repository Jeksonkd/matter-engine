// Exercises the exact reentrancy pattern the bacteria demo depends on: a
// script's on_update spawns new bodies and re-attaches itself to them, and
// later removes bodies -- all while ScriptEngine::update() is mid-iteration
// over its own attachment table. This is precisely the scenario that used
// to crash (unordered_map mutated during iteration) before the snapshot-
// based update loop and the ScriptEngine::reset()/move-assignment fix.

#include "p2d/World.hpp"
#include "p2d/script/ScriptEngine.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>

#ifndef P2D_SCRIPTS_DIR
#define P2D_SCRIPTS_DIR "scripts"
#endif

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
    world.gravity = {0.0f, -3.0f};

    ScriptEngine engine;
    engine.onError = [](const std::string& msg) { std::printf("[lua error] %s\n", msg.c_str()); };
    world.onBodyRemoved = [&engine](Body* b) {
        if (b) engine.detachScript(*b);
    };
    engine.bindWorld(world);

    Body* ground = world.createBox({0.0f, 0.0f}, 9.0f, 0.5f, BodyType::Static);
    ground->name = "ground";

    Body* seed = world.createCircle({0.0f, 3.0f}, 0.2f, BodyType::Dynamic);
    seed->name = "seed";
    engine.attachScript(*seed, std::string(P2D_SCRIPTS_DIR) + "/bacteria.lua");

    size_t maxSeen = 0;
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 60 * 40; ++i) { // 40 simulated seconds
        engine.update(world, dt);
        world.step(dt);
        maxSeen = std::max(maxSeen, world.bodies().size());
    }

    check(true, "40 seconds of bacteria reproduction/death ran without crashing");
    check(maxSeen > 3, "population grew beyond the single seed body (reproduction happened)");
    check(maxSeen < 200, "population stayed bounded by the cap (no runaway growth)");
    check(!world.bodies().empty(), "world is still in a valid state at the end (ground still present)");

    // A full reset (fresh ScriptEngine + fresh World) after heavy scripted
    // activity is exactly the sequence that used to crash via the old
    // `scriptEngine_ = ScriptEngine()` move-assignment bug.
    engine.reset();
    world.clear();
    engine.bindWorld(world);
    Body* afterReset = world.createCircle({0.0f, 3.0f}, 0.2f, BodyType::Dynamic);
    engine.attachScript(*afterReset, std::string(P2D_SCRIPTS_DIR) + "/bacteria.lua");
    engine.update(world, dt);
    world.step(dt);
    check(true, "reset() after heavy scripted activity, then reuse, did not crash");

    if (g_failures == 0) {
        std::printf("\nAll bacteria reproduction tests passed. Final population before reset: %zu\n", maxSeen);
        return EXIT_SUCCESS;
    }
    std::printf("\n%d bacteria test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
