// Regression test for a reported bug: two boxes stacked flush on top of
// each other (a floor + one box + one box directly above it, same width)
// never truly settled -- the stack looked at rest, but the contact's
// tangent (friction) impulse kept flipping between a tiny positive and
// negative value in periodic bursts (every ~timeToSleep seconds) forever
// instead of staying converged, and neither box ever reached a lasting
// sleep.
//
// Root cause (found by hooking World::onContact and logging
// tangentImpulse/isAwake/sleepTime together): the two boxes' independent
// per-body sleep timers cross World::timeToSleep a few steps apart. The
// first one to cross goes to sleep, but next step's contact generation
// unconditionally wakes any sleeping body touched by one that isn't ALSO
// asleep (isEffectivelyStatic() needs BOTH sides asleep to skip
// re-testing) -- so it's immediately woken again, with its velocity having
// been hard-reset to zero and then given one step of ungoverned gravity
// before the solver catches it. That hard reset perturbs the warm-started
// contact impulses, producing a burst of tangentImpulse sign flips as they
// re-converge -- and since the same desync recurs every time either body's
// timer next crosses the threshold, this repeated forever, so the stack
// never actually reached a lasting sleep. The fix
// (World::updateSleepState's two-pass "island" check) makes a body's sleep
// touching another still-awake dynamic body wait for that neighbor to also
// be ready, so touching bodies fall asleep together instead of taking
// turns.
//
// This test hooks World::onContact to record every tangentImpulse sample
// and sign flip across a full settle run, then asserts (a) both boxes
// reach a lasting sleep by the end (the bug's bodies never did -- they
// stayed permanently awake, cycling through this hiccup) and (b) the total
// number of sign flips across the whole run is small (the bug produced
// hundreds; ordinary initial-impact settling produces only a handful).

#include "p2d/World.hpp"

#include <algorithm>
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

    Body* bottom = world.createBox({0.0f, 1.0f}, 0.5f, 0.5f, BodyType::Dynamic);
    bottom->friction = 0.6f;
    bottom->restitution = 0.0f;

    Body* top = world.createBox({0.0f, 2.02f}, 0.5f, 0.5f, BodyType::Dynamic);
    top->friction = 0.6f;
    top->restitution = 0.0f;

    // Sample the bottom/top contact's tangentImpulse on every step across
    // the whole run (not just a late "settled" window): the bug was a
    // periodic hiccup recurring every ~timeToSleep seconds forever, so the
    // whole run needs watching, not just the tail end. Point 0 and point 1
    // are tracked as two SEPARATE time series -- a 2-point edge manifold's
    // two points have different (and generally opposite-sign) steady-state
    // tangent impulses, so interleaving them into one series would report
    // every single step as a "flip" purely from alternating between them,
    // not from anything actually oscillating over time.
    std::vector<float> samples[2];
    world.onContact = [&](const Contact& c) {
        bool isBoxPair = (c.a == bottom && c.b == top) || (c.a == top && c.b == bottom);
        if (!isBoxPair) return;
        for (int p = 0; p < c.count; ++p) samples[p].push_back(c.tangentImpulse[p]);
    };

    const float dt = 1.0f / 120.0f;
    const int totalSteps = 600; // 5 simulated seconds, same window as stacking_test
    for (int i = 0; i < totalSteps; ++i) world.step(dt);

    // The bug's bodies never reached a lasting sleep -- they cycled through
    // sleep/forced-rewake forever. A fixed stack settles down and both
    // boxes stay genuinely asleep well before 5 simulated seconds are up.
    check(!bottom->isAwake, "bottom box reaches a lasting sleep (bug: never slept, cycled forever)");
    check(!top->isAwake, "top box reaches a lasting sleep (bug: never slept, cycled forever)");

    // A brief burst of sign flips right after initial impact (while real
    // contact-impact energy is still dissipating) is normal solver
    // behavior, not the bug -- so only the SECOND HALF of each point's
    // pre-sleep time series is checked for stability. The bug's signature
    // was that it never had a quiet second half at all: flips recurred at
    // roughly the same rate for as long as the two boxes stayed in contact,
    // however far out you looked. A correctly-settling contact should flip
    // only a handful of times total, none of them in the back half.
    int totalFlips = 0;
    size_t totalBackHalfSamples = 0;
    for (int p = 0; p < 2; ++p) {
        const std::vector<float>& s = samples[p];
        size_t half = s.size() / 2;
        for (size_t i = std::max<size_t>(half, 1); i < s.size(); ++i) {
            if ((s[i] > 0.0f && s[i - 1] < 0.0f) || (s[i] < 0.0f && s[i - 1] > 0.0f)) ++totalFlips;
        }
        totalBackHalfSamples += s.size() - std::min(half, s.size());
    }
    char msg[160];
    std::snprintf(msg, sizeof(msg),
                  "tangentImpulse sign is stable in the back half of the pre-sleep window (%d flip(s) across %zu samples, expected <= 2)",
                  totalFlips, totalBackHalfSamples);
    check(totalFlips <= 2, msg);

    float maxAbs = 0.0f;
    for (int p = 0; p < 2; ++p)
        for (float s : samples[p]) maxAbs = std::max(maxAbs, std::fabs(s));
    std::snprintf(msg, sizeof(msg), "tangentImpulse stays small once settled (max |impulse| = %.6f)", maxAbs);
    check(maxAbs < 0.05f, msg);

    check(std::fabs(bottom->position.x) < 0.05f, "bottom box stays put (no sideways drift)");
    check(std::fabs(top->position.x) < 0.05f, "top box stays put (no sideways drift)");
    check(bottom->velocity.length() < 0.05f, "bottom box is at rest (or asleep)");
    check(top->velocity.length() < 0.05f, "top box is at rest (or asleep)");

    if (g_failures == 0) {
        std::printf("\nAll friction jitter tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d friction jitter test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
