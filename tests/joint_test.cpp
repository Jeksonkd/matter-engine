// Headless checks for p2d's rigid joints (Joint.hpp): DistanceJoint,
// RevoluteJoint, WeldJoint, PrismaticJoint. These are genuinely bilateral
// (equality) velocity constraints solved by World's sequential-impulse
// solver -- unlike SpringJoint, they hold firmly rather than stretching
// under load.

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
    const float dt = 1.0f / 120.0f;

    // --- DistanceJoint: holds its length under gravity, doesn't sag -------
    {
        World world;
        Body* anchor = world.createCircle({0.0f, 5.0f}, 0.1f, BodyType::Static);
        Body* bob = world.createCircle({0.0f, 0.0f}, 0.3f, BodyType::Dynamic);
        world.createDistanceJoint(anchor, bob, anchor->position, bob->position);

        for (int i = 0; i < 600; ++i) world.step(dt); // 5s, well past settling

        float dist = (bob->position - anchor->position).length();
        check(std::fabs(dist - 5.0f) < 0.1f,
              "DistanceJoint holds its 5m length under gravity (settled distance within 0.1m)");
    }

    // --- DistanceJoint holds even under a hard pull ------------------------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({-2.5f, 0.0f}, 0.3f, BodyType::Dynamic);
        Body* b = world.createCircle({2.5f, 0.0f}, 0.3f, BodyType::Dynamic);
        world.createDistanceJoint(a, b, a->position, b->position); // length = 5
        a->velocity = Vec2(-3.0f, 0.0f); // pulling apart hard
        b->velocity = Vec2(3.0f, 0.0f);

        for (int i = 0; i < 120; ++i) world.step(dt); // 1s

        float dist = (b->position - a->position).length();
        check(dist < 6.0f, "DistanceJoint resists being pulled far past its rest length (stays under 6m, not 11m)");
    }

    // --- DistanceJoint: enableMotorLimit on, untouched min/max still holds
    // the exact creation-time length (createDistanceJoint() sets minLength
    // = maxLength = length precisely so this is true) --------------------
    {
        World world;
        Body* anchor = world.createCircle({0.0f, 5.0f}, 0.1f, BodyType::Static);
        Body* bob = world.createCircle({0.0f, 0.0f}, 0.3f, BodyType::Dynamic);
        DistanceJoint* j = world.createDistanceJoint(anchor, bob, anchor->position, bob->position);
        j->enableMotorLimit = true; // min/max left untouched (both == 5, the creation length)

        for (int i = 0; i < 600; ++i) world.step(dt); // 5s, well past settling

        float dist = (bob->position - anchor->position).length();
        check(std::fabs(dist - 5.0f) < 0.1f,
              "DistanceJoint: enableMotorLimit alone (min/max untouched) still holds the exact length");
    }

    // --- DistanceJoint: limit lets it swing between min and max ------------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({-2.5f, 0.0f}, 0.3f, BodyType::Dynamic);
        Body* b = world.createCircle({2.5f, 0.0f}, 0.3f, BodyType::Dynamic);
        DistanceJoint* j = world.createDistanceJoint(a, b, a->position, b->position); // length = 5
        j->enableMotorLimit = true;
        j->minLength = 4.0f;
        j->maxLength = 6.0f;
        a->velocity = Vec2(-3.0f, 0.0f); // pulling apart hard, same as the hard-pull test above
        b->velocity = Vec2(3.0f, 0.0f);

        float maxDistSeen = 0.0f;
        for (int i = 0; i < 120; ++i) {
            world.step(dt);
            maxDistSeen = std::max(maxDistSeen, (b->position - a->position).length());
        }

        check(maxDistSeen < 6.2f, "DistanceJoint limit stops lengthening at maxLength (stays near 6m, not 11m)");
        check(maxDistSeen > 5.5f,
              "DistanceJoint limit actually let it lengthen past the old exact 5m (limits, not still exact)");
    }

    // --- DistanceJoint: motor reels the length in at a target rate ---------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({-2.5f, 0.0f}, 0.3f, BodyType::Dynamic);
        Body* b = world.createCircle({2.5f, 0.0f}, 0.3f, BodyType::Dynamic);
        DistanceJoint* j = world.createDistanceJoint(a, b, a->position, b->position); // length = 5
        j->enableMotorLimit = true;
        j->minLength = 0.5f;
        j->maxLength = 5.0f;
        j->motorSpeed = -1.0f; // reel in at 1 m/s
        j->maxMotorForce = 100.0f;

        for (int i = 0; i < 240; ++i) world.step(dt); // 2s -- should reel in ~2m, well short of minLength

        float dist = (b->position - a->position).length();
        check(dist < 4.0f, "DistanceJoint motor actually reels the length in over time");
        check(dist > 2.0f, "DistanceJoint motor hasn't overshot past a sane distance in 2s at 1 m/s");
    }

    // --- RevoluteJoint: anchor points stay coincident under gravity -------
    {
        World world;
        Body* anchor = world.createCircle({0.0f, 5.0f}, 0.1f, BodyType::Static);
        Body* arm = world.createBox({2.0f, 5.0f}, 1.0f, 0.2f, BodyType::Dynamic); // extends to the right of the pivot
        world.createRevoluteJoint(anchor, arm, anchor->position); // pinned at anchor's own center

        for (int i = 0; i < 300; ++i) world.step(dt); // 2.5s -- swings like a pendulum

        // The arm's center of mass is 2m from the pivot at creation (pivot
        // at the arm's local (-1,0)) -- a rigid pin keeps it swinging on
        // that same radius rather than drifting toward or away from it.
        float distFromPivot = (arm->position - anchor->position).length();
        check(std::fabs(distFromPivot - 2.0f) < 0.15f,
              "RevoluteJoint keeps the arm at a constant ~2m radius from its pivot (swings, doesn't drift)");
    }

    // --- RevoluteJoint: relative rotation is free ---------------------------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({0.0f, 0.0f}, 0.5f, BodyType::Static);
        Body* b = world.createBox({1.0f, 0.0f}, 1.0f, 0.2f, BodyType::Dynamic);
        world.createRevoluteJoint(a, b, {0.0f, 0.0f});
        b->applyTorque(50.0f);

        for (int i = 0; i < 60; ++i) world.step(dt); // 0.5s

        check(std::fabs(b->angularVelocity) > 0.1f,
              "RevoluteJoint allows free relative rotation -- a torque actually spins the body");
        // And the pin should still hold despite the spin.
        Vec2 rB = rotate(Vec2(-1.0f, 0.0f), b->rotation); // b's local anchor was its own left edge, at creation
        float pinError = (b->position + rB - a->position).length();
        check(pinError < 0.2f, "RevoluteJoint's pin still holds while the body spins freely around it");
    }

    // --- RevoluteJoint: enableMotorLimit on, lower/upper untouched (both 0)
    // still leaves rotation completely free -- the exact bug found and
    // fixed while building this: without the lowerAngle < upperAngle guard,
    // both limit branches trigger at angle == 0 and lock the joint solid
    // the instant this flag is set. -----------------------------------------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({0.0f, 0.0f}, 0.5f, BodyType::Static);
        Body* b = world.createBox({1.0f, 0.0f}, 1.0f, 0.2f, BodyType::Dynamic);
        RevoluteJoint* j = world.createRevoluteJoint(a, b, {0.0f, 0.0f});
        j->enableMotorLimit = true; // lowerAngle == upperAngle == 0, untouched
        b->applyTorque(50.0f);

        for (int i = 0; i < 60; ++i) world.step(dt); // 0.5s

        check(std::fabs(b->angularVelocity) > 0.1f,
              "RevoluteJoint: enableMotorLimit alone (angle range untouched) does NOT lock rotation");
    }

    // --- RevoluteJoint: motor drives relative angular velocity to its target
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({0.0f, 0.0f}, 0.5f, BodyType::Static);
        Body* b = world.createBox({1.0f, 0.0f}, 1.0f, 0.2f, BodyType::Dynamic);
        RevoluteJoint* j = world.createRevoluteJoint(a, b, {0.0f, 0.0f});
        j->enableMotorLimit = true;
        j->motorSpeed = 4.0f; // rad/s
        j->maxMotorTorque = 1000.0f; // generous -- should reach the target easily

        for (int i = 0; i < 120; ++i) world.step(dt); // 1s

        check(std::fabs(b->angularVelocity - 4.0f) < 0.2f,
              "RevoluteJoint motor drives relative angular velocity to motorSpeed");
    }

    // --- RevoluteJoint: motor stalls under a load it can't overcome --------
    {
        World world;
        world.gravity = {0.0f, -9.81f};
        Body* a = world.createCircle({0.0f, 5.0f}, 0.5f, BodyType::Static);
        // An off-center arm: gravity applies a real resisting torque about
        // the pivot, unlike the zero-gravity motor test above.
        Body* b = world.createBox({2.0f, 5.0f}, 1.0f, 0.2f, BodyType::Dynamic);
        RevoluteJoint* j = world.createRevoluteJoint(a, b, a->position);
        j->enableMotorLimit = true;
        j->motorSpeed = 10.0f; // an unreachably fast target
        j->maxMotorTorque = 0.5f; // deliberately too weak to lift the arm's weight

        for (int i = 0; i < 120; ++i) world.step(dt); // 1s

        check(std::fabs(b->angularVelocity) < 5.0f,
              "RevoluteJoint motor's maxMotorTorque cap actually limits it -- weak motor can't reach motorSpeed");
    }

    // --- RevoluteJoint: angle limit stops rotation within [lower, upper] ---
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({0.0f, 0.0f}, 0.5f, BodyType::Static);
        Body* b = world.createBox({1.0f, 0.0f}, 1.0f, 0.2f, BodyType::Dynamic);
        RevoluteJoint* j = world.createRevoluteJoint(a, b, {0.0f, 0.0f});
        j->enableMotorLimit = true;
        j->lowerAngle = -0.5f;
        j->upperAngle = 0.5f;
        b->applyTorque(80.0f); // hard enough to spin well past the limit if unconstrained

        for (int i = 0; i < 180; ++i) world.step(dt); // 1.5s -- plenty of time to settle at the limit

        float angle = b->rotation - a->rotation;
        check(angle < 0.6f, "RevoluteJoint angle limit stops rotation at upperAngle, not well past it");
        check(angle > 0.3f, "RevoluteJoint angle limit actually let it rotate up to near the limit, not stay at 0");
    }

    // --- WeldJoint: locks relative rotation too -----------------------------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createCircle({0.0f, 0.0f}, 0.5f, BodyType::Static);
        Body* b = world.createBox({1.0f, 0.0f}, 1.0f, 0.2f, BodyType::Dynamic);
        world.createWeldJoint(a, b, {0.0f, 0.0f});
        b->applyTorque(50.0f);

        for (int i = 0; i < 60; ++i) world.step(dt); // 0.5s

        check(std::fabs(b->rotation - a->rotation) < 0.1f,
              "WeldJoint keeps the relative angle fixed even when torque is applied (unlike Revolute)");
        check(std::fabs(b->angularVelocity) < 0.1f,
              "WeldJoint's angle lock stops the torqued body from actually spinning (A is Static/immovable)");
    }

    // --- PrismaticJoint: slides along its axis, doesn't drift sideways -----
    {
        World world;
        Body* a = world.createBox({0.0f, 5.0f}, 2.0f, 0.2f, BodyType::Static); // a horizontal rail
        Body* b = world.createBox({0.0f, 5.0f}, 0.3f, 0.3f, BodyType::Dynamic); // a slider hanging on it
        world.createPrismaticJoint(a, b, {0.0f, 5.0f}, {0.0f, -1.0f}); // slides straight down

        for (int i = 0; i < 120; ++i) world.step(dt); // 1s of falling along the rail

        check(b->position.y < 4.5f, "PrismaticJoint lets the slider move along its axis (falls under gravity)");
        check(std::fabs(b->position.x - 0.0f) < 0.05f,
              "PrismaticJoint keeps the slider from drifting perpendicular to its axis");
        check(std::fabs(b->rotation - a->rotation) < 0.05f,
              "PrismaticJoint locks relative rotation (slider doesn't tumble while sliding)");
    }

    // --- PrismaticJoint: enableMotorLimit on, lower/upper untouched (both 0)
    // still leaves sliding completely free -- same guard-bug shape as
    // RevoluteJoint's equivalent test above. ------------------------------
    {
        World world;
        Body* a = world.createBox({0.0f, 5.0f}, 2.0f, 0.2f, BodyType::Static);
        Body* b = world.createBox({0.0f, 5.0f}, 0.3f, 0.3f, BodyType::Dynamic);
        PrismaticJoint* j = world.createPrismaticJoint(a, b, {0.0f, 5.0f}, {0.0f, -1.0f});
        j->enableMotorLimit = true; // lowerTranslation == upperTranslation == 0, untouched

        for (int i = 0; i < 120; ++i) world.step(dt); // 1s of falling along the rail

        check(b->position.y < 4.5f,
              "PrismaticJoint: enableMotorLimit alone (translation range untouched) does NOT lock the slide");
    }

    // --- PrismaticJoint: motor drives translation speed to its target ------
    {
        World world;
        world.gravity = {0.0f, 0.0f};
        Body* a = world.createBox({0.0f, 5.0f}, 2.0f, 0.2f, BodyType::Static);
        Body* b = world.createBox({0.0f, 5.0f}, 0.3f, 0.3f, BodyType::Dynamic);
        PrismaticJoint* j = world.createPrismaticJoint(a, b, {0.0f, 5.0f}, {0.0f, -1.0f});
        j->enableMotorLimit = true;
        j->motorSpeed = 3.0f; // m/s along the axis (0,-1) -- i.e. falling at 3 m/s
        j->maxMotorForce = 1000.0f; // generous

        for (int i = 0; i < 120; ++i) world.step(dt); // 1s

        // axis is (0,-1), so "translation increasing at 3 m/s" means
        // b->position.y decreasing at 3 m/s.
        check(std::fabs(b->velocity.y + 3.0f) < 0.2f,
              "PrismaticJoint motor drives the slide velocity to motorSpeed");
    }

    // --- PrismaticJoint: translation limit stops the slide within range ----
    {
        World world;
        Body* a = world.createBox({0.0f, 5.0f}, 2.0f, 0.2f, BodyType::Static);
        Body* b = world.createBox({0.0f, 5.0f}, 0.3f, 0.3f, BodyType::Dynamic);
        PrismaticJoint* j = world.createPrismaticJoint(a, b, {0.0f, 5.0f}, {0.0f, -1.0f});
        j->enableMotorLimit = true;
        j->lowerTranslation = 0.0f;
        j->upperTranslation = 1.0f; // falls at most 1m before the limit catches it

        for (int i = 0; i < 180; ++i) world.step(dt); // 1.5s, well past settling at the limit

        float translation = 5.0f - b->position.y; // axis is (0,-1): translation = -(y - 5) = 5 - y
        check(translation < 1.2f, "PrismaticJoint translation limit stops the slide near upperTranslation");
        check(translation > 0.7f, "PrismaticJoint translation limit actually let it slide close to the limit");
    }

    // --- Removing a body drops any joint touching it (no dangling ptr) -----
    {
        World world;
        Body* a = world.createCircle({0.0f, 0.0f}, 0.3f, BodyType::Static);
        Body* b = world.createCircle({1.0f, 0.0f}, 0.3f, BodyType::Dynamic);
        world.createDistanceJoint(a, b, a->position, b->position);
        check(world.distanceJoints().size() == 1, "sanity: the joint was created");
        world.removeBody(b);
        check(world.distanceJoints().empty(), "removing a body drops any joint touching it");
        // Would crash/UB if a dangling Body* were dereferenced here.
        for (int i = 0; i < 10; ++i) world.step(dt);
        check(true, "stepping after removing a jointed body doesn't crash");
    }

    if (g_failures == 0) {
        std::printf("\nAll joint tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d joint test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
