#pragma once

#include "p2d/Vec2.hpp"

#include <cstdint>
#include <string>

namespace p2d {

// Per-object simulation-fidelity dial. Shared by both p2d::Matter (below)
// and p2d::Body (see Body::matterKind) -- same three-way meaning either
// way: Rigidbody is full, ordinary fidelity (the only sensible default for
// a Body -- an "ordinary rigidbody, no fidelity opt-in"); Matter is
// deliberately MORE accurate than Rigidbody (tighter sleep thresholds,
// smaller position-correction cap, tighter substep-forcing fraction -- see
// World's matter*/optiMatter* fields), at real extra cost; OptiMatter pulls
// the opposite way, opting a single body/particle into cheaper treatment
// (see World::step()'s substep/sleep/correction handling for both Body and
// Matter), independent of anything else in the scene. A plain Body
// defaults to Rigidbody and is completely unaffected unless its matterKind
// is explicitly changed.
enum class MatterKind { Rigidbody, Matter, OptiMatter };

// A point-mass particle: linear-only physics (position, velocity, mass) --
// deliberately NOT a rigidbody. Unlike p2d::Body, a Matter object has no
// rotation, angular velocity, torque, or moment of inertia at all: it's
// always a plain circle that translates but never spins, the same
// simplification most particle systems make (dust, sand, projectiles,
// debris -- things you don't need to see tumble). That's what makes this a
// genuinely different simulation model from a rigidbody, not just a
// cheaper rigidbody preset.
//
// A Matter object still collides with everything -- other Matter, and
// ordinary Body rigidbodies (walls, crates, ...) -- through World's own
// broadphase/narrowphase/solver, just through simpler math on its side of
// each contact (no moment-arm term, since it never rotates). See
// World::step() and Collision.cpp's generateMatterContact()/
// generateMatterBodyContact(). It CAN still impart angular impulse to a
// Body it hits (a rigidbody's own side of the contact keeps its usual
// r-cross-impulse term) -- it just never spins itself in response.
//
// Deliberately out of scope for this first version (all differences from
// Body, not bugs): always simulated (no Static/Kinematic equivalent -- a
// particle that never moves has little reason to exist), always a circle
// (no polygon Matter -- a shape needs orientation to make polygon corners
// meaningful, which Matter has none of), and no script/UI-Element/spring-
// joint hosting the way Body has. Extend here if any of that turns out to
// be wanted later.
class Matter {
public:
    int id = -1;
    std::string name;

    Vec2 position;
    Vec2 velocity;
    Vec2 force;

    float radius = 0.5f;
    float density = 1.0f;
    float mass = 0.0f;
    float invMass = 0.0f;

    float restitution = 0.3f;
    float friction = 0.3f;

    // Per-second velocity decay, applied each step as
    // velocity *= 1 / (1 + linearDamping * dt). 0 = no drag.
    float linearDamping = 0.0f;

    // Multiplies World::gravity for this particle only.
    float gravityScale = 1.0f;

    // Sleeping: same idea as Body::isAwake/sleepTime, but linear-only --
    // there's no angular velocity to also check, since Matter never
    // rotates. See World::updateSleepState()'s Matter pass.
    bool isAwake = true;
    float sleepTime = 0.0f;

    // Scratch flag used internally by World::updateSleepState() to
    // coordinate sleeping between touching bodies/particles within a
    // single step, mirroring Body::sleepReady -- not meaningful outside
    // that function.
    bool sleepReady = false;

    MatterKind kind = MatterKind::Matter;

    // Viewport display color (RGB, 0-255) -- same fixed-once-set convention
    // as Body::colorR/G/B.
    uint8_t colorR = 220, colorG = 140, colorB = 60;

    // Path to an image file to render on this particle instead of a flat
    // color fill -- same convention as Body::texturePath.
    std::string texturePath;

    // Recomputes mass from the current radius + density (area of a circle).
    // Call again after changing either.
    void computeMass();

    void applyForce(const Vec2& f) {
        force += f;
        wake();
    }
    void applyLinearImpulse(const Vec2& impulse) {
        velocity += impulse * invMass;
        wake();
    }

    // Marks the particle as awake and resets its sleep timer. Called
    // automatically by the apply*() methods above.
    void wake() {
        isAwake = true;
        sleepTime = 0.0f;
    }
};

} // namespace p2d
