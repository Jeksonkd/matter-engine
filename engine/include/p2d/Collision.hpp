#pragma once

#include "p2d/Body.hpp"
#include "p2d/Matter.hpp"
#include "p2d/Vec2.hpp"

namespace p2d {

struct AABB {
    Vec2 min;
    Vec2 max;
};

// Contact manifold between two bodies. `normal` always points from a to b.
// Up to 2 points are produced for polygon-polygon contacts (a stable edge
// contact); circle contacts always produce exactly 1.
struct Contact {
    Body* a = nullptr;
    Body* b = nullptr;
    Vec2 normal;
    Vec2 points[2];
    float penetration[2] = {0.0f, 0.0f};
    int count = 0;
    float restitution = 0.0f;
    float friction = 0.0f;

    // True if either body is a sensor: the contact is still generated (and
    // World::onContact still fires) but the solver skips impulse resolution
    // for it, so sensors detect overlap without physically colliding.
    bool isSensor = false;

    // Accumulated impulse magnitudes per point, along `normal` and along the
    // fixed tangent (normal.perp()) respectively. World carries these over
    // from the previous step's matching contact (warm starting) so the
    // solver starts near the converged solution instead of from zero every
    // frame -- the key technique for stable stacking without needing many
    // more velocity iterations. Zero-initialized for a brand new contact.
    float normalImpulse[2] = {0.0f, 0.0f};
    float tangentImpulse[2] = {0.0f, 0.0f};
};

// Contact between two Matter particles -- always exactly one point (both
// sides are circles), and no moment-arm/angular terms at all on either
// side, since neither particle rotates. `normal` points from a to b.
struct MatterContact {
    Matter* a = nullptr;
    Matter* b = nullptr;
    Vec2 normal;
    Vec2 point;
    float penetration = 0.0f;
    float restitution = 0.0f;
    float friction = 0.0f;

    // Warm-starting accumulators, same idea as Contact::normalImpulse/
    // tangentImpulse -- a single point, since two circles only ever
    // produce one.
    float normalImpulse = 0.0f;
    float tangentImpulse = 0.0f;
};

// Contact between a Matter particle and an ordinary Body (rigidbody).
// `normal` points from the particle towards the body. The particle's side
// of the solver has no moment-arm term (it never rotates), but the body's
// side keeps its usual one -- a Matter particle CAN still impart angular
// impulse to a Body it hits, it just never spins itself in response.
struct MatterBodyContact {
    Matter* matter = nullptr;
    Body* body = nullptr;
    Vec2 normal;
    Vec2 point;
    float penetration = 0.0f;
    float restitution = 0.0f;
    float friction = 0.0f;
    bool isSensor = false; // true if the Body side is a sensor (Matter has no isSensor of its own)

    float normalImpulse = 0.0f;
    float tangentImpulse = 0.0f;
};

namespace collision {

AABB computeAABB(const Body& body);
AABB computeMatterAABB(const Matter& m);
bool aabbOverlap(const AABB& a, const AABB& b);

// Populates `out` (a and b are taken from the given bodies, in that order)
// and returns true if the two bodies are currently touching.
//
// `prevNormalHint`, if given, is the previous step's contact normal for this
// same body pair (World passes its warm-starting data through here). It's
// only consulted to break an otherwise-ambiguous reference-face tie in
// polygon-polygon SAT (see polygonVsPolygon's use of it in Collision.cpp) --
// two flush, exactly-stacked boxes have both candidate separations sitting
// right at 0, so ordinary numerical noise (from position correction, etc.)
// can flip which face wins the tie from one step to the next. Since the
// fixed geometric tangent used by the velocity solver's friction term is
// derived from the contact normal, that flip alternates the *sign* of the
// warm-started tangent impulse it's matched against, which showed up as a
// tiny (~1e-4), never-settling oscillation for ordinary box-on-box stacks.
bool generateContact(Body& a, Body& b, Contact& out, const Vec2* prevNormalHint = nullptr);

// Matter-vs-Matter and Matter-vs-Body narrowphase -- both reuse the exact
// same circle-circle/circle-polygon math generateContact() uses for a
// Body's own circle shapes (see Collision.cpp), just fed a bare position
// and radius instead of a Body, since Matter has no rotation to matter for
// either.
bool generateMatterContact(Matter& a, Matter& b, MatterContact& out);
bool generateMatterBodyContact(Matter& matter, Body& body, MatterBodyContact& out);

} // namespace collision
} // namespace p2d
