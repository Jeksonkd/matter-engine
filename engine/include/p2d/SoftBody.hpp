#pragma once

#include "p2d/World.hpp"

#include <vector>

namespace p2d {

// A soft (deformable) body approximated as a ring of small circular
// rigid-body "particles" connected by spring constraints -- the classic
// mass-spring soft body technique. This is deliberately *not* a from-scratch
// deformable-body solver: each particle is a completely ordinary Dynamic
// circle Body, so it collides with the rest of the world using the exact
// same broadphase/narrowphase/impulse solver as everything else, for free.
// SoftBody only adds Hooke's-law spring forces between particles on top of
// that -- edge springs hold the ring's perimeter together, and diagonal
// cross-braces (skip-one neighbors) resist the ring folding flat, which a
// ring of edge springs alone cannot do.
//
// This trades away things a "real" deformable-body solver (FEM, position-
// based dynamics/XPBD) would give you -- exact volume preservation, no
// possibility of self-intersection, stiffer convergence at large time steps
// -- in exchange for reusing the entire existing rigid-body pipeline
// unchanged. For a 2D editor toy/prototyping tool that's the right trade;
// it would not be for a AAA cloth/jelly simulation.
class SoftBody {
public:
    struct Spring {
        int a = 0, b = 0; // indices into `particles`
        float restLength = 0.0f;
        float stiffness = 0.0f; // N per meter of stretch
        float damping = 0.0f;   // N per (m/s) of closing velocity, along the spring axis
    };

    std::vector<Body*> particles;
    std::vector<Spring> springs;

    // Applies this step's spring forces to `particles` via Body::applyForce.
    // MUST be called from World::onPreSubstep, not once before World::step(dt)
    // -- see onPreSubstep's doc comment in World.hpp for why: step() can
    // internally subdivide dt into several substeps for fast-moving bodies,
    // and a force set before step() is only actually seen by the first of
    // those, silently under-applying (and destabilizing) a *continuous*
    // force like a spring on every later substep.
    void applySpringForces();
};

// Builds a soft body: `segments` particles evenly spaced around a ring of
// the given radius, centered at `center`, all created in `world` as
// ordinary small Dynamic circles. Connects each particle to its immediate
// neighbor (perimeter springs) and to the neighbor two steps away (cross
// braces, so the ring resists collapsing flat).
//
// The stiffness/damping/particleRadius defaults were tuned empirically
// (tests/soft_body_test.cpp) against two distinct numerical-stability
// concerns of explicit (Hooke's-law-via-applyForce) spring integration:
// stiffness*dt^2/mass has to stay well under the ~4 bound where a
// mass-spring oscillator's amplitude grows every step instead of decaying
// (a particle can carry ~4 springs at once -- 2 edge + 2 cross-brace -- so
// the *effective* stiffness on one particle is well above any single
// spring's own value); and, separately, the damping term has its OWN much
// tighter stability bound (empirically found well under damping*dt/mass < 2
// for this discretization) -- overshooting *that* one doesn't just fail to
// settle, it blows up to NaN within a second. Both were confirmed via a
// throwaway modified World copy that first added the onPreSubstep hook, to
// isolate this from the (separate, now-fixed) missing-onPreSubstep bug.
SoftBody makeSoftBodyRing(World& world, Vec2 center, float radius, int segments, float particleRadius = 0.15f,
                          float stiffness = 50.0f, float damping = 2.0f);

// A "jelly" blob: the same ring as makeSoftBodyRing, plus one extra hub
// particle at `center` connected to every ring particle by a spoke spring.
// The spokes are what make this a filled, volume-resisting blob rather than
// a hollow ring -- squash it and the hub pushes back outward through every
// spoke at once, instead of the ring being free to fold in on empty space.
// The hub is deliberately given a larger radius/mass than the ring
// particles (it carries `segments` springs at once, more than any single
// ring particle does, so it needs more inertia to stay inside the same
// numerical stability margin -- see makeSoftBodyRing's doc comment).
// particles[0] is the hub; particles[1..] are the ring, in the same order
// makeSoftBodyRing would produce.
SoftBody makeSoftBodyJelly(World& world, Vec2 center, float radius, int segments, float particleRadius = 0.15f,
                           float stiffness = 50.0f, float damping = 2.0f);

// A rectangular cloth: a `cols` x `rows` grid of particles spaced `spacing`
// apart, top-left corner at `topLeft`. Structural springs connect each
// particle to its right and below neighbor; shear springs connect each to
// its diagonal neighbors (resists the grid collapsing into a degenerate
// shape the way a grid of structural springs alone can). If `pinTop` is
// true, every particle in the top row is created Static instead of Dynamic
// -- springs still act on a Static body's neighbors normally, but a Static
// body itself never moves, so the cloth hangs and drapes from that pinned
// edge instead of just falling as a whole.
SoftBody makeSoftBodyCloth(World& world, Vec2 topLeft, int cols, int rows, float spacing = 0.4f,
                           float particleRadius = 0.15f, float stiffness = 50.0f, float damping = 2.0f,
                           bool pinTop = true);

} // namespace p2d
