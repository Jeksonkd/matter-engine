#pragma once

#include "p2d/BroadPhase.hpp"
#include "p2d/Body.hpp"
#include "p2d/Collision.hpp"
#include "p2d/Joint.hpp"
#include "p2d/Matter.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

namespace p2d {

class World {
public:
    Vec2 gravity{0.0f, -9.81f};
    int velocityIterations = 8;

    // --- Sleeping (Dynamic bodies only) --------------------------------
    bool allowSleeping = true;
    float linearSleepThreshold = 0.05f;  // m/s
    float angularSleepThreshold = 0.05f; // rad/s
    float timeToSleep = 0.5f;            // seconds below threshold before sleeping

    // --- Adaptive substepping (mitigates tunneling for fast bodies) ----
    // If a body would move more than this fraction of its own (smaller)
    // half-extent in one step, the step is subdivided so no single substep
    // moves it further than that -- capped at maxSubsteps for a worst-case
    // performance bound.
    float continuousDisplacementFraction = 0.5f;
    int maxSubsteps = 8;

    // --- Continuous collision detection (circle movers only, opt-in) ---
    // Adaptive substepping above mitigates tunneling by shrinking the step
    // size, but an extreme enough speed vs. a thin enough wall can still
    // outrun it (substeps are capped at maxSubsteps for a worst-case
    // performance bound) -- see the README's "Known limitations." This is
    // a genuine (not just finer-substepped) swept time-of-impact check,
    // layered on top: after integrateVelocities() moves a Dynamic circle
    // Body or a Matter particle, if it moved further this substep than
    // ccdMinMovementFraction * its own radius, its straight-line path is
    // swept (collision::raycastCircle/raycastPolygon) against every other
    // Body/Matter, and if that path crosses something's surface, its
    // position is pulled back to just short of the first impact point --
    // so the very next step's ordinary discrete narrowphase finds a small,
    // sane overlap to resolve normally, instead of the object having
    // already passed clean through. Deliberately scoped to circle movers
    // only (not polygon movers) -- a swept-POLYGON-vs-polygon time-of-impact
    // test is a substantially larger undertaking, and a fast circle
    // (bullet, particle, ball) is the overwhelmingly common tunneling
    // case in practice. The target being swept against is treated as
    // stationary for the duration of the sweep (not itself continuously
    // moving) -- correct for a fast mover vs. anything slow/static (the
    // common case), an approximation for two things both moving very fast
    // at once.
    //
    // OFF by default (enableCcd = false) and skips any mover whose own
    // matterKind/kind is OptiMatter even when on -- this check has no
    // broadphase acceleration of its own (it sweeps against EVERY other
    // Body/Matter, unconditionally), so turning it on on a scene with many
    // simultaneously-fast-moving circles (hundreds freefalling at once, say)
    // is a real O(n)-per-mover cost, not a free accuracy upgrade. Confirmed
    // via tests/broadphase_test.cpp's 2000-body performance budget, which
    // regressed roughly 5x when this was briefly unconditional -- exactly
    // the scenario this default-off/OptiMatter-exempt scoping avoids. Turn
    // it on for scenes with a modest number of fast movers (a few bullets,
    // not a few hundred) that specifically need the stronger guarantee.
    bool enableCcd = false;
    float ccdMinMovementFraction = 0.5f;

    // Caps how far correctPositions() will nudge a single contact point
    // apart in one step (meters), regardless of how deep the penetration
    // is. Without this, a body spawned heavily overlapping another (e.g.
    // via a script placing shapes without checking what's already there)
    // resolves a large *fraction* of a large penetration every step --
    // still bounded in principle, but with more substeps per rendered
    // frame (up to maxSubsteps of them), that fraction compounds across
    // many corrections within a single
    // visible frame, snapping bodies apart almost instantly and looking
    // like an explosion. Matches Box2D's b2_maxLinearCorrection, which
    // exists for exactly this reason.
    float maxLinearCorrection = 0.2f;

    // --- Matter-family fidelity dial (see MatterKind in Matter.hpp) --
    // anything (Body OR Matter) whose matterKind/kind is OptiMatter always
    // uses AT LEAST this loose a sleep threshold/timeToSleep and this large
    // a position-correction cap, regardless of the World-wide settings
    // above. optiMatterAngularSleepThreshold only ever applies to Body --
    // Matter (the particle class) has no rotation to begin with. A plain
    // Body/Matter left at MatterKind::Rigidbody is entirely unaffected by
    // any of these.
    float optiMatterLinearSleepThreshold = 0.3f;
    float optiMatterAngularSleepThreshold = 0.35f;
    float optiMatterTimeToSleep = 0.1f;
    float optiMatterMaxLinearCorrection = 0.6f;

    // --- MatterKind::Matter's OWN dial, pulling the opposite direction from
    // OptiMatter above: something (Body OR Matter) marked plain Matter uses
    // AT MOST this tight a sleep threshold, AT LEAST this long a
    // timeToSleep, AT MOST this small a position-correction cap, and AT
    // MOST this small a continuous-displacement fraction (so it forces
    // substeps sooner than an ordinary Rigidbody body/particle moving at
    // the same speed would) -- strictly more accurate than the World-wide
    // defaults above, not just "not cheapened" like Rigidbody.
    // matterAngularSleepThreshold only ever applies to Body, same
    // reasoning as its Opti counterpart.
    float matterLinearSleepThreshold = 0.02f;
    float matterAngularSleepThreshold = 0.02f;
    float matterTimeToSleep = 0.8f;
    float matterMaxLinearCorrection = 0.1f;
    float matterContinuousDisplacementFraction = 0.25f;

    Body* createCircle(Vec2 pos, float radius, BodyType type = BodyType::Dynamic);
    Body* createBox(Vec2 pos, float halfWidth, float halfHeight, BodyType type = BodyType::Dynamic);
    Body* createBody(ShapeData shape, Vec2 pos, BodyType type = BodyType::Dynamic);

    void removeBody(Body* body);
    Body* findByName(const std::string& name) const;
    const std::vector<std::unique_ptr<Body>>& bodies() const { return bodies_; }

    // Matter particles -- see Matter.hpp for what makes these genuinely
    // different from a Body (no rotation/torque/inertia at all, always a
    // circle, always simulated).
    Matter* createMatter(Vec2 pos, float radius, MatterKind kind = MatterKind::Matter);
    void removeMatter(Matter* m);
    Matter* findMatterByName(const std::string& name) const;
    const std::vector<std::unique_ptr<Matter>>& matter() const { return matter_; }

    void step(float dt);

    void clear();

    // A single raycast/query result -- exactly one of `body`/`matter` is
    // non-null, whichever kind of thing was actually hit.
    struct RaycastResult {
        Body* body = nullptr;
        Matter* matter = nullptr;
        Vec2 point;
        Vec2 normal;
        float fraction = 0.0f; // 0..1 along the origin->target segment
    };

    // Casts a ray from `origin` to `target` against every Body and Matter
    // particle in the world, and reports the CLOSEST hit (smallest
    // fraction), if any. `mask` is checked against each candidate's own
    // collisionCategory (same bitwise rule as ordinary collision filtering,
    // see Body::collisionCategory) -- the default (0xFFFF) hits anything.
    // A ray starting inside a shape never reports a hit for that shape (see
    // collision::raycastCircle/raycastPolygon).
    bool raycastClosest(Vec2 origin, Vec2 target, RaycastResult& outResult, uint16_t mask = 0xFFFF) const;

    // Same cast, but every hit along the segment, sorted by fraction
    // (nearest first) rather than just the closest one.
    std::vector<RaycastResult> raycastAll(Vec2 origin, Vec2 target, uint16_t mask = 0xFFFF) const;

    // The topmost (last-created, i.e. last in bodies()) Body whose shape
    // contains `point`, or nullptr if none does -- the same point-in-shape
    // test EditorApp's own click-to-select uses, generalized for scripts.
    Body* queryPoint(Vec2 point, uint16_t mask = 0xFFFF) const;

    // --- Rigid joints (see Joint.hpp) -- all four take a WORLD-space
    // anchor point (or two, for DistanceJoint, plus its own length),
    // converted once to each body's local space at creation time. Returned
    // pointers stay valid until removed (stable storage, like Body*/Matter*)
    // -- pass one to the matching remove*Joint() below to delete it.
    DistanceJoint* createDistanceJoint(Body* a, Body* b, Vec2 worldAnchorA, Vec2 worldAnchorB);
    RevoluteJoint* createRevoluteJoint(Body* a, Body* b, Vec2 worldAnchor);
    WeldJoint* createWeldJoint(Body* a, Body* b, Vec2 worldAnchor);
    // `worldAxis` need not be normalized; it's normalized internally.
    PrismaticJoint* createPrismaticJoint(Body* a, Body* b, Vec2 worldAnchor, Vec2 worldAxis);

    void removeDistanceJoint(DistanceJoint* j);
    void removeRevoluteJoint(RevoluteJoint* j);
    void removeWeldJoint(WeldJoint* j);
    void removePrismaticJoint(PrismaticJoint* j);

    const std::vector<std::unique_ptr<DistanceJoint>>& distanceJoints() const { return distanceJoints_; }
    const std::vector<std::unique_ptr<RevoluteJoint>>& revoluteJoints() const { return revoluteJoints_; }
    const std::vector<std::unique_ptr<WeldJoint>>& weldJoints() const { return weldJoints_; }
    const std::vector<std::unique_ptr<PrismaticJoint>>& prismaticJoints() const { return prismaticJoints_; }

    // Called once per contact point that is currently touching, right after
    // narrowphase, before the impulse solver runs. Useful for gameplay/script
    // hooks (e.g. counting collisions). Optional.
    std::function<void(const Contact&)> onContact;

    // Called just before a body is erased from removeBody(), while it's
    // still valid. Lets a host (e.g. the editor's ScriptEngine) detach any
    // script attachment keyed by that body's id. Optional.
    std::function<void(Body*)> onBodyRemoved;

    // Same idea as onBodyRemoved, for removeMatter(). Optional.
    std::function<void(Matter*)> onMatterRemoved;

    // Called at the start of every internal substep (see step()'s adaptive
    // substepping), before integrateForces() consumes each body's
    // accumulated `force`. Exists for hosts applying *continuous* custom
    // forces every step -- e.g. SoftBody's Hooke's-law springs -- rather
    // than a one-off impulse: calling applyForce() once before step(dt) is
    // NOT equivalent to this, because a fast-moving body can make step()
    // subdivide dt into several substeps internally, and a force set before
    // step() is only seen by integrateForces() on the *first* substep (it's
    // consumed and zeroed there) -- every later substep would otherwise see
    // stale geometry and zero force. Recomputing from onPreSubstep instead
    // keeps a continuous force's magnitude consistent with the actual
    // substep dt and current positions. (Discovered via
    // tests/soft_body_test.cpp: applying spring forces once per outer
    // step() call let the ring gain energy indefinitely instead of
    // settling, once particle speeds got fast enough to trigger
    // subdivision.)
    std::function<void(float subDt)> onPreSubstep;

private:
    // Per-contact-point data for the velocity solver: geometry and effective
    // mass terms that only depend on (unchanging-mid-solve) positions, so
    // they're computed once per step and reused across all velocityIterations
    // passes -- both a correctness detail (matches how the bias/mass terms
    // are meant to work) and a performance one (no redundant cross products).
    struct PointConstraint {
        Vec2 rA, rB;
        float normalMass = 0.0f;
        float tangentMass = 0.0f;
        float velocityBias = 0.0f;
    };
    struct ContactConstraint {
        Vec2 tangent; // fixed per contact (normal.perp()), not velocity-derived
        PointConstraint points[2];
    };

    // Matter-Matter: no angular terms at all on either side (neither
    // particle rotates), so there's no per-point rA/rB moment arm to
    // track -- just the fixed tangent and the two scalar masses/bias.
    struct MatterConstraint {
        Vec2 tangent;
        float normalMass = 0.0f;
        float tangentMass = 0.0f;
        float velocityBias = 0.0f;
    };
    // Matter-Body: the Body side keeps its usual moment arm (rB) since it
    // can still rotate in response; the Matter side contributes none (it
    // never rotates, hence no rA here at all -- contrast PointConstraint's
    // rA/rB pair above, used when BOTH sides can rotate).
    struct MatterBodyConstraint {
        Vec2 tangent;
        Vec2 rB;
        float normalMass = 0.0f;
        float tangentMass = 0.0f;
        float velocityBias = 0.0f;
    };

    std::vector<std::unique_ptr<Body>> bodies_;
    std::vector<Contact> contacts_;
    std::unordered_map<uint64_t, Contact> prevContacts_; // for warm-starting, keyed by body-id pair
    SpatialHashGrid broadphase_;
    std::vector<AABB> aabbCache_; // one per (bodies_ + matter_) entry, refreshed each substep -- see broadAndNarrowPhase()
    std::vector<ContactConstraint> constraintCache_;  // one per contacts_ entry, reused (not reallocated) each step
    int nextId_ = 0;

    std::vector<std::unique_ptr<Matter>> matter_;
    std::vector<MatterContact> matterContacts_;
    std::vector<MatterBodyContact> matterBodyContacts_;
    std::unordered_map<uint64_t, MatterContact> prevMatterContacts_;         // keyed by matter-id pair
    std::unordered_map<uint64_t, MatterBodyContact> prevMatterBodyContacts_; // keyed by (matterId, bodyId)
    std::vector<MatterConstraint> matterConstraintCache_;
    std::vector<MatterBodyConstraint> matterBodyConstraintCache_;
    int nextMatterId_ = 0;

    // Joint constraint caches -- same idea as ContactConstraint above
    // (geometry/effective-mass terms computed once per step, reused across
    // every velocityIterations pass), one struct per joint TYPE since each
    // constrains different DOF (see Joint.hpp).
    struct DistanceJointConstraint {
        Vec2 rA, rB;
        Vec2 u; // unit vector from A's anchor to B's anchor
        float mass = 0.0f;
    };
    struct RevoluteJointConstraint {
        Vec2 rA, rB;
        float k11 = 0.0f, k12 = 0.0f, k22 = 0.0f; // 2x2 effective mass matrix (symmetric)
    };
    struct WeldJointConstraint {
        Vec2 rA, rB;
        float k11 = 0.0f, k12 = 0.0f, k22 = 0.0f;
        float angleMass = 0.0f;
    };
    struct PrismaticJointConstraint {
        Vec2 axis, perp;
        float s1 = 0.0f, s2 = 0.0f; // perpendicular-constraint Jacobian terms, see Joint.hpp
        float perpMass = 0.0f;
        float angleMass = 0.0f;
    };

    std::vector<std::unique_ptr<DistanceJoint>> distanceJoints_;
    std::vector<std::unique_ptr<RevoluteJoint>> revoluteJoints_;
    std::vector<std::unique_ptr<WeldJoint>> weldJoints_;
    std::vector<std::unique_ptr<PrismaticJoint>> prismaticJoints_;
    std::vector<DistanceJointConstraint> distanceJointConstraintCache_;
    std::vector<RevoluteJointConstraint> revoluteJointConstraintCache_;
    std::vector<WeldJointConstraint> weldJointConstraintCache_;
    std::vector<PrismaticJointConstraint> prismaticJointConstraintCache_;

    int computeSubstepCount(float dt) const;
    void integrateForces(float dt);
    void broadAndNarrowPhase();
    void solveVelocities();
    void solveJointVelocities();
    void integrateVelocities(float dt);
    void applyCcd(float dt);
    void correctPositions();
    void correctJointPositions();
    void updateSleepState(float dt);
};

} // namespace p2d
