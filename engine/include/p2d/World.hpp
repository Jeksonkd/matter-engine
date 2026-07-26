#pragma once

#include "p2d/BroadPhase.hpp"
#include "p2d/Body.hpp"
#include "p2d/Collision.hpp"
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

    int computeSubstepCount(float dt) const;
    void integrateForces(float dt);
    void broadAndNarrowPhase();
    void solveVelocities();
    void integrateVelocities(float dt);
    void correctPositions();
    void updateSleepState(float dt);
};

} // namespace p2d
