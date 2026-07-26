#include "p2d/World.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace p2d {

namespace {

bool isEffectivelyStatic(const Body& b) {
    return b.type == BodyType::Static || (b.type == BodyType::Dynamic && !b.isAwake);
}

// Box2D-style category/mask filtering: two things collide only if EACH
// side's category is present in the OTHER's mask. Symmetric on purpose --
// either side can veto the pair.
bool shouldCollide(uint16_t categoryA, uint16_t maskA, uint16_t categoryB, uint16_t maskB) {
    return (categoryA & maskB) != 0 && (categoryB & maskA) != 0;
}

uint64_t pairKey(int idA, int idB) {
    if (idA > idB) std::swap(idA, idB);
    return (static_cast<uint64_t>(static_cast<uint32_t>(idA)) << 32) | static_cast<uint32_t>(idB);
}

} // namespace

Body* World::createCircle(Vec2 pos, float radius, BodyType type) {
    return createBody(ShapeData::MakeCircle(radius), pos, type);
}

Body* World::createBox(Vec2 pos, float halfWidth, float halfHeight, BodyType type) {
    return createBody(ShapeData::MakeBox(halfWidth, halfHeight), pos, type);
}

Body* World::createBody(ShapeData shape, Vec2 pos, BodyType type) {
    auto body = std::make_unique<Body>();
    body->id = nextId_++;
    body->shape = std::move(shape);
    body->position = pos;
    body->type = type;
    body->computeMass();

    // Just an initial value -- see Body::colorR/G/B's doc comment. Matches
    // what Renderer::colorForBody used to compute on the fly from BodyType
    // alone, so freshly spawned bodies still look the same as before by
    // default; from here on it's an ordinary editable property, not
    // recomputed from type or sleep state on every frame.
    switch (type) {
        case BodyType::Static: body->colorR = 130; body->colorG = 130; body->colorB = 130; break;
        case BodyType::Kinematic: body->colorR = 215; body->colorG = 150; body->colorB = 60; break;
        case BodyType::Dynamic: body->colorR = 80; body->colorG = 150; body->colorB = 220; break;
    }

    Body* ptr = body.get();
    bodies_.push_back(std::move(body));
    return ptr;
}

void World::removeBody(Body* body) {
    if (onBodyRemoved) onBodyRemoved(body);

    // Drop any joint touching this body first -- otherwise it's left
    // holding a dangling Body* the next time a joint solve runs.
    auto touches = [body](Body* a, Body* b) { return a == body || b == body; };
    distanceJoints_.erase(std::remove_if(distanceJoints_.begin(), distanceJoints_.end(),
                                          [&](const std::unique_ptr<DistanceJoint>& p) {
                                              return touches(p->a, p->b);
                                          }),
                          distanceJoints_.end());
    revoluteJoints_.erase(std::remove_if(revoluteJoints_.begin(), revoluteJoints_.end(),
                                          [&](const std::unique_ptr<RevoluteJoint>& p) {
                                              return touches(p->a, p->b);
                                          }),
                          revoluteJoints_.end());
    weldJoints_.erase(
        std::remove_if(weldJoints_.begin(), weldJoints_.end(),
                        [&](const std::unique_ptr<WeldJoint>& p) { return touches(p->a, p->b); }),
        weldJoints_.end());
    prismaticJoints_.erase(std::remove_if(prismaticJoints_.begin(), prismaticJoints_.end(),
                                           [&](const std::unique_ptr<PrismaticJoint>& p) {
                                               return touches(p->a, p->b);
                                           }),
                           prismaticJoints_.end());

    bodies_.erase(
        std::remove_if(bodies_.begin(), bodies_.end(),
                        [body](const std::unique_ptr<Body>& p) { return p.get() == body; }),
        bodies_.end());
}

Body* World::findByName(const std::string& name) const {
    for (auto& b : bodies_) {
        if (b->name == name) return b.get();
    }
    return nullptr;
}

Matter* World::createMatter(Vec2 pos, float radius, MatterKind kind) {
    auto m = std::make_unique<Matter>();
    m->id = nextMatterId_++;
    m->position = pos;
    m->radius = radius;
    m->kind = kind;
    m->computeMass();

    Matter* ptr = m.get();
    matter_.push_back(std::move(m));
    return ptr;
}

void World::removeMatter(Matter* m) {
    if (onMatterRemoved) onMatterRemoved(m);
    matter_.erase(
        std::remove_if(matter_.begin(), matter_.end(), [m](const std::unique_ptr<Matter>& p) { return p.get() == m; }),
        matter_.end());
}

Matter* World::findMatterByName(const std::string& name) const {
    for (auto& m : matter_) {
        if (m->name == name) return m.get();
    }
    return nullptr;
}

DistanceJoint* World::createDistanceJoint(Body* a, Body* b, Vec2 worldAnchorA, Vec2 worldAnchorB) {
    auto j = std::make_unique<DistanceJoint>();
    j->a = a;
    j->b = b;
    j->localAnchorA = rotate(worldAnchorA - a->position, -a->rotation);
    j->localAnchorB = rotate(worldAnchorB - b->position, -b->rotation);
    j->length = (worldAnchorB - worldAnchorA).length();
    a->wake();
    b->wake();
    DistanceJoint* ptr = j.get();
    distanceJoints_.push_back(std::move(j));
    return ptr;
}

RevoluteJoint* World::createRevoluteJoint(Body* a, Body* b, Vec2 worldAnchor) {
    auto j = std::make_unique<RevoluteJoint>();
    j->a = a;
    j->b = b;
    j->localAnchorA = rotate(worldAnchor - a->position, -a->rotation);
    j->localAnchorB = rotate(worldAnchor - b->position, -b->rotation);
    a->wake();
    b->wake();
    RevoluteJoint* ptr = j.get();
    revoluteJoints_.push_back(std::move(j));
    return ptr;
}

WeldJoint* World::createWeldJoint(Body* a, Body* b, Vec2 worldAnchor) {
    auto j = std::make_unique<WeldJoint>();
    j->a = a;
    j->b = b;
    j->localAnchorA = rotate(worldAnchor - a->position, -a->rotation);
    j->localAnchorB = rotate(worldAnchor - b->position, -b->rotation);
    j->referenceAngle = b->rotation - a->rotation;
    a->wake();
    b->wake();
    WeldJoint* ptr = j.get();
    weldJoints_.push_back(std::move(j));
    return ptr;
}

PrismaticJoint* World::createPrismaticJoint(Body* a, Body* b, Vec2 worldAnchor, Vec2 worldAxis) {
    auto j = std::make_unique<PrismaticJoint>();
    j->a = a;
    j->b = b;
    j->localAnchorA = rotate(worldAnchor - a->position, -a->rotation);
    j->localAnchorB = rotate(worldAnchor - b->position, -b->rotation);
    j->localAxisA = rotate(worldAxis.normalized(), -a->rotation);
    j->referenceAngle = b->rotation - a->rotation;
    a->wake();
    b->wake();
    PrismaticJoint* ptr = j.get();
    prismaticJoints_.push_back(std::move(j));
    return ptr;
}

void World::removeDistanceJoint(DistanceJoint* j) {
    distanceJoints_.erase(
        std::remove_if(distanceJoints_.begin(), distanceJoints_.end(),
                        [j](const std::unique_ptr<DistanceJoint>& p) { return p.get() == j; }),
        distanceJoints_.end());
}
void World::removeRevoluteJoint(RevoluteJoint* j) {
    revoluteJoints_.erase(
        std::remove_if(revoluteJoints_.begin(), revoluteJoints_.end(),
                        [j](const std::unique_ptr<RevoluteJoint>& p) { return p.get() == j; }),
        revoluteJoints_.end());
}
void World::removeWeldJoint(WeldJoint* j) {
    weldJoints_.erase(std::remove_if(weldJoints_.begin(), weldJoints_.end(),
                                      [j](const std::unique_ptr<WeldJoint>& p) { return p.get() == j; }),
                       weldJoints_.end());
}
void World::removePrismaticJoint(PrismaticJoint* j) {
    prismaticJoints_.erase(
        std::remove_if(prismaticJoints_.begin(), prismaticJoints_.end(),
                        [j](const std::unique_ptr<PrismaticJoint>& p) { return p.get() == j; }),
        prismaticJoints_.end());
}

void World::clear() {
    bodies_.clear();
    contacts_.clear();
    prevContacts_.clear();
    nextId_ = 0;

    matter_.clear();
    matterContacts_.clear();
    matterBodyContacts_.clear();
    prevMatterContacts_.clear();
    prevMatterBodyContacts_.clear();
    nextMatterId_ = 0;

    distanceJoints_.clear();
    revoluteJoints_.clear();
    weldJoints_.clear();
    prismaticJoints_.clear();
}

bool World::raycastClosest(Vec2 origin, Vec2 target, RaycastResult& outResult, uint16_t mask) const {
    Vec2 dir = target - origin;
    bool found = false;
    float bestFraction = 1.0f;

    for (auto& bp : bodies_) {
        Body& b = *bp;
        if ((b.collisionCategory & mask) == 0) continue;
        collision::RaycastHit hit;
        bool got = (b.shape.type == ShapeType::Circle) ? collision::raycastCircle(origin, dir, b.position, b.shape.radius, hit)
                                                        : collision::raycastPolygon(origin, dir, b, hit);
        if (got && hit.fraction < bestFraction) {
            bestFraction = hit.fraction;
            outResult.body = &b;
            outResult.matter = nullptr;
            outResult.point = hit.point;
            outResult.normal = hit.normal;
            outResult.fraction = hit.fraction;
            found = true;
        }
    }
    for (auto& mp : matter_) {
        Matter& m = *mp;
        if ((m.collisionCategory & mask) == 0) continue;
        collision::RaycastHit hit;
        if (collision::raycastCircle(origin, dir, m.position, m.radius, hit) && hit.fraction < bestFraction) {
            bestFraction = hit.fraction;
            outResult.body = nullptr;
            outResult.matter = &m;
            outResult.point = hit.point;
            outResult.normal = hit.normal;
            outResult.fraction = hit.fraction;
            found = true;
        }
    }
    return found;
}

std::vector<World::RaycastResult> World::raycastAll(Vec2 origin, Vec2 target, uint16_t mask) const {
    Vec2 dir = target - origin;
    std::vector<RaycastResult> results;

    for (auto& bp : bodies_) {
        Body& b = *bp;
        if ((b.collisionCategory & mask) == 0) continue;
        collision::RaycastHit hit;
        bool got = (b.shape.type == ShapeType::Circle) ? collision::raycastCircle(origin, dir, b.position, b.shape.radius, hit)
                                                        : collision::raycastPolygon(origin, dir, b, hit);
        if (got) results.push_back({&b, nullptr, hit.point, hit.normal, hit.fraction});
    }
    for (auto& mp : matter_) {
        Matter& m = *mp;
        if ((m.collisionCategory & mask) == 0) continue;
        collision::RaycastHit hit;
        if (collision::raycastCircle(origin, dir, m.position, m.radius, hit)) {
            results.push_back({nullptr, &m, hit.point, hit.normal, hit.fraction});
        }
    }
    std::sort(results.begin(), results.end(), [](const RaycastResult& a, const RaycastResult& b) {
        return a.fraction < b.fraction;
    });
    return results;
}

Body* World::queryPoint(Vec2 point, uint16_t mask) const {
    // Last-created (topmost) match wins -- same "iterate in reverse, first
    // hit wins" convention EditorApp::pickBodyAt() already uses for click
    // selection, so a script's point query matches what clicking there
    // would select.
    for (auto it = bodies_.rbegin(); it != bodies_.rend(); ++it) {
        Body& b = **it;
        if ((b.collisionCategory & mask) == 0) continue;
        if (collision::pointInBody(b, point)) return &b;
    }
    return nullptr;
}

int World::computeSubstepCount(float dt) const {
    // Per-object, not one shared ratio: a plain Matter body/particle uses
    // matterContinuousDisplacementFraction (tighter than the World-wide
    // default), so it can force substeps a Rigidbody object moving at the
    // same speed wouldn't need -- more accurate, not just "not cheapened".
    int neededSubsteps = 1;
    for (auto& bp : bodies_) {
        const Body& b = *bp;
        if (b.type != BodyType::Dynamic || !b.isAwake) continue;
        // OptiMatter never forces extra substeps -- see Body::matterKind.
        if (b.matterKind == MatterKind::OptiMatter) continue;

        float speed = b.velocity.length();
        if (speed <= 0.0f) continue;

        Vec2 half = b.shape.localHalfExtents();
        float extent = std::max(0.02f, std::min(half.x, half.y));
        float ratio = (speed * dt) / extent;
        float fraction =
            (b.matterKind == MatterKind::Matter) ? matterContinuousDisplacementFraction : continuousDisplacementFraction;
        if (ratio > fraction) {
            neededSubsteps = std::max(neededSubsteps, static_cast<int>(std::ceil(ratio / fraction)));
        }
    }
    for (auto& mp : matter_) {
        const Matter& m = *mp;
        if (!m.isAwake) continue;
        // OptiMatter never forces extra substeps -- see Matter::kind.
        if (m.kind == MatterKind::OptiMatter) continue;

        float speed = m.velocity.length();
        if (speed <= 0.0f) continue;

        float extent = std::max(0.02f, m.radius);
        float ratio = (speed * dt) / extent;
        float fraction =
            (m.kind == MatterKind::Matter) ? matterContinuousDisplacementFraction : continuousDisplacementFraction;
        if (ratio > fraction) {
            neededSubsteps = std::max(neededSubsteps, static_cast<int>(std::ceil(ratio / fraction)));
        }
    }

    return std::max(1, std::min(neededSubsteps, maxSubsteps));
}

void World::step(float dt) {
    if (dt <= 0.0f) return;

    int substeps = computeSubstepCount(dt);
    float subDt = dt / static_cast<float>(substeps);

    for (int i = 0; i < substeps; ++i) {
        if (onPreSubstep) onPreSubstep(subDt);
        integrateForces(subDt);
        broadAndNarrowPhase();
        solveVelocities();
        solveJointVelocities();
        integrateVelocities(subDt);
        applyCcd(subDt);
        correctPositions();
        correctJointPositions();
        updateSleepState(subDt);

        prevContacts_.clear();
        for (auto& c : contacts_) {
            prevContacts_[pairKey(c.a->id, c.b->id)] = c;
        }
        prevMatterContacts_.clear();
        for (auto& c : matterContacts_) {
            prevMatterContacts_[pairKey(c.a->id, c.b->id)] = c;
        }
        prevMatterBodyContacts_.clear();
        for (auto& c : matterBodyContacts_) {
            prevMatterBodyContacts_[pairKey(c.matter->id, c.body->id)] = c;
        }
    }
}

void World::integrateForces(float dt) {
    for (auto& bp : bodies_) {
        Body& b = *bp;
        if (b.type != BodyType::Dynamic || !b.isAwake) {
            b.force = Vec2();
            b.torque = 0.0f;
            continue;
        }
        b.velocity += dt * (gravity * b.gravityScale + b.force * b.invMass);
        b.angularVelocity += dt * b.invInertia * b.torque;

        // Simple velocity-proportional drag: velocity *= 1 / (1 + damping*dt).
        b.velocity *= 1.0f / (1.0f + dt * b.linearDamping);
        b.angularVelocity *= 1.0f / (1.0f + dt * b.angularDamping);

        b.force = Vec2();
        b.torque = 0.0f;
    }

    for (auto& mp : matter_) {
        Matter& m = *mp;
        if (!m.isAwake) {
            m.force = Vec2();
            continue;
        }
        m.velocity += dt * (gravity * m.gravityScale + m.force * m.invMass);
        m.velocity *= 1.0f / (1.0f + dt * m.linearDamping);
        m.force = Vec2();
    }
}

void World::broadAndNarrowPhase() {
    contacts_.clear();
    matterContacts_.clear();
    matterBodyContacts_.clear();

    // One combined AABB list -- bodies_ first, then matter_ -- run through a
    // SINGLE broadphase pass so Body-Body, Matter-Matter, AND Body-Matter
    // pairs are all found together, rather than needing three separate grid
    // builds. computePairs() hands back index pairs into exactly this list
    // (see BroadPhase.hpp), so an index >= bodies_.size() unambiguously
    // means "matter_[index - bodies_.size()]".
    size_t bodyCount = bodies_.size();
    size_t matterCount = matter_.size();
    aabbCache_.resize(bodyCount + matterCount);
    for (size_t i = 0; i < bodyCount; ++i) {
        aabbCache_[i] = collision::computeAABB(*bodies_[i]);
    }
    for (size_t i = 0; i < matterCount; ++i) {
        aabbCache_[bodyCount + i] = collision::computeMatterAABB(*matter_[i]);
    }

    // Cell size ~2x the scene's average extent across everything (bodies
    // AND matter): cells sized for a fixed-but-wrong "typical" size would
    // either cram many small things into one cell (degrading toward
    // O(cluster^2) within it) or scatter large ones across many cells for
    // no benefit. Safe to recompute every step now -- the grid does a full
    // clear+rebuild each call regardless (see BroadPhase.hpp), so there's
    // no stale-key accumulation risk from the cell size changing between
    // steps.
    float cellSize = 1.0f;
    if (!aabbCache_.empty()) {
        float totalExtent = 0.0f;
        for (const auto& box : aabbCache_) {
            totalExtent += (box.max.x - box.min.x) + (box.max.y - box.min.y);
        }
        float avgExtent = totalExtent / (static_cast<float>(aabbCache_.size()) * 2.0f);
        cellSize = std::max(0.1f, avgExtent * 2.0f);
    }

    broadphase_.computePairs(aabbCache_, cellSize, [&](int i, int j) {
        bool iIsMatter = static_cast<size_t>(i) >= bodyCount;
        bool jIsMatter = static_cast<size_t>(j) >= bodyCount;

        if (!iIsMatter && !jIsMatter) {
            // --- Body-Body: unchanged from before Matter existed. -------
            Body& a = *bodies_[static_cast<size_t>(i)];
            Body& b = *bodies_[static_cast<size_t>(j)];

            // Two things that can't move relative to each other don't need
            // (re-)testing -- this is both a performance shortcut and what
            // lets sleeping bodies stay asleep instead of being woken by
            // their own resting contacts every step.
            if (isEffectivelyStatic(a) && isEffectivelyStatic(b)) return;

            if (!shouldCollide(a.collisionCategory, a.collisionMask, b.collisionCategory, b.collisionMask)) return;

            // Looked up before generating the new contact so polygon-polygon
            // SAT can use last step's normal to break an otherwise-ambiguous
            // reference-face tie (see generateContact's doc comment) -- keeps
            // a flush box-on-box contact's normal/tangent from flipping
            // direction on nothing more than sub-epsilon numerical noise.
            auto it = prevContacts_.find(pairKey(a.id, b.id));
            const Vec2* prevNormalHint =
                (it != prevContacts_.end() && it->second.count > 0) ? &it->second.normal : nullptr;

            Contact c;
            if (!collision::generateContact(a, b, c, prevNormalHint)) return;

            // Wake sleeping dynamic bodies touched by something that can
            // move (an awake dynamic body or a script/kinematic-driven one).
            if (a.type == BodyType::Dynamic && !a.isAwake) a.wake();
            if (b.type == BodyType::Dynamic && !b.isAwake) b.wake();

            // Warm start: carry over accumulated impulses from the previous
            // step's matching contact for this body pair, matched point-by-
            // point by nearest position. This is a simplification of
            // Box2D's feature-ID matching, but effective: contact points
            // move only a little frame to frame, so nearest-position
            // matching finds the right correspondence in practice.
            if (it != prevContacts_.end()) {
                const Contact& old = it->second;
                for (int p = 0; p < c.count; ++p) {
                    float bestDistSq = 0.05f * 0.05f;
                    int bestIdx = -1;
                    for (int q = 0; q < old.count; ++q) {
                        float d = (c.points[p] - old.points[q]).lengthSq();
                        if (d < bestDistSq) {
                            bestDistSq = d;
                            bestIdx = q;
                        }
                    }
                    if (bestIdx >= 0) {
                        c.normalImpulse[p] = old.normalImpulse[bestIdx];
                        c.tangentImpulse[p] = old.tangentImpulse[bestIdx];
                    }
                }
            }

            contacts_.push_back(c);
            if (onContact) onContact(c);
        } else if (iIsMatter && jIsMatter) {
            // --- Matter-Matter: always exactly one contact point (both
            // sides are circles), so no nearest-point matching needed for
            // warm-starting -- a direct lookup is unambiguous. -----------
            Matter& a = *matter_[static_cast<size_t>(i) - bodyCount];
            Matter& b = *matter_[static_cast<size_t>(j) - bodyCount];

            if (!a.isAwake && !b.isAwake) return;
            if (!shouldCollide(a.collisionCategory, a.collisionMask, b.collisionCategory, b.collisionMask)) return;

            MatterContact c;
            if (!collision::generateMatterContact(a, b, c)) return;

            if (!a.isAwake) a.wake();
            if (!b.isAwake) b.wake();

            auto it = prevMatterContacts_.find(pairKey(a.id, b.id));
            if (it != prevMatterContacts_.end()) {
                c.normalImpulse = it->second.normalImpulse;
                c.tangentImpulse = it->second.tangentImpulse;
            }

            matterContacts_.push_back(c);
        } else {
            // --- Matter-Body (mixed): normalize so `m` is always the
            // Matter side regardless of which of i/j it was. -------------
            Matter& m = iIsMatter ? *matter_[static_cast<size_t>(i) - bodyCount]
                                  : *matter_[static_cast<size_t>(j) - bodyCount];
            Body& b = iIsMatter ? *bodies_[static_cast<size_t>(j)] : *bodies_[static_cast<size_t>(i)];

            if (isEffectivelyStatic(b) && !m.isAwake) return;
            if (!shouldCollide(m.collisionCategory, m.collisionMask, b.collisionCategory, b.collisionMask)) return;

            MatterBodyContact c;
            if (!collision::generateMatterBodyContact(m, b, c)) return;

            if (!m.isAwake) m.wake();
            if (b.type == BodyType::Dynamic && !b.isAwake) b.wake();

            auto it = prevMatterBodyContacts_.find(pairKey(m.id, b.id));
            if (it != prevMatterBodyContacts_.end()) {
                c.normalImpulse = it->second.normalImpulse;
                c.tangentImpulse = it->second.tangentImpulse;
            }

            matterBodyContacts_.push_back(c);
        }
    });
}

void World::solveVelocities() {
    const float restitutionThreshold = 1.0f;

    // Reused across steps (resize keeps existing capacity -- no repeated
    // heap allocation once the scene's contact count stabilizes).
    constraintCache_.resize(contacts_.size());
    auto& constraints = constraintCache_;

    // --- Init: fixed geometry/mass terms (positions don't change during
    // this phase, so these stay valid across every iteration below), and
    // each point's restitution bias computed from the CURRENT (pre-warm-
    // start) relative velocity. -------------------------------------------
    //
    // NOTE: velocityBias here is restitution ONLY -- no positional/Baumgarte
    // term. An earlier version folded positional correction in here too,
    // which is wrong once warm-starting is involved: whatever bias impulse
    // was needed to correct penetration this step got accumulated into
    // c.normalImpulse and then warm-started into the *next* step, where a
    // fresh bias gets added on top if the penetration hadn't fully resolved
    // yet -- a slow compounding energy leak (a settled stack's velocity
    // visibly *grew* over several seconds instead of decaying to zero).
    // Positional correction is instead handled by the separate, unaccumulated
    // correctPositions() pass below, exactly so it can never leak into
    // warm-started velocity like this.
    for (size_t ci = 0; ci < contacts_.size(); ++ci) {
        Contact& c = contacts_[ci];
        if (c.isSensor) continue;

        Body& A = *c.a;
        Body& B = *c.b;
        ContactConstraint& cc = constraints[ci];
        cc.tangent = c.normal.perp(); // fixed per contact, NOT velocity-derived

        for (int p = 0; p < c.count; ++p) {
            PointConstraint& pc = cc.points[p];
            pc.rA = c.points[p] - A.position;
            pc.rB = c.points[p] - B.position;

            float raCrossN = pc.rA.cross(c.normal);
            float rbCrossN = pc.rB.cross(c.normal);
            float normalMassSum = A.invMass + B.invMass + raCrossN * raCrossN * A.invInertia +
                                   rbCrossN * rbCrossN * B.invInertia;
            pc.normalMass = normalMassSum > 0.0f ? 1.0f / normalMassSum : 0.0f;

            float raCrossT = pc.rA.cross(cc.tangent);
            float rbCrossT = pc.rB.cross(cc.tangent);
            float tangentMassSum = A.invMass + B.invMass + raCrossT * raCrossT * A.invInertia +
                                    rbCrossT * rbCrossT * B.invInertia;
            pc.tangentMass = tangentMassSum > 0.0f ? 1.0f / tangentMassSum : 0.0f;

            Vec2 relVel = (B.velocity + cross(B.angularVelocity, pc.rB)) -
                          (A.velocity + cross(A.angularVelocity, pc.rA));
            float velAlongNormal = relVel.dot(c.normal);
            pc.velocityBias = (velAlongNormal < -restitutionThreshold) ? -c.restitution * velAlongNormal : 0.0f;
        }
    }

    // Warm-started impulses are applied in their OWN pass, strictly after
    // every contact's velocityBias above has been computed -- NOT
    // interleaved into the same loop. Interleaving them (the original
    // code: compute bias, then immediately apply that contact's warm
    // start, contact by contact) meant each contact's warm-start impulse
    // was already perturbing the bodies' velocities by the time LATER
    // contacts in the list computed their OWN relVel/bias -- for a small
    // number of simultaneous contacts (an ordinary stack) that perturbation
    // is negligible, but for hundreds of bodies packed densely enough that
    // one body touches many neighbors at once (e.g. spawning ~1000 circles
    // into an already-settled ~1000-circle pile), it fed a real feedback
    // loop: contaminated relVel -> spurious restitution bias -> a bigger
    // warm-started impulse next step -> more contamination. Confirmed via
    // a throwaway /tmp diagnostic that the bug's signature was "gets WORSE
    // with MORE velocityIterations" (proof it was a correctness bug, not
    // an insufficient-convergence one -- more iterations just applied the
    // same wrong bias more forcefully) and that it only manifested with
    // restitution > 0 (a bias of exactly 0 can't be contaminated). Fully
    // *removing* warm-starting also fixes it, but was rejected: it breaks
    // ordinary box-stacking (tests/stacking_test.cpp), which is what warm-
    // starting exists for in the first place.
    for (size_t ci = 0; ci < contacts_.size(); ++ci) {
        Contact& c = contacts_[ci];
        if (c.isSensor) continue;

        Body& A = *c.a;
        Body& B = *c.b;
        ContactConstraint& cc = constraints[ci];

        for (int p = 0; p < c.count; ++p) {
            PointConstraint& pc = cc.points[p];
            if (pc.normalMass <= 0.0f) continue;

            Vec2 warmImpulse = c.normal * c.normalImpulse[p] + cc.tangent * c.tangentImpulse[p];
            A.applyImpulse(-1.0f * warmImpulse, pc.rA);
            B.applyImpulse(warmImpulse, pc.rB);
        }
    }

    // --- Iterate: accumulate impulses, clamped each step (standard
    // sequential-impulse solver with warm starting). --------------------
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (size_t ci = 0; ci < contacts_.size(); ++ci) {
            Contact& c = contacts_[ci];
            if (c.isSensor) continue;

            Body& A = *c.a;
            Body& B = *c.b;
            ContactConstraint& cc = constraints[ci];

            for (int p = 0; p < c.count; ++p) {
                PointConstraint& pc = cc.points[p];
                if (pc.normalMass <= 0.0f) continue;

                Vec2 relVel = (B.velocity + cross(B.angularVelocity, pc.rB)) -
                              (A.velocity + cross(A.angularVelocity, pc.rA));
                float velAlongNormal = relVel.dot(c.normal);

                float dPn = (pc.velocityBias - velAlongNormal) * pc.normalMass;
                float newImpulse = std::max(c.normalImpulse[p] + dPn, 0.0f);
                dPn = newImpulse - c.normalImpulse[p];
                c.normalImpulse[p] = newImpulse;

                Vec2 impulse = c.normal * dPn;
                A.applyImpulse(-1.0f * impulse, pc.rA);
                B.applyImpulse(impulse, pc.rB);

                // Friction along the fixed tangent, clamped to the Coulomb
                // cone using the (just-updated) accumulated normal impulse.
                relVel = (B.velocity + cross(B.angularVelocity, pc.rB)) -
                         (A.velocity + cross(A.angularVelocity, pc.rA));
                float velAlongTangent = relVel.dot(cc.tangent);

                float dPt = -velAlongTangent * pc.tangentMass;
                float maxFriction = c.friction * c.normalImpulse[p];
                float newTangentImpulse = std::clamp(c.tangentImpulse[p] + dPt, -maxFriction, maxFriction);
                dPt = newTangentImpulse - c.tangentImpulse[p];
                c.tangentImpulse[p] = newTangentImpulse;

                Vec2 frictionImpulse = cc.tangent * dPt;
                A.applyImpulse(-1.0f * frictionImpulse, pc.rA);
                B.applyImpulse(frictionImpulse, pc.rB);
            }
        }
    }

    // --- Matter-Matter: same sequential-impulse algorithm as Body-Body
    // above, just with every rA/rB/angular term dropped -- neither side
    // rotates, so a contact point's velocity IS the particle's velocity,
    // with no cross(angularVelocity, r) term to add. --------------------
    matterConstraintCache_.resize(matterContacts_.size());
    for (size_t ci = 0; ci < matterContacts_.size(); ++ci) {
        MatterContact& c = matterContacts_[ci];
        Matter& A = *c.a;
        Matter& B = *c.b;
        MatterConstraint& cc = matterConstraintCache_[ci];
        cc.tangent = c.normal.perp();

        float massSum = A.invMass + B.invMass;
        cc.normalMass = massSum > 0.0f ? 1.0f / massSum : 0.0f;
        cc.tangentMass = cc.normalMass; // identical formula with no angular terms to differ by

        Vec2 relVel = B.velocity - A.velocity;
        float velAlongNormal = relVel.dot(c.normal);
        cc.velocityBias = (velAlongNormal < -restitutionThreshold) ? -c.restitution * velAlongNormal : 0.0f;
    }
    for (size_t ci = 0; ci < matterContacts_.size(); ++ci) {
        MatterContact& c = matterContacts_[ci];
        MatterConstraint& cc = matterConstraintCache_[ci];
        if (cc.normalMass <= 0.0f) continue;

        Vec2 warmImpulse = c.normal * c.normalImpulse + cc.tangent * c.tangentImpulse;
        c.a->velocity -= warmImpulse * c.a->invMass;
        c.b->velocity += warmImpulse * c.b->invMass;
    }
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (size_t ci = 0; ci < matterContacts_.size(); ++ci) {
            MatterContact& c = matterContacts_[ci];
            MatterConstraint& cc = matterConstraintCache_[ci];
            if (cc.normalMass <= 0.0f) continue;
            Matter& A = *c.a;
            Matter& B = *c.b;

            Vec2 relVel = B.velocity - A.velocity;
            float velAlongNormal = relVel.dot(c.normal);
            float dPn = (cc.velocityBias - velAlongNormal) * cc.normalMass;
            float newImpulse = std::max(c.normalImpulse + dPn, 0.0f);
            dPn = newImpulse - c.normalImpulse;
            c.normalImpulse = newImpulse;

            Vec2 impulse = c.normal * dPn;
            A.velocity -= impulse * A.invMass;
            B.velocity += impulse * B.invMass;

            relVel = B.velocity - A.velocity;
            float velAlongTangent = relVel.dot(cc.tangent);
            float dPt = -velAlongTangent * cc.tangentMass;
            float maxFriction = c.friction * c.normalImpulse;
            float newTangentImpulse = std::clamp(c.tangentImpulse + dPt, -maxFriction, maxFriction);
            dPt = newTangentImpulse - c.tangentImpulse;
            c.tangentImpulse = newTangentImpulse;

            Vec2 frictionImpulse = cc.tangent * dPt;
            A.velocity -= frictionImpulse * A.invMass;
            B.velocity += frictionImpulse * B.invMass;
        }
    }

    // --- Matter-Body: the Body side keeps its usual moment arm/rotation
    // response; the Matter side never rotates, so it only ever gets a
    // plain linear velocity change -- it can still torque the Body it hit
    // (via the Body's own rB-cross-impulse term), it just never spins
    // itself in response. ------------------------------------------------
    matterBodyConstraintCache_.resize(matterBodyContacts_.size());
    for (size_t ci = 0; ci < matterBodyContacts_.size(); ++ci) {
        MatterBodyContact& c = matterBodyContacts_[ci];
        if (c.isSensor) continue;

        Matter& m = *c.matter;
        Body& body = *c.body;
        MatterBodyConstraint& cc = matterBodyConstraintCache_[ci];
        cc.tangent = c.normal.perp();
        cc.rB = c.point - body.position;

        float rbCrossN = cc.rB.cross(c.normal);
        float normalMassSum = m.invMass + body.invMass + rbCrossN * rbCrossN * body.invInertia;
        cc.normalMass = normalMassSum > 0.0f ? 1.0f / normalMassSum : 0.0f;

        float rbCrossT = cc.rB.cross(cc.tangent);
        float tangentMassSum = m.invMass + body.invMass + rbCrossT * rbCrossT * body.invInertia;
        cc.tangentMass = tangentMassSum > 0.0f ? 1.0f / tangentMassSum : 0.0f;

        Vec2 velAtPointB = body.velocity + cross(body.angularVelocity, cc.rB);
        Vec2 relVel = velAtPointB - m.velocity;
        float velAlongNormal = relVel.dot(c.normal);
        cc.velocityBias = (velAlongNormal < -restitutionThreshold) ? -c.restitution * velAlongNormal : 0.0f;
    }
    for (size_t ci = 0; ci < matterBodyContacts_.size(); ++ci) {
        MatterBodyContact& c = matterBodyContacts_[ci];
        if (c.isSensor) continue;
        MatterBodyConstraint& cc = matterBodyConstraintCache_[ci];
        if (cc.normalMass <= 0.0f) continue;

        Vec2 warmImpulse = c.normal * c.normalImpulse + cc.tangent * c.tangentImpulse;
        c.matter->velocity -= warmImpulse * c.matter->invMass;
        c.body->applyImpulse(warmImpulse, cc.rB);
    }
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (size_t ci = 0; ci < matterBodyContacts_.size(); ++ci) {
            MatterBodyContact& c = matterBodyContacts_[ci];
            if (c.isSensor) continue;
            MatterBodyConstraint& cc = matterBodyConstraintCache_[ci];
            if (cc.normalMass <= 0.0f) continue;

            Matter& m = *c.matter;
            Body& body = *c.body;

            Vec2 velAtPointB = body.velocity + cross(body.angularVelocity, cc.rB);
            Vec2 relVel = velAtPointB - m.velocity;
            float velAlongNormal = relVel.dot(c.normal);
            float dPn = (cc.velocityBias - velAlongNormal) * cc.normalMass;
            float newImpulse = std::max(c.normalImpulse + dPn, 0.0f);
            dPn = newImpulse - c.normalImpulse;
            c.normalImpulse = newImpulse;

            Vec2 impulse = c.normal * dPn;
            m.velocity -= impulse * m.invMass;
            body.applyImpulse(impulse, cc.rB);

            velAtPointB = body.velocity + cross(body.angularVelocity, cc.rB);
            relVel = velAtPointB - m.velocity;
            float velAlongTangent = relVel.dot(cc.tangent);
            float dPt = -velAlongTangent * cc.tangentMass;
            float maxFriction = c.friction * c.normalImpulse;
            float newTangentImpulse = std::clamp(c.tangentImpulse + dPt, -maxFriction, maxFriction);
            dPt = newTangentImpulse - c.tangentImpulse;
            c.tangentImpulse = newTangentImpulse;

            Vec2 frictionImpulse = cc.tangent * dPt;
            m.velocity -= frictionImpulse * m.invMass;
            body.applyImpulse(frictionImpulse, cc.rB);
        }
    }
}

void World::solveJointVelocities() {
    // Same three-phase structure as solveVelocities() above (init geometry/
    // effective-mass terms once, warm-start, then iterate) -- see Joint.hpp
    // for what each joint type actually constrains and why Weld/Prismatic
    // solve their two constraints as independent passes rather than one
    // coupled system.

    distanceJointConstraintCache_.resize(distanceJoints_.size());
    for (size_t i = 0; i < distanceJoints_.size(); ++i) {
        DistanceJoint& j = *distanceJoints_[i];
        Body& A = *j.a;
        Body& B = *j.b;
        DistanceJointConstraint& dc = distanceJointConstraintCache_[i];

        dc.rA = rotate(j.localAnchorA, A.rotation);
        dc.rB = rotate(j.localAnchorB, B.rotation);
        Vec2 d = (B.position + dc.rB) - (A.position + dc.rA);
        float len = d.length();
        dc.u = (len > 1e-9f) ? d * (1.0f / len) : Vec2(1.0f, 0.0f);

        float crAu = dc.rA.cross(dc.u);
        float crBu = dc.rB.cross(dc.u);
        float invMassSum = A.invMass + B.invMass + A.invInertia * crAu * crAu + B.invInertia * crBu * crBu;
        dc.mass = invMassSum > 0.0f ? 1.0f / invMassSum : 0.0f;

        // Warm start.
        Vec2 P = dc.u * j.impulse;
        A.applyImpulse(-1.0f * P, dc.rA);
        B.applyImpulse(P, dc.rB);
    }
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (size_t i = 0; i < distanceJoints_.size(); ++i) {
            DistanceJoint& j = *distanceJoints_[i];
            DistanceJointConstraint& dc = distanceJointConstraintCache_[i];
            if (dc.mass <= 0.0f) continue;
            Body& A = *j.a;
            Body& B = *j.b;

            Vec2 relVel = (B.velocity + cross(B.angularVelocity, dc.rB)) - (A.velocity + cross(A.angularVelocity, dc.rA));
            float Cdot = relVel.dot(dc.u);
            float lambda = -dc.mass * Cdot;
            j.impulse += lambda;

            Vec2 P = dc.u * lambda;
            A.applyImpulse(-1.0f * P, dc.rA);
            B.applyImpulse(P, dc.rB);
        }
    }

    revoluteJointConstraintCache_.resize(revoluteJoints_.size());
    for (size_t i = 0; i < revoluteJoints_.size(); ++i) {
        RevoluteJoint& j = *revoluteJoints_[i];
        Body& A = *j.a;
        Body& B = *j.b;
        RevoluteJointConstraint& rc = revoluteJointConstraintCache_[i];

        rc.rA = rotate(j.localAnchorA, A.rotation);
        rc.rB = rotate(j.localAnchorB, B.rotation);
        rc.k11 = A.invMass + B.invMass + A.invInertia * rc.rA.y * rc.rA.y + B.invInertia * rc.rB.y * rc.rB.y;
        rc.k12 = -A.invInertia * rc.rA.x * rc.rA.y - B.invInertia * rc.rB.x * rc.rB.y;
        rc.k22 = A.invMass + B.invMass + A.invInertia * rc.rA.x * rc.rA.x + B.invInertia * rc.rB.x * rc.rB.x;

        A.applyImpulse(-1.0f * j.impulse, rc.rA);
        B.applyImpulse(j.impulse, rc.rB);
    }
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (size_t i = 0; i < revoluteJoints_.size(); ++i) {
            RevoluteJoint& j = *revoluteJoints_[i];
            RevoluteJointConstraint& rc = revoluteJointConstraintCache_[i];
            float det = rc.k11 * rc.k22 - rc.k12 * rc.k12;
            if (std::fabs(det) < 1e-12f) continue;
            float invDet = 1.0f / det;
            Body& A = *j.a;
            Body& B = *j.b;

            Vec2 Cdot = (B.velocity + cross(B.angularVelocity, rc.rB)) - (A.velocity + cross(A.angularVelocity, rc.rA));
            Vec2 impulse(-invDet * (rc.k22 * Cdot.x - rc.k12 * Cdot.y), -invDet * (-rc.k12 * Cdot.x + rc.k11 * Cdot.y));
            j.impulse += impulse;

            A.applyImpulse(-1.0f * impulse, rc.rA);
            B.applyImpulse(impulse, rc.rB);
        }
    }

    weldJointConstraintCache_.resize(weldJoints_.size());
    for (size_t i = 0; i < weldJoints_.size(); ++i) {
        WeldJoint& j = *weldJoints_[i];
        Body& A = *j.a;
        Body& B = *j.b;
        WeldJointConstraint& wc = weldJointConstraintCache_[i];

        wc.rA = rotate(j.localAnchorA, A.rotation);
        wc.rB = rotate(j.localAnchorB, B.rotation);
        wc.k11 = A.invMass + B.invMass + A.invInertia * wc.rA.y * wc.rA.y + B.invInertia * wc.rB.y * wc.rB.y;
        wc.k12 = -A.invInertia * wc.rA.x * wc.rA.y - B.invInertia * wc.rB.x * wc.rB.y;
        wc.k22 = A.invMass + B.invMass + A.invInertia * wc.rA.x * wc.rA.x + B.invInertia * wc.rB.x * wc.rB.x;
        float angleInvMassSum = A.invInertia + B.invInertia;
        wc.angleMass = angleInvMassSum > 0.0f ? 1.0f / angleInvMassSum : 0.0f;

        A.applyImpulse(-1.0f * j.pointImpulse, wc.rA);
        B.applyImpulse(j.pointImpulse, wc.rB);
        A.angularVelocity -= A.invInertia * j.angleImpulse;
        B.angularVelocity += B.invInertia * j.angleImpulse;
    }
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (size_t i = 0; i < weldJoints_.size(); ++i) {
            WeldJoint& j = *weldJoints_[i];
            WeldJointConstraint& wc = weldJointConstraintCache_[i];
            Body& A = *j.a;
            Body& B = *j.b;

            // Angle constraint first (decoupled from the point constraint --
            // see Joint.hpp).
            if (wc.angleMass > 0.0f) {
                float Cdot = B.angularVelocity - A.angularVelocity;
                float impulse = -wc.angleMass * Cdot;
                j.angleImpulse += impulse;
                A.angularVelocity -= A.invInertia * impulse;
                B.angularVelocity += B.invInertia * impulse;
            }

            float det = wc.k11 * wc.k22 - wc.k12 * wc.k12;
            if (std::fabs(det) < 1e-12f) continue;
            float invDet = 1.0f / det;
            Vec2 Cdot = (B.velocity + cross(B.angularVelocity, wc.rB)) - (A.velocity + cross(A.angularVelocity, wc.rA));
            Vec2 impulse(-invDet * (wc.k22 * Cdot.x - wc.k12 * Cdot.y), -invDet * (-wc.k12 * Cdot.x + wc.k11 * Cdot.y));
            j.pointImpulse += impulse;

            A.applyImpulse(-1.0f * impulse, wc.rA);
            B.applyImpulse(impulse, wc.rB);
        }
    }

    prismaticJointConstraintCache_.resize(prismaticJoints_.size());
    for (size_t i = 0; i < prismaticJoints_.size(); ++i) {
        PrismaticJoint& j = *prismaticJoints_[i];
        Body& A = *j.a;
        Body& B = *j.b;
        PrismaticJointConstraint& pc = prismaticJointConstraintCache_[i];

        Vec2 rA = rotate(j.localAnchorA, A.rotation);
        Vec2 rB = rotate(j.localAnchorB, B.rotation);
        pc.axis = rotate(j.localAxisA, A.rotation);
        pc.perp = pc.axis.perp();
        Vec2 d = (B.position + rB) - (A.position + rA);

        pc.s1 = (d + rA).cross(pc.perp);
        pc.s2 = rB.cross(pc.perp);
        float invMassSum = A.invMass + B.invMass + A.invInertia * pc.s1 * pc.s1 + B.invInertia * pc.s2 * pc.s2;
        pc.perpMass = invMassSum > 0.0f ? 1.0f / invMassSum : 0.0f;
        float angleInvMassSum = A.invInertia + B.invInertia;
        pc.angleMass = angleInvMassSum > 0.0f ? 1.0f / angleInvMassSum : 0.0f;

        Vec2 P = pc.perp * j.perpImpulse;
        float LA = j.perpImpulse * pc.s1;
        float LB = j.perpImpulse * pc.s2;
        A.velocity -= P * A.invMass;
        A.angularVelocity -= A.invInertia * LA;
        B.velocity += P * B.invMass;
        B.angularVelocity += B.invInertia * LB;
        A.angularVelocity -= A.invInertia * j.angleImpulse;
        B.angularVelocity += B.invInertia * j.angleImpulse;
    }
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (size_t i = 0; i < prismaticJoints_.size(); ++i) {
            PrismaticJoint& j = *prismaticJoints_[i];
            PrismaticJointConstraint& pc = prismaticJointConstraintCache_[i];
            Body& A = *j.a;
            Body& B = *j.b;

            if (pc.angleMass > 0.0f) {
                float Cdot = B.angularVelocity - A.angularVelocity;
                float impulse = -pc.angleMass * Cdot;
                j.angleImpulse += impulse;
                A.angularVelocity -= A.invInertia * impulse;
                B.angularVelocity += B.invInertia * impulse;
            }

            if (pc.perpMass > 0.0f) {
                float Cdot = pc.perp.dot(B.velocity - A.velocity) + pc.s2 * B.angularVelocity - pc.s1 * A.angularVelocity;
                float impulse = -pc.perpMass * Cdot;
                j.perpImpulse += impulse;

                Vec2 P = pc.perp * impulse;
                float LA = impulse * pc.s1;
                float LB = impulse * pc.s2;
                A.velocity -= P * A.invMass;
                A.angularVelocity -= A.invInertia * LA;
                B.velocity += P * B.invMass;
                B.angularVelocity += B.invInertia * LB;
            }
        }
    }
}

void World::integrateVelocities(float dt) {
    for (auto& bp : bodies_) {
        Body& b = *bp;
        if (b.type == BodyType::Static) continue;
        if (b.type == BodyType::Dynamic && !b.isAwake) continue;
        b.position += b.velocity * dt;
        b.rotation += b.angularVelocity * dt;
    }

    for (auto& mp : matter_) {
        Matter& m = *mp;
        if (!m.isAwake) continue;
        m.position += m.velocity * dt; // no rotation to integrate -- Matter never spins
    }
}

void World::applyCcd(float dt) {
    if (!enableCcd) return;

    // Sweeps `startPos -> pos` (the displacement integrateVelocities() just
    // applied) against every other Body/Matter, and pulls `pos` back to
    // just short of the first thing it would have passed clean through
    // this substep, if any -- see ccdMinMovementFraction's doc comment in
    // World.hpp for the full picture.
    auto sweepAndClamp = [this](Vec2& pos, const Vec2& velocity, float radius, uint16_t category, uint16_t mask,
                                 const Body* selfBody, const Matter* selfMatter, float subDt) {
        Vec2 disp = velocity * subDt;
        if (disp.lengthSq() < (radius * ccdMinMovementFraction) * (radius * ccdMinMovementFraction)) return;
        Vec2 startPos = pos - disp;

        float bestToi = 1.0f;
        bool hit = false;
        for (auto& bp : bodies_) {
            Body& target = *bp;
            if (&target == selfBody || target.isSensor) continue;
            if (!shouldCollide(category, mask, target.collisionCategory, target.collisionMask)) continue;
            collision::RaycastHit rh;
            bool got = (target.shape.type == ShapeType::Circle)
                           ? collision::raycastCircle(startPos, disp, target.position, target.shape.radius + radius, rh)
                           : collision::raycastPolygon(startPos, disp, target, rh, radius);
            if (got && rh.fraction < bestToi) {
                bestToi = rh.fraction;
                hit = true;
            }
        }
        for (auto& mp : matter_) {
            Matter& target = *mp;
            if (&target == selfMatter) continue;
            if (!shouldCollide(category, mask, target.collisionCategory, target.collisionMask)) continue;
            collision::RaycastHit rh;
            if (collision::raycastCircle(startPos, disp, target.position, target.radius + radius, rh) &&
                rh.fraction < bestToi) {
                bestToi = rh.fraction;
                hit = true;
            }
        }

        if (hit) {
            // Back off slightly from the exact impact fraction so the next
            // step's ordinary discrete narrowphase finds a small, sane
            // overlap to resolve normally, rather than landing exactly on
            // the boundary where floating-point noise could go either way.
            pos = startPos + disp * (bestToi * 0.95f);
        }
    };

    for (auto& bp : bodies_) {
        Body& b = *bp;
        if (b.type != BodyType::Dynamic || !b.isAwake) continue;
        if (b.shape.type != ShapeType::Circle) continue; // CCD covers circle movers only -- see World.hpp
        if (b.matterKind == MatterKind::OptiMatter) continue; // exempt, same as forced substeps
        sweepAndClamp(b.position, b.velocity, b.shape.radius, b.collisionCategory, b.collisionMask, &b, nullptr, dt);
    }
    for (auto& mp : matter_) {
        Matter& m = *mp;
        if (!m.isAwake) continue;
        if (m.kind == MatterKind::OptiMatter) continue; // exempt, same as forced substeps
        sweepAndClamp(m.position, m.velocity, m.radius, m.collisionCategory, m.collisionMask, nullptr, &m, dt);
    }
}

void World::correctPositions() {
    // Deliberately separate from solveVelocities/its warm-started impulses
    // (see the comment there): a direct, one-shot position nudge using the
    // penetration measured at narrowphase time, proportional to inverse
    // mass, with a small slop allowance and a fractional correction per
    // step so it doesn't overshoot. Recomputed fresh every step from
    // scratch -- nothing here is carried over, so it can't compound.
    const float slop = 0.005f;
    const float percent = 0.2f;

    // OptiMatter (either side) wins over Matter (either side) wins over
    // the World-wide default -- cheap-and-rough takes priority over
    // accurate-and-tight when a contact mixes kinds, same as picking the
    // rougher of two caps rather than trying to average them.
    auto pickCap = [this](MatterKind a, MatterKind b) {
        if (a == MatterKind::OptiMatter || b == MatterKind::OptiMatter) return optiMatterMaxLinearCorrection;
        if (a == MatterKind::Matter || b == MatterKind::Matter) return matterMaxLinearCorrection;
        return maxLinearCorrection;
    };

    for (auto& c : contacts_) {
        if (c.isSensor) continue;

        Body& A = *c.a;
        Body& B = *c.b;
        float invMassSum = A.invMass + B.invMass;
        if (invMassSum <= 0.0f) continue;

        float cap = pickCap(A.matterKind, B.matterKind);

        for (int p = 0; p < c.count; ++p) {
            float excess = std::max(c.penetration[p] - slop, 0.0f);
            // Clamp the ABSOLUTE amount of penetration resolved this step,
            // not just the fraction -- 20% of a huge initial overlap is
            // still huge. See maxLinearCorrection's doc comment.
            float resolvedThisStep = std::min(percent * excess, cap);
            float correctionMag = resolvedThisStep / invMassSum;
            Vec2 correction = c.normal * correctionMag;
            A.position -= correction * A.invMass;
            B.position += correction * B.invMass;
        }
    }

    for (auto& c : matterContacts_) {
        Matter& A = *c.a;
        Matter& B = *c.b;
        float invMassSum = A.invMass + B.invMass;
        if (invMassSum <= 0.0f) continue;

        float cap = pickCap(A.kind, B.kind);
        float excess = std::max(c.penetration - slop, 0.0f);
        float resolvedThisStep = std::min(percent * excess, cap);
        float correctionMag = resolvedThisStep / invMassSum;
        Vec2 correction = c.normal * correctionMag;
        A.position -= correction * A.invMass;
        B.position += correction * B.invMass;
    }

    for (auto& c : matterBodyContacts_) {
        if (c.isSensor) continue;

        Matter& m = *c.matter;
        Body& body = *c.body;
        float invMassSum = m.invMass + body.invMass;
        if (invMassSum <= 0.0f) continue;

        float cap = pickCap(m.kind, body.matterKind);
        float excess = std::max(c.penetration - slop, 0.0f);
        float resolvedThisStep = std::min(percent * excess, cap);
        float correctionMag = resolvedThisStep / invMassSum;
        Vec2 correction = c.normal * correctionMag;
        m.position -= correction * m.invMass;
        body.position += correction * body.invMass;
    }
}

void World::correctJointPositions() {
    // Same NGS-style direct position nudge as correctPositions() above
    // (recomputed fresh from current positions every step, nothing carried
    // over, so it can't compound) -- a joint's error is "how far its
    // constraint currently is from satisfied," not a contact's penetration,
    // but the technique (bias-and-cap a correction, split by invMass) is
    // the same.
    //
    // Every one of these ALSO updates rotation, not just position -- unlike
    // correctPositions()'s contacts (where a translation-only nudge is a
    // reasonable simplification), a joint's anchor is generally offset from
    // its body's center, so the same rA/rB moment-arm coupling the velocity
    // solve above already accounts for applies here too: correcting the
    // anchor's position without ALSO correcting rotation can't actually
    // converge to zero error when there's rotational leverage involved, it
    // just asymptotes at some nonzero residual instead (caught by
    // tests/joint_test.cpp initially reporting a persistent ~0.22m pin
    // error that never shrank -- exactly this bug, first shipped without
    // the rotation update below, then fixed).
    //
    // percent = 1.0 (FULL correction), deliberately NOT the 0.2 Baumgarte-
    // style fraction correctPositions() uses for contacts -- matches
    // Box2D's actual joint position solver (b2RevoluteJoint::
    // SolvePositionConstraints and friends both use the full -K^-1*C
    // impulse, no damping factor). A partial-percent correction here
    // converges GEOMETRICALLY, but to the wrong (nonzero) fixed point, not
    // to zero: the rA/rB moment-arm relationship used to derive the
    // correction is only exact for infinitesimal changes (first-order in
    // the rotation delta), so repeatedly applying a WEAKENED version of it
    // doesn't shrink the residual to nothing, it just finds whatever
    // magnitude makes "correction shrinks C" and "linearization error
    // regrows C" balance out. Confirmed empirically (a throwaway diagnostic
    // isolating just this pass, with velocity solving/gravity zeroed out
    // entirely, converged to ~0.00000 with percent=1.0 vs. plateauing at a
    // persistent ~0.22-0.27 with percent=0.2) before landing on this fix.
    const float percent = 1.0f;

    for (auto& jp : distanceJoints_) {
        DistanceJoint& j = *jp;
        Body& A = *j.a;
        Body& B = *j.b;

        Vec2 rA = rotate(j.localAnchorA, A.rotation);
        Vec2 rB = rotate(j.localAnchorB, B.rotation);
        Vec2 d = (B.position + rB) - (A.position + rA);
        float len = d.length();
        if (len < 1e-9f) continue;
        Vec2 u = d * (1.0f / len);
        float C = len - j.length;

        float crAu = rA.cross(u);
        float crBu = rB.cross(u);
        float invMassSum = A.invMass + B.invMass + A.invInertia * crAu * crAu + B.invInertia * crBu * crBu;
        if (invMassSum <= 0.0f) continue;
        float mass = 1.0f / invMassSum;

        float lambda = -mass * C * percent;
        Vec2 P = u * lambda;
        A.position -= P * A.invMass;
        A.rotation -= A.invInertia * rA.cross(P);
        B.position += P * B.invMass;
        B.rotation += B.invInertia * rB.cross(P);
    }

    for (auto& jp : revoluteJoints_) {
        RevoluteJoint& j = *jp;
        Body& A = *j.a;
        Body& B = *j.b;

        Vec2 rA = rotate(j.localAnchorA, A.rotation);
        Vec2 rB = rotate(j.localAnchorB, B.rotation);
        Vec2 C = (B.position + rB) - (A.position + rA);

        float k11 = A.invMass + B.invMass + A.invInertia * rA.y * rA.y + B.invInertia * rB.y * rB.y;
        float k12 = -A.invInertia * rA.x * rA.y - B.invInertia * rB.x * rB.y;
        float k22 = A.invMass + B.invMass + A.invInertia * rA.x * rA.x + B.invInertia * rB.x * rB.x;
        float det = k11 * k22 - k12 * k12;
        if (std::fabs(det) < 1e-12f) continue;
        float invDet = 1.0f / det;

        Vec2 P(-invDet * (k22 * C.x - k12 * C.y) * percent, -invDet * (-k12 * C.x + k11 * C.y) * percent);
        A.position -= P * A.invMass;
        A.rotation -= A.invInertia * rA.cross(P);
        B.position += P * B.invMass;
        B.rotation += B.invInertia * rB.cross(P);
    }

    for (auto& jp : weldJoints_) {
        WeldJoint& j = *jp;
        Body& A = *j.a;
        Body& B = *j.b;

        float angleInvMassSum = A.invInertia + B.invInertia;
        if (angleInvMassSum > 0.0f) {
            float angleMass = 1.0f / angleInvMassSum;
            float Cangle = (B.rotation - A.rotation) - j.referenceAngle;
            float impulse = -Cangle * angleMass * percent;
            A.rotation -= impulse * A.invInertia;
            B.rotation += impulse * B.invInertia;
        }

        Vec2 rA = rotate(j.localAnchorA, A.rotation);
        Vec2 rB = rotate(j.localAnchorB, B.rotation);
        Vec2 C = (B.position + rB) - (A.position + rA);

        float k11 = A.invMass + B.invMass + A.invInertia * rA.y * rA.y + B.invInertia * rB.y * rB.y;
        float k12 = -A.invInertia * rA.x * rA.y - B.invInertia * rB.x * rB.y;
        float k22 = A.invMass + B.invMass + A.invInertia * rA.x * rA.x + B.invInertia * rB.x * rB.x;
        float det = k11 * k22 - k12 * k12;
        if (std::fabs(det) < 1e-12f) continue;
        float invDet = 1.0f / det;

        Vec2 P(-invDet * (k22 * C.x - k12 * C.y) * percent, -invDet * (-k12 * C.x + k11 * C.y) * percent);
        A.position -= P * A.invMass;
        A.rotation -= A.invInertia * rA.cross(P);
        B.position += P * B.invMass;
        B.rotation += B.invInertia * rB.cross(P);
    }

    for (auto& jp : prismaticJoints_) {
        PrismaticJoint& j = *jp;
        Body& A = *j.a;
        Body& B = *j.b;

        float angleInvMassSum = A.invInertia + B.invInertia;
        if (angleInvMassSum > 0.0f) {
            float angleMass = 1.0f / angleInvMassSum;
            float Cangle = (B.rotation - A.rotation) - j.referenceAngle;
            float impulse = -Cangle * angleMass * percent;
            A.rotation -= impulse * A.invInertia;
            B.rotation += impulse * B.invInertia;
        }

        Vec2 rA = rotate(j.localAnchorA, A.rotation);
        Vec2 rB = rotate(j.localAnchorB, B.rotation);
        Vec2 axis = rotate(j.localAxisA, A.rotation);
        Vec2 perp = axis.perp();
        Vec2 d = (B.position + rB) - (A.position + rA);

        float s1 = (d + rA).cross(perp);
        float s2 = rB.cross(perp);
        float invMassSum = A.invMass + B.invMass + A.invInertia * s1 * s1 + B.invInertia * s2 * s2;
        if (invMassSum <= 0.0f) continue;
        float perpMass = 1.0f / invMassSum;

        float Cperp = perp.dot(d);
        float impulse = -Cperp * perpMass * percent;
        Vec2 P = perp * impulse;
        float LA = impulse * s1;
        float LB = impulse * s2;
        A.position -= P * A.invMass;
        A.rotation -= LA * A.invInertia;
        B.position += P * B.invMass;
        B.rotation += LB * B.invInertia;
    }
}

void World::updateSleepState(float dt) {
    if (!allowSleeping) {
        for (auto& bp : bodies_) {
            Body& b = *bp;
            if (b.type == BodyType::Dynamic) {
                b.isAwake = true;
                b.sleepTime = 0.0f;
            }
        }
        for (auto& mp : matter_) {
            mp->isAwake = true;
            mp->sleepTime = 0.0f;
        }
        return;
    }

    // Pass 1: update each awake dynamic body's own sleep timer from its own
    // velocity, and provisionally mark it ready to actually go to sleep if
    // that timer has passed the threshold.
    for (auto& bp : bodies_) {
        Body& b = *bp;
        if (b.type != BodyType::Dynamic || !b.isAwake) continue;

        // MatterKind-aware, mirroring Matter's pass below: an OptiMatter
        // body always uses AT LEAST its own looser thresholds/timeToSleep,
        // so it settles down sooner than a Rigidbody would under the same
        // World-wide settings; a plain Matter body pulls the opposite way
        // (AT MOST its tighter threshold, AT LEAST its longer timeToSleep),
        // so it stays awake and accurate longer instead (see
        // Body::matterKind).
        float linThresh = linearSleepThreshold;
        float angThresh = angularSleepThreshold;
        float ttSleep = timeToSleep;
        if (b.matterKind == MatterKind::OptiMatter) {
            linThresh = std::max(linThresh, optiMatterLinearSleepThreshold);
            angThresh = std::max(angThresh, optiMatterAngularSleepThreshold);
            ttSleep = std::min(ttSleep, optiMatterTimeToSleep);
        } else if (b.matterKind == MatterKind::Matter) {
            linThresh = std::min(linThresh, matterLinearSleepThreshold);
            angThresh = std::min(angThresh, matterAngularSleepThreshold);
            ttSleep = std::max(ttSleep, matterTimeToSleep);
        }

        bool slow = b.velocity.lengthSq() < linThresh * linThresh &&
                    (b.angularVelocity * b.angularVelocity) < angThresh * angThresh;
        b.sleepTime = slow ? b.sleepTime + dt : 0.0f;
        b.sleepReady = (b.sleepTime >= ttSleep);
    }

    // Same idea for Matter, linear-only (no angular term -- Matter never
    // rotates) and MatterKind-aware: an OptiMatter particle always uses AT
    // LEAST its own looser threshold/timeToSleep, so it settles down and
    // stops costing anything sooner than a plain one would under the same
    // World-wide settings; MatterKind::Matter pulls the opposite way (see
    // Matter::kind).
    for (auto& mp : matter_) {
        Matter& m = *mp;
        if (!m.isAwake) continue;

        float linThresh = linearSleepThreshold;
        float ttSleep = timeToSleep;
        if (m.kind == MatterKind::OptiMatter) {
            linThresh = std::max(linThresh, optiMatterLinearSleepThreshold);
            ttSleep = std::min(ttSleep, optiMatterTimeToSleep);
        } else if (m.kind == MatterKind::Matter) {
            linThresh = std::min(linThresh, matterLinearSleepThreshold);
            ttSleep = std::max(ttSleep, matterTimeToSleep);
        }

        bool slow = m.velocity.lengthSq() < linThresh * linThresh;
        m.sleepTime = slow ? m.sleepTime + dt : 0.0f;
        m.sleepReady = (m.sleepTime >= ttSleep);
    }

    // Pass 2: a body touching another still-active dynamic body doesn't
    // actually sleep yet this step, even if its own timer says it's ready.
    // Without this, two bodies resting on each other whose sleep timers
    // cross the threshold even a step or two apart take turns forever: the
    // first one sleeps, then gets immediately woken right back up next step
    // (contact generation always wakes a sleeping body touched by anything
    // that isn't ALSO asleep -- isEffectivelyStatic() needs BOTH sides
    // asleep to skip re-testing), and each hard sleep -> wake velocity
    // reset (velocity snapped to zero, then one step of unimpeded gravity
    // before the solver catches it again) perturbs the warm-started contact
    // impulses. That perturbation is exactly what showed up as an ordinary
    // two-box stack's friction never settling -- a small, sign-flipping
    // tangentImpulse jitter recurring every ~timeToSleep seconds forever
    // (found by hooking World::onContact and logging tangentImpulse/
    // isAwake/sleepTime together: the flips landed in a burst immediately
    // after a body's isAwake flipped 1->0->1 within two consecutive steps).
    //
    // This is a cheap, single-step approximation of "island" sleeping
    // (every mainstream engine sleeps a connected group together, not
    // body-by-body) using this step's already-computed contact list rather
    // than a full connected-component pass -- an ordinary pair or short
    // stack converges within this same step; a long chain may take a few
    // extra steps to fully propagate, which just delays sleep slightly and
    // is far cheaper than iterating contacts_ to a fixed point every step.
    for (auto& c : contacts_) {
        if (c.isSensor) continue;
        bool aIsAwakeDynamic = c.a->type == BodyType::Dynamic && c.a->isAwake;
        bool bIsAwakeDynamic = c.b->type == BodyType::Dynamic && c.b->isAwake;
        if (!aIsAwakeDynamic || !bIsAwakeDynamic) continue;
        if (!c.a->sleepReady) c.b->sleepReady = false;
        if (!c.b->sleepReady) c.a->sleepReady = false;
    }

    // Same coordination, extended to Matter-Matter and Matter-Body contacts
    // -- a Matter particle touching a still-active partner (Matter or Body)
    // doesn't sleep yet either, same reasoning as above.
    for (auto& c : matterContacts_) {
        if (!c.a->isAwake || !c.b->isAwake) continue;
        if (!c.a->sleepReady) c.b->sleepReady = false;
        if (!c.b->sleepReady) c.a->sleepReady = false;
    }
    for (auto& c : matterBodyContacts_) {
        if (c.isSensor) continue;
        if (!c.matter->isAwake) continue;
        bool bodyIsAwakeDynamic = c.body->type == BodyType::Dynamic && c.body->isAwake;
        if (!bodyIsAwakeDynamic) continue;
        if (!c.matter->sleepReady) c.body->sleepReady = false;
        if (!c.body->sleepReady) c.matter->sleepReady = false;
    }

    for (auto& bp : bodies_) {
        Body& b = *bp;
        if (b.type != BodyType::Dynamic || !b.isAwake) continue;
        if (b.sleepReady) {
            b.isAwake = false;
            b.velocity = Vec2(0.0f, 0.0f);
            b.angularVelocity = 0.0f;
        }
    }
    for (auto& mp : matter_) {
        Matter& m = *mp;
        if (!m.isAwake) continue;
        if (m.sleepReady) {
            m.isAwake = false;
            m.velocity = Vec2(0.0f, 0.0f);
        }
    }
}

} // namespace p2d
