#include "p2d/Body.hpp"

namespace p2d {

void Body::computeMass() {
    if (type != BodyType::Dynamic) {
        mass = 0.0f;
        invMass = 0.0f;
        inertia = 0.0f;
        invInertia = 0.0f;
        return;
    }

    MassData md = shape.computeMass(density);
    mass = md.mass;
    invMass = (mass > 1e-9f) ? 1.0f / mass : 0.0f;

    if (fixedRotation) {
        inertia = 0.0f;
        invInertia = 0.0f;
    } else {
        inertia = md.inertia;
        invInertia = (inertia > 1e-9f) ? 1.0f / inertia : 0.0f;
    }
}

void Body::applyForceAtPoint(const Vec2& f, const Vec2& worldPoint) {
    force += f;
    Vec2 r = worldPoint - position;
    torque += r.cross(f);
    wake();
}

// NOTE: deliberately does not call wake() -- this is the solver's hot-path
// primitive (called every velocity iteration, every contact point, both
// bodies), and by the time the solver touches a contact both bodies are
// already guaranteed awake (World wakes sleeping bodies when a new contact
// with an awake/kinematic body is generated, before solving runs). The
// public, script-facing impulse/force methods above wake on use instead.
void Body::applyImpulse(const Vec2& impulse, const Vec2& contactVectorFromCenter) {
    velocity += impulse * invMass;
    angularVelocity += invInertia * contactVectorFromCenter.cross(impulse);
}

Vec2 Body::toWorldPoint(const Vec2& local) const {
    return position + rotate(local, rotation);
}

Vec2 Body::toWorldVector(const Vec2& localVec) const {
    return rotate(localVec, rotation);
}

} // namespace p2d
