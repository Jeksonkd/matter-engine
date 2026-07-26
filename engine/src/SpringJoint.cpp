#include "p2d/SpringJoint.hpp"

namespace p2d {

void applyHookeSpring(Body& a, Body& b, float restLength, float stiffness, float damping) {
    Vec2 delta = b.position - a.position;
    float len = delta.length();
    if (len < 1e-6f) return;
    Vec2 dir = delta * (1.0f / len);

    float stretch = len - restLength;
    float closingSpeed = (b.velocity - a.velocity).dot(dir);
    float forceMag = stiffness * stretch + damping * closingSpeed;

    Vec2 force = dir * forceMag;
    a.applyForce(force);
    b.applyForce(-force);
}

} // namespace p2d
