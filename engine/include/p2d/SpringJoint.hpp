#pragma once

#include "p2d/Body.hpp"

namespace p2d {

// Applies one step's Hooke's-law-plus-damping spring force directly between
// two bodies, via Body::applyForce. This is the one place the actual spring
// math lives -- both SpringJoint (below) and p2d::SoftBody's internal
// particle-to-particle springs call this, so "what a spring does" only
// needs to be right in one place.
void applyHookeSpring(Body& a, Body& b, float restLength, float stiffness, float damping);

// A single spring connecting two arbitrary, already-existing Bodies -- the
// same force model p2d::SoftBody uses between its own generated particles,
// but generalized to any two Body*, of any shape or BodyType, already
// placed in the scene. This is what lets a script or the editor "tie
// objects together" (the ask that motivated this) rather than being
// limited to a SoftBody's own auto-generated particle cluster: select any
// two ordinary bodies and connect them, without replacing either one.
struct SpringJoint {
    Body* a = nullptr;
    Body* b = nullptr;
    float restLength = 0.0f;
    float stiffness = 0.0f; // N per meter of stretch
    float damping = 0.0f;   // N per (m/s) of closing velocity, along the spring axis

    // MUST be called from World::onPreSubstep, not once before World::step()
    // -- see World::onPreSubstep's doc comment for why: step() can subdivide
    // dt into several internal substeps, and a force set before step() is
    // only actually seen by the first of those.
    void applyForce() const {
        if (a && b) applyHookeSpring(*a, *b, restLength, stiffness, damping);
    }
};

} // namespace p2d
