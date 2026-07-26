#pragma once

#include "p2d/Vec2.hpp"
#include <vector>

namespace p2d {

enum class ShapeType { Circle, Polygon };

// Mass properties expressed about the shape's own local origin.
struct MassData {
    float mass = 0.0f;
    float inertia = 0.0f; // about local origin
    Vec2 centroid;
};

// A convex shape in local (body) space. Polygon winding is always CCW and
// normals[i] corresponds to the outward normal of the edge from vertices[i]
// to vertices[i + 1].
struct ShapeData {
    ShapeType type = ShapeType::Circle;

    float radius = 0.5f; // Circle only

    std::vector<Vec2> vertices; // Polygon only, local space, CCW
    std::vector<Vec2> normals;  // Polygon only, matches vertices

    static ShapeData MakeCircle(float radius);
    static ShapeData MakeBox(float halfWidth, float halfHeight);
    static ShapeData MakePolygon(std::vector<Vec2> localVerts);
    // A regular N-gon (sides >= 3) of the given circumradius, centered on
    // the local origin, CCW-wound to match MakePolygon's expectations.
    // Covers triangles/pentagons/hexagons/etc. with one function rather
    // than a named one per shape -- the SAT narrowphase already handles
    // polygon-polygon collision for any convex shape, so nothing beyond
    // building the vertices is needed to support a new regular polygon.
    static ShapeData MakeRegularPolygon(int sides, float circumradius);

    MassData computeMass(float density) const;

    // Axis-aligned half-extents of the shape in local space (used to build
    // a world-space AABB together with the body's transform).
    Vec2 localHalfExtents() const;
};

} // namespace p2d
