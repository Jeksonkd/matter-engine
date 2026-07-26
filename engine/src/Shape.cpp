#include "p2d/Shape.hpp"

#include <algorithm>
#include <cmath>

namespace p2d {

ShapeData ShapeData::MakeCircle(float radius) {
    ShapeData s;
    s.type = ShapeType::Circle;
    s.radius = radius;
    return s;
}

ShapeData ShapeData::MakeBox(float halfWidth, float halfHeight) {
    std::vector<Vec2> verts = {
        {-halfWidth, -halfHeight},
        { halfWidth, -halfHeight},
        { halfWidth,  halfHeight},
        {-halfWidth,  halfHeight},
    };
    return MakePolygon(std::move(verts));
}

ShapeData ShapeData::MakeRegularPolygon(int sides, float circumradius) {
    if (sides < 3) sides = 3;
    constexpr float kPi = 3.14159265358979323846f;

    std::vector<Vec2> verts;
    verts.reserve(static_cast<size_t>(sides));
    for (int i = 0; i < sides; ++i) {
        // Increasing angle order is CCW (matches MakePolygon's expectations).
        float angle = (static_cast<float>(i) / static_cast<float>(sides)) * 2.0f * kPi;
        verts.emplace_back(circumradius * std::cos(angle), circumradius * std::sin(angle));
    }
    return MakePolygon(std::move(verts));
}

ShapeData ShapeData::MakePolygon(std::vector<Vec2> localVerts) {
    ShapeData s;
    s.type = ShapeType::Polygon;
    s.vertices = std::move(localVerts);
    s.normals.resize(s.vertices.size());
    size_t n = s.vertices.size();
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        Vec2 edge = s.vertices[j] - s.vertices[i];
        // outward normal for a CCW-wound polygon is the edge rotated -90deg
        s.normals[i] = Vec2(edge.y, -edge.x).normalized();
    }
    return s;
}

MassData ShapeData::computeMass(float density) const {
    MassData md;

    if (type == ShapeType::Circle) {
        md.mass = density * 3.14159265358979323846f * radius * radius;
        md.inertia = md.mass * radius * radius * 0.5f; // about centroid == local origin
        md.centroid = Vec2(0.0f, 0.0f);
        return md;
    }

    // Polygon mass properties: standard triangle-fan decomposition about an
    // arbitrary reference point (vertices[0]), then re-expressed about the
    // shape's local origin. Same approach used by Box2D's b2PolygonShape.
    const float k_inv3 = 1.0f / 3.0f;
    Vec2 origin = vertices[0];

    float area = 0.0f;
    Vec2 center(0.0f, 0.0f);
    float I = 0.0f;

    for (size_t i = 1; i + 1 < vertices.size(); ++i) {
        Vec2 e1 = vertices[i] - origin;
        Vec2 e2 = vertices[i + 1] - origin;

        float d = e1.cross(e2);
        float triArea = 0.5f * d;
        area += triArea;

        center += triArea * k_inv3 * (e1 + e2);

        float intx2 = e1.x * e1.x + e1.x * e2.x + e2.x * e2.x;
        float inty2 = e1.y * e1.y + e1.y * e2.y + e2.y * e2.y;
        I += (0.25f * k_inv3 * d) * (intx2 + inty2);
    }

    if (area > 1e-9f) {
        center *= 1.0f / area;
    }

    md.mass = density * area;
    md.centroid = center + origin;

    // I is currently about `origin`; shift to be about `center` (centroid
    // relative to origin), then shift again to be about the local origin.
    I = density * I - md.mass * center.dot(center);
    md.inertia = I + md.mass * md.centroid.dot(md.centroid);

    return md;
}

Vec2 ShapeData::localHalfExtents() const {
    if (type == ShapeType::Circle) {
        return Vec2(radius, radius);
    }
    float maxX = 0.0f, maxY = 0.0f;
    for (const auto& v : vertices) {
        maxX = std::max(maxX, std::fabs(v.x));
        maxY = std::max(maxY, std::fabs(v.y));
    }
    return Vec2(maxX, maxY);
}

} // namespace p2d
