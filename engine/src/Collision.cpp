#include "p2d/Collision.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <vector>

namespace p2d::collision {

namespace {

struct WorldPoly {
    std::vector<Vec2> verts;
    std::vector<Vec2> normals;
};

WorldPoly toWorld(const Body& b) {
    WorldPoly w;
    w.verts.reserve(b.shape.vertices.size());
    w.normals.reserve(b.shape.normals.size());
    for (const auto& v : b.shape.vertices) w.verts.push_back(b.toWorldPoint(v));
    for (const auto& n : b.shape.normals) w.normals.push_back(b.toWorldVector(n));
    return w;
}

// Largest separation of `ref`'s faces against `inc`'s vertices. A positive
// result means a separating axis was found (no collision).
float findMaxSeparation(size_t& faceIndex, const WorldPoly& ref, const WorldPoly& inc) {
    float bestSep = -FLT_MAX;
    size_t bestIdx = 0;
    for (size_t i = 0; i < ref.verts.size(); ++i) {
        Vec2 n = ref.normals[i];
        Vec2 v1 = ref.verts[i];
        float minProj = FLT_MAX;
        for (const auto& incVert : inc.verts) {
            minProj = std::min(minProj, n.dot(incVert - v1));
        }
        if (minProj > bestSep) {
            bestSep = minProj;
            bestIdx = i;
        }
    }
    faceIndex = bestIdx;
    return bestSep;
}

int clipSegmentToLine(Vec2 outPts[2], const Vec2 inPts[2], const Vec2& normal, float offset) {
    int numOut = 0;
    float d0 = normal.dot(inPts[0]) - offset;
    float d1 = normal.dot(inPts[1]) - offset;
    if (d0 <= 0.0f) outPts[numOut++] = inPts[0];
    if (d1 <= 0.0f) outPts[numOut++] = inPts[1];
    if (d0 * d1 < 0.0f) {
        float t = d0 / (d0 - d1);
        outPts[numOut++] = inPts[0] + t * (inPts[1] - inPts[0]);
    }
    return numOut;
}

// Pure geometry, no Body/Matter dependency -- shared by circleVsCircle()
// (Body-vs-Body) and generateMatterContact()/generateMatterBodyContact()
// (anything involving a Matter, which is always just a position+radius).
bool circleVsCircleRaw(Vec2 posA, float radiusA, Vec2 posB, float radiusB, Contact& out) {
    Vec2 delta = posB - posA;
    float dist = delta.length();
    float radiusSum = radiusA + radiusB;
    if (dist >= radiusSum) return false;

    Vec2 normal = (dist > 1e-9f) ? delta * (1.0f / dist) : Vec2(0.0f, 1.0f);
    out.normal = normal;
    out.penetration[0] = radiusSum - dist;
    out.points[0] = posA + normal * radiusA;
    out.count = 1;
    return true;
}

bool circleVsCircle(const Body& a, const Body& b, Contact& out) {
    return circleVsCircleRaw(a.position, a.shape.radius, b.position, b.shape.radius, out);
}

// Pure geometry (circle position/radius, not tied to Body/Matter) against a
// polygon Body -- shared by circleVsPolygon() (Body-vs-Body) and
// generateMatterBodyContact() (a Matter circle against a polygon Body).
// Normal in the result points from the polygon body towards the circle.
bool circleVsPolygonRaw(Vec2 circlePos, float radius, const Body& polyBody, Contact& out) {
    const ShapeData& poly = polyBody.shape;

    Vec2 localCenter = rotate(circlePos - polyBody.position, -polyBody.rotation);

    float separation = -FLT_MAX;
    size_t faceIndex = 0;
    for (size_t i = 0; i < poly.vertices.size(); ++i) {
        float s = poly.normals[i].dot(localCenter - poly.vertices[i]);
        if (s > radius) return false;
        if (s > separation) {
            separation = s;
            faceIndex = i;
        }
    }

    size_t next = (faceIndex + 1) % poly.vertices.size();
    Vec2 v1 = poly.vertices[faceIndex];
    Vec2 v2 = poly.vertices[next];

    Vec2 localNormal;
    Vec2 localContact;
    float dist;

    if (separation < 1e-9f) {
        // Center is inside the polygon: push out along the best face.
        localNormal = poly.normals[faceIndex];
        dist = separation;
        localContact = localCenter - localNormal * dist;
    } else {
        float u1 = (localCenter - v1).dot(v2 - v1);
        float u2 = (localCenter - v2).dot(v1 - v2);

        if (u1 <= 0.0f) {
            dist = (localCenter - v1).length();
            if (dist > radius) return false;
            localNormal = (localCenter - v1).normalized();
            localContact = v1;
        } else if (u2 <= 0.0f) {
            dist = (localCenter - v2).length();
            if (dist > radius) return false;
            localNormal = (localCenter - v2).normalized();
            localContact = v2;
        } else {
            dist = separation;
            localNormal = poly.normals[faceIndex];
            localContact = localCenter - localNormal * dist;
        }
    }

    out.normal = rotate(localNormal, polyBody.rotation);
    out.points[0] = polyBody.toWorldPoint(localContact);
    out.penetration[0] = radius - dist;
    out.count = 1;
    return true;
}

bool polygonVsPolygon(const Body& a, const Body& b, Contact& out, const Vec2* prevNormalHint) {
    WorldPoly wa = toWorld(a);
    WorldPoly wb = toWorld(b);

    size_t faceA;
    float sepA = findMaxSeparation(faceA, wa, wb);
    if (sepA >= 0.0f) return false;

    size_t faceB;
    float sepB = findMaxSeparation(faceB, wb, wa);
    if (sepB >= 0.0f) return false;

    const WorldPoly* refPoly;
    const WorldPoly* incPoly;
    size_t refFace;
    bool flip;

    // Reference-face tie-break. For a flush contact (e.g. one box resting
    // squarely on another) sepA and sepB both sit right at 0 and can drift
    // either side of each other from step to step on nothing more than
    // solver/positional-correction noise -- deciding purely on "sepB >
    // sepA + tol" then flips `flip` back and forth, which flips the contact
    // normal (and, via cc.tangent in World.cpp, the fixed friction tangent)
    // by a small angle every time it happens. Whichever candidate is closer
    // to last step's actual contact normal (if we have one) is preferred
    // over the raw comparison, so a tie stays resolved the same way it was
    // last step instead of chasing sub-epsilon noise; with no prior contact
    // to compare against (a brand new contact) it falls back to the same
    // deterministic "A wins ties" rule as before.
    const float k_relTol = 1e-4f;
    if (sepB > sepA + k_relTol) {
        flip = true;
    } else if (sepA > sepB + k_relTol) {
        flip = false;
    } else if (prevNormalHint) {
        float dotA = wa.normals[faceA].dot(*prevNormalHint);
        float dotB = (-1.0f * wb.normals[faceB]).dot(*prevNormalHint);
        flip = dotB > dotA;
    } else {
        flip = false;
    }

    if (flip) {
        refPoly = &wb;
        incPoly = &wa;
        refFace = faceB;
    } else {
        refPoly = &wa;
        incPoly = &wb;
        refFace = faceA;
    }

    size_t refFaceNext = (refFace + 1) % refPoly->verts.size();
    Vec2 v1 = refPoly->verts[refFace];
    Vec2 v2 = refPoly->verts[refFaceNext];
    Vec2 refNormal = refPoly->normals[refFace];

    size_t incFace = 0;
    float minDot = FLT_MAX;
    for (size_t i = 0; i < incPoly->normals.size(); ++i) {
        float d = refNormal.dot(incPoly->normals[i]);
        if (d < minDot) {
            minDot = d;
            incFace = i;
        }
    }
    size_t incFaceNext = (incFace + 1) % incPoly->verts.size();
    Vec2 incEdge[2] = {incPoly->verts[incFace], incPoly->verts[incFaceNext]};

    Vec2 tangent = (v2 - v1).normalized();
    float negSide = -tangent.dot(v1);
    float posSide = tangent.dot(v2);

    Vec2 clipped1[2];
    if (clipSegmentToLine(clipped1, incEdge, -1.0f * tangent, negSide) < 2) return false;

    Vec2 clipped2[2];
    if (clipSegmentToLine(clipped2, clipped1, tangent, posSide) < 2) return false;

    out.normal = flip ? -1.0f * refNormal : refNormal;
    out.count = 0;
    for (const auto& p : clipped2) {
        float sep = refNormal.dot(p - v1);
        if (sep <= 0.0f) {
            out.points[out.count] = p;
            out.penetration[out.count] = -sep;
            ++out.count;
        }
    }
    return out.count > 0;
}

} // namespace

AABB computeAABB(const Body& body) {
    AABB box;
    if (body.shape.type == ShapeType::Circle) {
        Vec2 r(body.shape.radius, body.shape.radius);
        box.min = body.position - r;
        box.max = body.position + r;
        return box;
    }

    Vec2 first = body.toWorldPoint(body.shape.vertices[0]);
    box.min = first;
    box.max = first;
    for (size_t i = 1; i < body.shape.vertices.size(); ++i) {
        Vec2 wp = body.toWorldPoint(body.shape.vertices[i]);
        box.min.x = std::min(box.min.x, wp.x);
        box.min.y = std::min(box.min.y, wp.y);
        box.max.x = std::max(box.max.x, wp.x);
        box.max.y = std::max(box.max.y, wp.y);
    }
    return box;
}

bool aabbOverlap(const AABB& a, const AABB& b) {
    if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
    if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
    return true;
}

bool generateContact(Body& a, Body& b, Contact& out, const Vec2* prevNormalHint) {
    out = Contact{};
    out.a = &a;
    out.b = &b;

    bool hit = false;
    if (a.shape.type == ShapeType::Circle && b.shape.type == ShapeType::Circle) {
        hit = circleVsCircle(a, b, out);
    } else if (a.shape.type == ShapeType::Circle && b.shape.type == ShapeType::Polygon) {
        hit = circleVsPolygonRaw(a.position, a.shape.radius, b, out);
        if (hit) out.normal = -1.0f * out.normal; // raw normal points poly->circle == b->a
    } else if (a.shape.type == ShapeType::Polygon && b.shape.type == ShapeType::Circle) {
        hit = circleVsPolygonRaw(b.position, b.shape.radius, a, out); // raw normal points a(poly)->b(circle), already correct
    } else {
        hit = polygonVsPolygon(a, b, out, prevNormalHint);
    }

    if (hit) {
        out.restitution = std::max(a.restitution, b.restitution);
        out.friction = std::sqrt(a.friction * b.friction);
        out.isSensor = a.isSensor || b.isSensor;
    }
    return hit;
}

AABB computeMatterAABB(const Matter& m) {
    Vec2 r(m.radius, m.radius);
    return {m.position - r, m.position + r};
}

bool generateMatterContact(Matter& a, Matter& b, MatterContact& out) {
    Contact geo; // reused only for its geometry fields (normal/points/penetration) -- a/restitution/etc. below are Matter's own
    if (!circleVsCircleRaw(a.position, a.radius, b.position, b.radius, geo)) return false;

    out.a = &a;
    out.b = &b;
    out.normal = geo.normal;
    out.point = geo.points[0];
    out.penetration = geo.penetration[0];
    out.restitution = std::max(a.restitution, b.restitution);
    out.friction = std::sqrt(a.friction * b.friction);
    return true;
}

bool raycastCircle(Vec2 origin, Vec2 dir, Vec2 center, float radius, RaycastHit& out) {
    // Solve |origin + t*dir - center|^2 = radius^2 for the smallest t in
    // [0, 1] -- standard quadratic-formula circle raycast (same approach as
    // Box2D's b2CircleShape::RayCast).
    Vec2 s = origin - center;
    float b = s.dot(dir);
    float rr = dir.dot(dir);
    if (rr < 1e-12f) return false;
    float c = s.dot(s) - radius * radius;

    float sigma = b * b - rr * c;
    if (sigma < 0.0f) return false;

    float t = (-b - std::sqrt(sigma)) / rr;
    if (t < 0.0f || t > 1.0f) return false; // origin already inside, or hit beyond the ray's far end

    out.fraction = t;
    out.point = origin + dir * t;
    out.normal = (out.point - center).normalized();
    return true;
}

bool raycastPolygon(Vec2 origin, Vec2 dir, const Body& polyBody, RaycastHit& out, float inflateRadius) {
    // Transform the ray into the polygon's local space rather than
    // transforming every vertex/normal into world space -- same slab-
    // clipping algorithm Box2D's b2PolygonShape::RayCast uses: each edge's
    // half-plane either can't affect [lower, upper] (ray parallel to it,
    // origin already on the inside), or clips one end of the range:
    // entering half-planes (denominator < 0) can only raise `lower`,
    // exiting ones (denominator > 0) can only lower `upper`. Once
    // upper < lower, no fraction satisfies every edge simultaneously.
    Vec2 localOrigin = rotate(origin - polyBody.position, -polyBody.rotation);
    Vec2 localDir = rotate(dir, -polyBody.rotation);

    const ShapeData& poly = polyBody.shape;
    float lower = 0.0f;
    float upper = 1.0f;
    int hitEdge = -1;

    for (size_t i = 0; i < poly.vertices.size(); ++i) {
        // + inflateRadius shifts each face's effective plane outward by
        // that amount (see this function's doc comment in Collision.hpp for
        // the derivation) -- 0 for an ordinary point raycast, leaves the
        // rest of the algorithm untouched either way.
        float numerator = poly.normals[i].dot(poly.vertices[i] - localOrigin) + inflateRadius;
        float denominator = poly.normals[i].dot(localDir);

        if (denominator == 0.0f) {
            if (numerator < 0.0f) return false; // origin outside this edge, ray never enters
        } else {
            if (denominator < 0.0f && numerator < lower * denominator) {
                lower = numerator / denominator;
                hitEdge = static_cast<int>(i);
            } else if (denominator > 0.0f && numerator < upper * denominator) {
                upper = numerator / denominator;
            }
        }
        if (upper < lower) return false;
    }

    if (hitEdge < 0) return false; // origin starts inside the polygon -- no entry surface to report

    out.fraction = lower;
    Vec2 localPoint = localOrigin + localDir * lower;
    out.point = polyBody.toWorldPoint(localPoint);
    out.normal = rotate(poly.normals[static_cast<size_t>(hitEdge)], polyBody.rotation);
    return true;
}

bool pointInBody(const Body& body, Vec2 point) {
    if (body.shape.type == ShapeType::Circle) {
        return (point - body.position).lengthSq() <= body.shape.radius * body.shape.radius;
    }
    Vec2 local = rotate(point - body.position, -body.rotation);
    for (size_t i = 0; i < body.shape.vertices.size(); ++i) {
        if (body.shape.normals[i].dot(local - body.shape.vertices[i]) > 0.0f) return false;
    }
    return true;
}

bool generateMatterBodyContact(Matter& matter, Body& body, MatterBodyContact& out) {
    Contact geo;
    bool hit;
    if (body.shape.type == ShapeType::Circle) {
        hit = circleVsCircleRaw(matter.position, matter.radius, body.position, body.shape.radius, geo);
    } else {
        hit = circleVsPolygonRaw(matter.position, matter.radius, body, geo);
        if (hit) geo.normal = -1.0f * geo.normal; // raw normal points poly(body)->circle(matter); we want matter->body
    }
    if (!hit) return false;

    out.matter = &matter;
    out.body = &body;
    out.normal = geo.normal;
    out.point = geo.points[0];
    out.penetration = geo.penetration[0];
    out.restitution = std::max(matter.restitution, body.restitution);
    out.friction = std::sqrt(matter.friction * body.friction);
    out.isSensor = body.isSensor;
    return true;
}

} // namespace p2d::collision
