#include "p2d/SoftBody.hpp"
#include "p2d/SpringJoint.hpp"

#include <cmath>

namespace p2d {

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

void SoftBody::applySpringForces() {
    for (const Spring& s : springs) {
        if (s.a < 0 || s.b < 0 || static_cast<size_t>(s.a) >= particles.size() ||
            static_cast<size_t>(s.b) >= particles.size()) {
            continue; // pruned by a mid-simulation particle deletion -- skip rather than crash
        }
        applyHookeSpring(*particles[static_cast<size_t>(s.a)], *particles[static_cast<size_t>(s.b)], s.restLength,
                         s.stiffness, s.damping);
    }
}

SoftBody makeSoftBodyRing(World& world, Vec2 center, float radius, int segments, float particleRadius,
                          float stiffness, float damping) {
    SoftBody sb;
    if (segments < 3) segments = 3;

    sb.particles.reserve(static_cast<size_t>(segments));
    for (int i = 0; i < segments; ++i) {
        float angle = (static_cast<float>(i) / static_cast<float>(segments)) * 2.0f * kPi;
        Vec2 pos = center + Vec2(std::cos(angle), std::sin(angle)) * radius;
        Body* p = world.createCircle(pos, particleRadius, BodyType::Dynamic);
        p->restitution = 0.1f;
        p->friction = 0.5f;
        sb.particles.push_back(p);
    }

    auto restLength = [&](int i, int j) { return (sb.particles[static_cast<size_t>(j)]->position -
                                                    sb.particles[static_cast<size_t>(i)]->position)
                                                       .length(); };

    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        sb.springs.push_back({i, next, restLength(i, next), stiffness, damping});

        // Skip-one cross brace: without this, a ring of edge springs alone
        // can fold flat (edge springs only resist changes in perimeter
        // length, not the ring's overall shape).
        int acrossOne = (i + 2) % segments;
        if (acrossOne != i) {
            sb.springs.push_back({i, acrossOne, restLength(i, acrossOne), stiffness * 0.5f, damping});
        }
    }

    return sb;
}

SoftBody makeSoftBodyJelly(World& world, Vec2 center, float radius, int segments, float particleRadius,
                           float stiffness, float damping) {
    SoftBody sb = makeSoftBodyRing(world, center, radius, segments, particleRadius, stiffness, damping);

    // Insert the hub at the FRONT so every existing spring index (which
    // refers to the ring particles built above) stays valid -- everything
    // just shifts up by one, which we do explicitly below rather than
    // relying on push_front (particles is a std::vector).
    Body* hub = world.createCircle(center, particleRadius * 1.5f, BodyType::Dynamic);
    hub->restitution = 0.1f;
    hub->friction = 0.5f;

    sb.particles.insert(sb.particles.begin(), hub);
    for (SoftBody::Spring& s : sb.springs) {
        ++s.a;
        ++s.b;
    }

    int hubIndex = 0;
    for (int i = 1; i <= segments; ++i) {
        float spokeLength = (sb.particles[static_cast<size_t>(i)]->position - center).length();
        sb.springs.push_back({hubIndex, i, spokeLength, stiffness, damping});
    }

    return sb;
}

SoftBody makeSoftBodyCloth(World& world, Vec2 topLeft, int cols, int rows, float spacing, float particleRadius,
                           float stiffness, float damping, bool pinTop) {
    SoftBody sb;
    if (cols < 2) cols = 2;
    if (rows < 2) rows = 2;

    auto index = [cols](int col, int row) { return row * cols + col; };

    sb.particles.reserve(static_cast<size_t>(cols) * static_cast<size_t>(rows));
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            Vec2 pos = topLeft + Vec2(static_cast<float>(col) * spacing, -static_cast<float>(row) * spacing);
            bool pinned = pinTop && row == 0;
            Body* p = world.createCircle(pos, particleRadius, pinned ? BodyType::Static : BodyType::Dynamic);
            p->restitution = 0.1f;
            p->friction = 0.5f;
            sb.particles.push_back(p);
        }
    }

    auto restLength = [&](int i, int j) { return (sb.particles[static_cast<size_t>(j)]->position -
                                                    sb.particles[static_cast<size_t>(i)]->position)
                                                       .length(); };

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            int here = index(col, row);
            // Structural: right and down neighbors.
            if (col + 1 < cols) {
                int right = index(col + 1, row);
                sb.springs.push_back({here, right, restLength(here, right), stiffness, damping});
            }
            if (row + 1 < rows) {
                int down = index(col, row + 1);
                sb.springs.push_back({here, down, restLength(here, down), stiffness, damping});
            }
            // Shear: both diagonal neighbors -- resists the grid collapsing
            // into a degenerate parallelogram, which structural springs
            // alone don't constrain.
            if (col + 1 < cols && row + 1 < rows) {
                int diag = index(col + 1, row + 1);
                sb.springs.push_back({here, diag, restLength(here, diag), stiffness * 0.5f, damping});
            }
            if (col > 0 && row + 1 < rows) {
                int diag = index(col - 1, row + 1);
                sb.springs.push_back({here, diag, restLength(here, diag), stiffness * 0.5f, damping});
            }
        }
    }

    return sb;
}

} // namespace p2d
