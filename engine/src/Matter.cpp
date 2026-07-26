#include "p2d/Matter.hpp"

#include <cmath>

namespace p2d {

void Matter::computeMass() {
    constexpr float kPi = 3.14159265358979323846f;
    float area = kPi * radius * radius;
    mass = density * area;
    invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
}

} // namespace p2d
