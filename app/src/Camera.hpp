#pragma once

#include "p2d/Vec2.hpp"

#include <SFML/System/Vector2.hpp>

#include <algorithm>

namespace p2d::app {

// Maps physics world space (meters, +y up) to SFML screen space (pixels,
// +y down) and back. `center` is the world point rendered at the middle
// of the window.
struct Camera {
    Vec2 center{0.0f, 2.0f};
    float pixelsPerMeter = 50.0f;

    sf::Vector2f worldToScreen(const Vec2& w, sf::Vector2u windowSize) const {
        float sx = (w.x - center.x) * pixelsPerMeter + static_cast<float>(windowSize.x) * 0.5f;
        float sy = static_cast<float>(windowSize.y) * 0.5f - (w.y - center.y) * pixelsPerMeter;
        return {sx, sy};
    }

    Vec2 screenToWorld(sf::Vector2f s, sf::Vector2u windowSize) const {
        float wx = (s.x - static_cast<float>(windowSize.x) * 0.5f) / pixelsPerMeter + center.x;
        float wy = -(s.y - static_cast<float>(windowSize.y) * 0.5f) / pixelsPerMeter + center.y;
        return {wx, wy};
    }

    void zoomAt(sf::Vector2f screenPoint, sf::Vector2u windowSize, float factor) {
        Vec2 before = screenToWorld(screenPoint, windowSize);
        pixelsPerMeter *= factor;
        pixelsPerMeter = std::max(5.0f, std::min(pixelsPerMeter, 2000.0f));
        Vec2 after = screenToWorld(screenPoint, windowSize);
        center = center + (before - after);
    }
};

} // namespace p2d::app
