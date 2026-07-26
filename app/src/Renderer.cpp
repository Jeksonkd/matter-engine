#include "Renderer.hpp"

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/ConvexShape.hpp>
#include <SFML/Graphics/RenderStates.hpp>
#include <SFML/Graphics/VertexArray.hpp>

#include <cmath>

namespace p2d::app {

namespace {

constexpr float kPi = 3.14159265358979323846f;

// A body's own color (see Body::colorR/G/B) -- fixed once set, independent
// of BodyType or whether it's currently asleep. The Inspector's "Awake /
// Asleep" text indicator already covers that status; this used to also dim
// sleeping bodies and color by BodyType, which meant a body's viewport color
// changed on its own at runtime with no way to pin a specific look.
sf::Color colorForBody(const Body& body) {
    return sf::Color(body.colorR, body.colorG, body.colorB);
}

} // namespace

sf::Texture* Renderer::getOrLoadTexture(const std::string& path) {
    if (path.empty()) return nullptr;
    if (failedTexturePaths_.count(path)) return nullptr;

    auto it = textureCache_.find(path);
    if (it != textureCache_.end()) return &it->second;

    sf::Texture tex;
    if (!tex.loadFromFile(path)) {
        failedTexturePaths_.insert(path);
        return nullptr;
    }
    // Left on unconditionally: harmless for a stretch-fit body texture
    // (its texture coordinates never leave [0, size]) and required for a
    // tiled background -- one cache serves both without needing to track
    // which use(s) a given image is currently assigned to.
    tex.setRepeated(true);
    auto [insertedIt, inserted] = textureCache_.emplace(path, std::move(tex));
    (void)inserted;
    return &insertedIt->second;
}

void Renderer::drawGrid(sf::RenderTarget& target, const Camera& camera) {
    sf::Vector2u size = target.getSize();
    if (size.x == 0 || size.y == 0) return;

    Vec2 topLeft = camera.screenToWorld({0.0f, 0.0f}, size);
    Vec2 bottomRight = camera.screenToWorld({static_cast<float>(size.x), static_cast<float>(size.y)}, size);

    int xStart = static_cast<int>(std::floor(std::min(topLeft.x, bottomRight.x)));
    int xEnd = static_cast<int>(std::ceil(std::max(topLeft.x, bottomRight.x)));
    int yStart = static_cast<int>(std::floor(std::min(topLeft.y, bottomRight.y)));
    int yEnd = static_cast<int>(std::ceil(std::max(topLeft.y, bottomRight.y)));

    sf::VertexArray lines(sf::Lines);
    for (int x = xStart; x <= xEnd; ++x) {
        sf::Color c = (x == 0) ? sf::Color(110, 110, 130) : sf::Color(45, 45, 50);
        sf::Vector2f a = camera.worldToScreen({static_cast<float>(x), static_cast<float>(yStart)}, size);
        sf::Vector2f b = camera.worldToScreen({static_cast<float>(x), static_cast<float>(yEnd)}, size);
        lines.append(sf::Vertex(a, c));
        lines.append(sf::Vertex(b, c));
    }
    for (int y = yStart; y <= yEnd; ++y) {
        sf::Color c = (y == 0) ? sf::Color(110, 110, 130) : sf::Color(45, 45, 50);
        sf::Vector2f a = camera.worldToScreen({static_cast<float>(xStart), static_cast<float>(y)}, size);
        sf::Vector2f b = camera.worldToScreen({static_cast<float>(xEnd), static_cast<float>(y)}, size);
        lines.append(sf::Vertex(a, c));
        lines.append(sf::Vertex(b, c));
    }
    target.draw(lines);
}

void Renderer::drawBackground(sf::RenderTarget& target, const Camera& camera, uint8_t colorR, uint8_t colorG,
                              uint8_t colorB, const std::string& texturePath, bool tiled, float tileWorldSize) {
    target.clear(sf::Color(colorR, colorG, colorB));

    sf::Texture* tex = getOrLoadTexture(texturePath);
    if (!tex) return;

    sf::Vector2u size = target.getSize();
    if (size.x == 0 || size.y == 0) return;
    sf::Vector2u texSize = tex->getSize();
    if (texSize.x == 0 || texSize.y == 0) return;

    // Per-corner texture coordinates computed directly from world position
    // (tiled) or normalized screen fraction (stretched) -- a plain
    // full-screen quad with sf::Texture's Repeated wrap doing the actual
    // tiling in the rasterizer, rather than juggling a sprite's rect/scale/
    // position to approximate the same thing.
    auto texCoordFor = [&](sf::Vector2f screenPos) -> sf::Vector2f {
        if (tiled) {
            Vec2 world = camera.screenToWorld(screenPos, size);
            return {world.x / tileWorldSize * static_cast<float>(texSize.x),
                    -world.y / tileWorldSize * static_cast<float>(texSize.y)};
        }
        return {screenPos.x / static_cast<float>(size.x) * static_cast<float>(texSize.x),
                screenPos.y / static_cast<float>(size.y) * static_cast<float>(texSize.y)};
    };

    sf::Vector2f corners[4] = {
        {0.0f, 0.0f}, {static_cast<float>(size.x), 0.0f}, {static_cast<float>(size.x), static_cast<float>(size.y)},
        {0.0f, static_cast<float>(size.y)}};

    sf::VertexArray quad(sf::Quads, 4);
    for (int i = 0; i < 4; ++i) {
        quad[i].position = corners[i];
        quad[i].texCoords = texCoordFor(corners[i]);
        quad[i].color = sf::Color::White;
    }

    sf::RenderStates states;
    states.texture = tex;
    target.draw(quad, states);
}

void Renderer::drawWorld(sf::RenderTarget& target, const World& world, const Camera& camera, const Body* selected,
                          const std::vector<Body*>& multiSelected, bool showCircleDirectionLine) {
    for (const auto& bodyPtr : world.bodies()) {
        bool isSelected = bodyPtr.get() == selected;
        if (!isSelected) {
            for (Body* b : multiSelected) {
                if (b == bodyPtr.get()) {
                    isSelected = true;
                    break;
                }
            }
        }
        drawBody(target, *bodyPtr, camera, isSelected, showCircleDirectionLine);
    }
    for (const auto& matterPtr : world.matter()) {
        drawMatterParticle(target, *matterPtr, camera);
    }
}

void Renderer::drawMatterParticle(sf::RenderTarget& target, const Matter& m, const Camera& camera) {
    sf::Vector2u size = target.getSize();
    float pixelRadius = m.radius * camera.pixelsPerMeter;
    sf::Vector2f screenPos = camera.worldToScreen(m.position, size);

    sf::Texture* tex = m.texturePath.empty() ? nullptr : getOrLoadTexture(m.texturePath);

    sf::CircleShape shape(pixelRadius, 32);
    shape.setOrigin(pixelRadius, pixelRadius);
    shape.setPosition(screenPos);
    if (tex) {
        shape.setTexture(tex);
        shape.setFillColor(sf::Color::White);
    } else {
        shape.setFillColor(sf::Color(m.colorR, m.colorG, m.colorB));
    }
    shape.setOutlineThickness(1.0f);
    shape.setOutlineColor(sf::Color(20, 20, 20));
    target.draw(shape);
}

void Renderer::drawBody(sf::RenderTarget& target, const Body& body, const Camera& camera, bool selected,
                        bool showCircleDirectionLine) {
    sf::Vector2u size = target.getSize();
    sf::Color fill = colorForBody(body);
    sf::Color outline = selected ? sf::Color::Yellow : sf::Color(20, 20, 20);
    float outlineThickness = selected ? 3.0f : 1.0f;

    sf::Texture* tex = body.texturePath.empty() ? nullptr : getOrLoadTexture(body.texturePath);

    if (body.shape.type == ShapeType::Circle) {
        float pixelRadius = body.shape.radius * camera.pixelsPerMeter;
        sf::Vector2f screenPos = camera.worldToScreen(body.position, size);

        if (tex) {
            // Manually triangulated (rather than sf::CircleShape) so texture
            // coordinates come from the circle's own LOCAL space (a fixed
            // cos/sin unit-circle mapping) instead of sf::Shape's default,
            // which derives them from each point's on-screen position --
            // that would make the image visibly slide across the circle as
            // it rotates or as the camera pans/zooms, instead of looking
            // painted onto it.
            constexpr int kSegments = 32;
            sf::Vector2u texSize = tex->getSize();
            sf::VertexArray fan(sf::TriangleFan, kSegments + 2);
            fan[0].position = screenPos;
            fan[0].texCoords = sf::Vector2f(static_cast<float>(texSize.x) * 0.5f, static_cast<float>(texSize.y) * 0.5f);
            fan[0].color = sf::Color::White;
            for (int i = 0; i <= kSegments; ++i) {
                float angle = (static_cast<float>(i) / static_cast<float>(kSegments)) * 2.0f * kPi;
                Vec2 worldPt = body.toWorldPoint(Vec2(std::cos(angle), std::sin(angle)) * body.shape.radius);
                fan[i + 1].position = camera.worldToScreen(worldPt, size);
                fan[i + 1].texCoords = sf::Vector2f((std::cos(angle) * 0.5f + 0.5f) * static_cast<float>(texSize.x),
                                                     (1.0f - (std::sin(angle) * 0.5f + 0.5f)) *
                                                         static_cast<float>(texSize.y));
                fan[i + 1].color = sf::Color::White;
            }
            sf::RenderStates states;
            states.texture = tex;
            target.draw(fan, states);

            // A separate thin ring for the selection outline -- VertexArray
            // has no built-in outline the way sf::Shape does.
            sf::CircleShape outlineOnly(pixelRadius, 32);
            outlineOnly.setOrigin(pixelRadius, pixelRadius);
            outlineOnly.setPosition(screenPos);
            outlineOnly.setFillColor(sf::Color::Transparent);
            outlineOnly.setOutlineThickness(outlineThickness);
            outlineOnly.setOutlineColor(outline);
            target.draw(outlineOnly);
        } else {
            sf::CircleShape shape(pixelRadius, 32);
            shape.setOrigin(pixelRadius, pixelRadius);
            shape.setPosition(screenPos);
            shape.setFillColor(fill);
            shape.setOutlineThickness(outlineThickness);
            shape.setOutlineColor(outline);
            target.draw(shape);
        }

        if (showCircleDirectionLine) {
            Vec2 spokeWorld = body.toWorldPoint(Vec2(body.shape.radius, 0.0f));
            sf::Vector2f spokeScreen = camera.worldToScreen(spokeWorld, size);
            sf::Vertex line[2] = {sf::Vertex(screenPos, sf::Color::White), sf::Vertex(spokeScreen, sf::Color::White)};
            target.draw(line, 2, sf::Lines);
        }
    } else if (tex) {
        // Same reasoning as the circle case: texture coordinates mapped
        // from each vertex's own LOCAL position (fixed relative to the
        // body's local half-extents) rather than sf::ConvexShape's
        // screen-position-derived default, so the image stays fixed to the
        // shape instead of sliding as it rotates or the camera moves.
        Vec2 half = body.shape.localHalfExtents();
        sf::Vector2u texSize = tex->getSize();
        size_t n = body.shape.vertices.size();
        sf::VertexArray fan(sf::TriangleFan, n);
        for (size_t i = 0; i < n; ++i) {
            Vec2 local = body.shape.vertices[i];
            Vec2 world = body.toWorldPoint(local);
            fan[i].position = camera.worldToScreen(world, size);
            float u = half.x > 1e-6f ? (local.x + half.x) / (2.0f * half.x) : 0.5f;
            float v = half.y > 1e-6f ? (local.y + half.y) / (2.0f * half.y) : 0.5f;
            fan[i].texCoords = sf::Vector2f(u * static_cast<float>(texSize.x), (1.0f - v) * static_cast<float>(texSize.y));
            fan[i].color = sf::Color::White;
        }
        sf::RenderStates states;
        states.texture = tex;
        target.draw(fan, states);

        sf::ConvexShape outlineOnly;
        outlineOnly.setPointCount(n);
        for (size_t i = 0; i < n; ++i) outlineOnly.setPoint(i, fan[i].position);
        outlineOnly.setFillColor(sf::Color::Transparent);
        outlineOnly.setOutlineThickness(outlineThickness);
        outlineOnly.setOutlineColor(outline);
        target.draw(outlineOnly);
    } else {
        sf::ConvexShape shape;
        shape.setPointCount(body.shape.vertices.size());
        for (size_t i = 0; i < body.shape.vertices.size(); ++i) {
            Vec2 world = body.toWorldPoint(body.shape.vertices[i]);
            shape.setPoint(i, camera.worldToScreen(world, size));
        }
        shape.setFillColor(fill);
        shape.setOutlineThickness(outlineThickness);
        shape.setOutlineColor(outline);
        target.draw(shape);
    }
}

} // namespace p2d::app
