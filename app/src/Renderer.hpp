#pragma once

#include "Camera.hpp"
#include "p2d/World.hpp"

#include <SFML/Graphics/RenderTarget.hpp>
#include <SFML/Graphics/Texture.hpp>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace p2d::app {

class Renderer {
public:
    void drawGrid(sf::RenderTarget& target, const Camera& camera);

    // Fills the whole target with a flat color, then optionally draws an
    // image over it -- tiled (repeating every `tileWorldSize` world units,
    // anchored to world space so it pans/zooms with the scene rather than
    // sliding independently of it) or stretched to exactly cover the
    // current view. `texturePath` empty just leaves the flat color. Call
    // before drawGrid()/drawWorld() -- this is the backdrop everything else
    // draws on top of.
    void drawBackground(sf::RenderTarget& target, const Camera& camera, uint8_t colorR, uint8_t colorG,
                         uint8_t colorB, const std::string& texturePath, bool tiled, float tileWorldSize = 2.0f);

    // `showCircleDirectionLine` draws the small radial line marking a
    // circle's current rotation (otherwise invisible on a plain circle) --
    // on by default, toggled off via Settings > "Show circle direction
    // line" for scenes that don't want it cluttering the view.
    void drawWorld(sf::RenderTarget& target, const World& world, const Camera& camera, const Body* selected,
                    const std::vector<Body*>& multiSelected = {}, bool showCircleDirectionLine = true);

    // Loads (and caches) the image at `path`. Returns nullptr if it doesn't
    // exist or isn't a format SFML can decode -- callers fall back to a
    // flat color fill (or a plain, textureless widget) in that case. Cached
    // textures are always left "repeated" (harmless for a stretch-fit use,
    // since those texture coordinates never leave [0, size] anyway; required
    // for a tiled one) -- one cache serves every use without needing to know
    // in advance which a given image will be used for. Public so EditorApp's
    // `ui.image`/`ui.image_button` Lua bindings (see bindUiApi()) can share
    // this same cache instead of loading their own separate copy of the same
    // file.
    sf::Texture* getOrLoadTexture(const std::string& path);

private:
    void drawBody(sf::RenderTarget& target, const Body& body, const Camera& camera, bool selected,
                  bool showCircleDirectionLine);
    // Matter is always a circle and never rotates (see Matter.hpp), so this
    // is considerably simpler than drawBody()'s circle case: a plain
    // sf::CircleShape with an optional texture is enough (no manual
    // VertexArray triangulation needed -- that exists in drawBody() purely
    // to keep a texture stable under a Body's OWN rotation, which Matter
    // has none of). No selection highlight support yet -- Matter isn't
    // selectable in the Inspector in this first version.
    void drawMatterParticle(sf::RenderTarget& target, const Matter& m, const Camera& camera);

    std::unordered_map<std::string, sf::Texture> textureCache_;
    // Paths that failed to load once already -- retried on every single
    // drawBody() call otherwise (every frame, for every body referencing a
    // since-moved/deleted file), which is wasted disk I/O for something
    // that isn't going to start working again on its own.
    std::unordered_set<std::string> failedTexturePaths_;
};

} // namespace p2d::app
