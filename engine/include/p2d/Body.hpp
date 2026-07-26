#pragma once

#include "p2d/Matter.hpp"
#include "p2d/Shape.hpp"
#include "p2d/Vec2.hpp"

#include <cstdint>
#include <string>

namespace p2d {

enum class BodyType { Static, Kinematic, Dynamic };

class Body {
public:
    int id = -1;
    std::string name;

    BodyType type = BodyType::Dynamic;
    ShapeData shape;

    // Transform. NOTE: shapes are expected to be defined with their centroid
    // at the local origin (true for MakeCircle/MakeBox); `position` then
    // doubles as the body's center of mass, which keeps rotational dynamics
    // simple. Off-center polygons are not correctly supported.
    Vec2 position;
    float rotation = 0.0f; // radians

    Vec2 velocity;
    float angularVelocity = 0.0f;

    Vec2 force;
    float torque = 0.0f;

    float density = 1.0f;
    float restitution = 0.3f; // "bounciness": 0 = no bounce, 1 = perfectly elastic
    float friction = 0.3f;

    // Per-second velocity decay (Dynamic bodies only), applied each step as
    // velocity *= 1 / (1 + damping * dt). 0 = no drag.
    float linearDamping = 0.0f;
    float angularDamping = 0.0f;

    // Multiplies World::gravity for this body only. 1 = normal, 0 = floats,
    // negative = falls upward.
    float gravityScale = 1.0f;

    // If true, this body never rotates -- torque and off-center collisions
    // can't spin it (infinite rotational inertia). Toggling requires a
    // computeMass() call to take effect, hence it's not just a public field
    // with no side effects.
    bool fixedRotation = false;

    // If true, this body still generates contacts (World::onContact fires)
    // but never receives collision impulses -- useful for trigger zones.
    // Matches Box2D's "sensor fixture" semantics: a contact is a sensor if
    // *either* body in the pair is a sensor.
    bool isSensor = false;

    // Simulation-fidelity dial -- see MatterKind (Matter.hpp) for the full
    // explanation. Rigidbody (the default) is an ordinary rigidbody,
    // entirely unaffected by any of this. Setting it to Matter or
    // OptiMatter opts THIS body into World's Matter-family fidelity
    // handling (computeSubstepCount/correctPositions/updateSleepState) --
    // Matter is MORE accurate than Rigidbody (tighter sleep thresholds,
    // smaller correction cap, tighter substep-forcing fraction), while
    // OptiMatter pulls the opposite way: exempt from forced extra
    // substeps, looser sleep thresholds, larger correction cap. Unlike
    // p2d::Matter the standalone particle class, a Body with this set
    // still keeps every other rigidbody capability -- rotation, any
    // shape, scripting, UI-Element hosting -- this is purely a solver
    // fidelity knob, not a different kind of object.
    MatterKind matterKind = MatterKind::Rigidbody;

    // Viewport display color (RGB, 0-255). World::createBody sets a sensible
    // per-BodyType default when a body is first created, but from then on
    // it's just an ordinary editable property -- unlike the old
    // Renderer::colorForBody, which derived a body's color from its
    // BodyType and isAwake every frame, so a body's displayed color changed
    // on its own the moment it fell asleep or had its type changed, with no
    // way to pin a specific color for a scripted/color-coded scene. Not
    // touched by anything at runtime now; only the Inspector's color picker
    // and scene load/save write to it.
    uint8_t colorR = 200, colorG = 200, colorB = 200;

    // Path to an image file to render on this body instead of a flat color
    // fill (empty = no texture, just colorR/G/B as before). Purely a
    // rendering concern -- the engine itself never reads this; Renderer
    // loads/caches the actual image and maps it onto the body's own local
    // shape space (stable under rotation and independent of the camera),
    // stretched to fit rather than tiled (tiling is background-only, see
    // EditorApp's backgroundTexturePath_/backgroundTiled_).
    std::string texturePath;

    // Sleeping (Dynamic bodies only): once a body's velocity has stayed
    // below World's sleep thresholds for World::timeToSleep seconds, it
    // stops being integrated and solved entirely until something wakes it
    // (see wake()) -- a large performance win for scenes with many bodies
    // at rest, and standard behavior in every mainstream engine.
    bool isAwake = true;
    float sleepTime = 0.0f;

    // Scratch flag used internally by World::updateSleepState() to
    // coordinate sleeping between touching bodies within a single step (see
    // its use there) -- not meaningful outside that function, and not
    // exposed to Lua.
    bool sleepReady = false;

    // Marks the body as awake and resets its sleep timer. Called
    // automatically by the apply*() methods below (a script/API call that
    // disturbs a body should always wake it); World also calls this when an
    // awake or kinematic body touches a sleeping one.
    void wake() {
        isAwake = true;
        sleepTime = 0.0f;
    }

    float mass = 0.0f;
    float invMass = 0.0f;
    float inertia = 0.0f;
    float invInertia = 0.0f;

    // Path (relative to the working directory or an app-defined script root)
    // to a Lua script driving this body's behavior. Empty = no script.
    std::string scriptPath;

    // True for a body created as a UI Element (see
    // EditorApp::createUiElementInScene()/createUiElement()) -- just a
    // marker so the editor can tag it distinctly in the Hierarchy and show
    // the uiText/uiKind-dependent fields in the Inspector; doesn't change
    // anything about how the engine simulates this body (it's ordinary
    // otherwise, typically Static + isSensor).
    bool isUiElement = false;

    // Which ui.* widget a UI Element's generated template script draws:
    // "button", "text", "checkbox", or "slider". Purely an editor-side
    // marker (the generated script already hardcodes the matching ui.*
    // call, so it never needs to branch on this itself) -- used only to
    // decide which fields the Inspector shows. Meaningless for a body with
    // no such script attached.
    std::string uiKind = "button";

    // Bound to Lua as `body.ui_text` -- lets a UI Element's generated
    // template script read its label/button text from here instead of a
    // string literal baked into the .lua source, so the Inspector's "UI
    // Text" field can change what's shown/clicked without opening the code
    // editor. Meaningless for a body with no such script attached; just an
    // inert string otherwise.
    std::string uiText;

    // Bound to Lua as `body.ui_value` -- a checkbox's checked state (0.0/1.0)
    // or a slider's current value; unused by "button"/"text" kinds. The
    // template script both reads AND writes this each frame (ui.checkbox/
    // ui.slider_float return the possibly-changed value), so toggling a
    // checkbox or dragging a slider in the Script UI panel is immediately
    // reflected back here -- and, symmetrically, editing it from the
    // Inspector changes what the widget shows next frame.
    float uiValue = 0.0f;

    // Bound to Lua as `body.ui_min`/`body.ui_max` -- a slider's range; unused
    // by every other kind.
    float uiMin = 0.0f;
    float uiMax = 1.0f;

    // Bound to Lua as `body.ui_hide_text` -- when true, a UI Element's
    // generated template script omits its label (still passing ImGui a
    // unique id under the hood so the widget itself keeps working), useful
    // once `texturePath` above gives it an icon that speaks for itself.
    // Meaningless for a "text" kind (its whole purpose is the label) and for
    // a body with no UI Element script attached.
    bool uiHideText = false;

    // Where this body's on_gui() widget is anchored in EditorApp's Viewport
    // overlay -- pixels offset from the viewport's own top-left corner (NOT
    // world space or normalized: a raw screen offset, so it stays put
    // regardless of camera pan/zoom, same as the overlay itself). -1/-1
    // means "never positioned yet"; EditorApp seeds a real, cascaded
    // default the first time it draws this body's overlay window, then
    // updates it every frame to wherever the user has dragged that window
    // to (the whole point -- see EditorApp::drawUiOverlay()). Purely a
    // rendering/editor concern, not read by the engine itself; meaningless
    // for a body with no on_gui-defining script attached.
    float uiOverlayX = -1.0f, uiOverlayY = -1.0f;

    // Recomputes mass/inertia from the current shape + density + type
    // (and fixedRotation). Static and Kinematic bodies always get infinite
    // mass (invMass = 0). Call again after changing density, shape, type,
    // or fixedRotation.
    void computeMass();

    void applyForce(const Vec2& f) {
        force += f;
        wake();
    }
    void applyTorque(float t) {
        torque += t;
        wake();
    }
    void applyForceAtPoint(const Vec2& f, const Vec2& worldPoint);

    void applyLinearImpulse(const Vec2& impulse) {
        velocity += impulse * invMass;
        wake();
    }
    void applyImpulse(const Vec2& impulse, const Vec2& contactVectorFromCenter);

    Vec2 toWorldPoint(const Vec2& local) const;
    Vec2 toWorldVector(const Vec2& localVec) const;

    bool isStatic() const { return type == BodyType::Static; }
};

} // namespace p2d
