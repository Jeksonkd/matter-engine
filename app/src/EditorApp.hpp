#pragma once

#include "Camera.hpp"
#include "Renderer.hpp"
#include "p2d/SoftBody.hpp"
#include "p2d/SpringJoint.hpp"
#include "p2d/World.hpp"
#include "p2d/script/ScriptEngine.hpp"

#include <SFML/Graphics/RenderTexture.hpp>
#include <SFML/Window/Event.hpp>
#include <TextEditor.h>

#include <deque>
#include <filesystem>
#include <string>
#include <vector>

namespace p2d::app {

enum class SpawnTool { None, Spawn, Resize, Rotate, BoxSelect };

// What the (single, unified) Spawn tool places when you click the viewport --
// all ordinary rigid Bodies (SAT polygon collision already handles any
// convex shape, not just boxes). Soft bodies (p2d::SoftBody -- Ring/Jelly/
// Cloth) and UI Elements are NOT spawn-tool shapes: neither is "a rigid body
// placed at a clicked point" the way these are. Soft bodies are still
// creatable from Lua (soft_body.create_ring/create_jelly/create_cloth, see
// bindUiApi()) -- just not from this toolbar. UI Elements are created via
// createObjectInScene() (Hierarchy panel's "Create" button) or
// createUiElement() (FileSystem panel's "New Script" popup) instead, neither
// of which needs a viewport click: a UI Element isn't a physical object
// placed in the world, so it doesn't have a meaningful "position you
// clicked" the way a spawned shape does. createObjectInScene() ALSO handles
// ordinary physics shapes now (spawned at world (0, 0), optionally in a
// batch -- see createQuantityIdx_/createMatterKind_/createBodyType_), not
// just UI kinds. For tying
// *existing* objects together instead (any shape, any BodyType, without
// replacing them) see connectWithSpring() and the Inspector's Connections
// section, which uses the same underlying spring math (p2d::SpringJoint)
// generalized to arbitrary body pairs.
enum class SpawnShapeKind { Circle, Box, Triangle, Pentagon, Hexagon };

class EditorApp {
public:
    // `projectDir` is the specific project folder to open (see
    // ProjectManagerScreen) -- its scripts/ subfolder is scaffolded if
    // missing, then used for the default scene and the FileSystem panel.
    explicit EditorApp(std::filesystem::path projectDir);

    // Forces one final save (see saveSceneToDisk()) rather than relying on
    // the periodic autosave timer alone -- without this, editing something
    // (e.g. attaching a script) and then immediately clicking "Projects" or
    // closing the app within the autosave interval would silently lose that
    // change, since main.cpp destroys this object (via unique_ptr::reset()
    // or just program exit) right away in both cases.
    ~EditorApp();

    // Raw SFML events -- still needed for wheel/press/release edges even
    // though the viewport now lives inside a docked ImGui panel.
    void handleEvent(const sf::Event& event);

    void update(float realDt);

    // Renders the physics world into the offscreen viewport texture. Call
    // before drawUI() so the texture is fresh when the Viewport panel
    // displays it this frame.
    void renderViewport();

    // Builds the full docked editor layout (toolbar, viewport, hierarchy,
    // inspector, console) via ImGui.
    void drawUI();

    // True for exactly one frame after the user clicks "Projects" in the
    // toolbar -- the caller (main.cpp) should destroy this EditorApp and
    // return to the ProjectManagerScreen.
    bool wantsReturnToProjectManager() const { return returnToProjectManager_; }

private:
    World world_;
    script::ScriptEngine scriptEngine_;
    Camera camera_;
    Renderer renderer_;

    // --- Play/Pause/Stop -----------------------------------------------
    // `running_` mirrors Godot's Play state. `playSnapshot_` is continuously
    // refreshed (see update()) while the project is in genuine edit mode --
    // i.e. NOT currently playing AND not merely paused mid-session (see
    // `everPlayedSinceCheckpoint_` below). Pressing Play, Pausing, or Step
    // all freeze it at "the scene as it looked right before Play was first
    // pressed"; Stop and Reset Scene both restore that and re-arm continuous
    // capture. This means anything spawned/changed during a play session
    // (including via a scripted UI button while playing or paused) is
    // discarded by Reset Scene, not just by Stop. Reset Scene only falls
    // back to rebuilding the hardcoded starter scene if no snapshot exists
    // yet at all. If `keepSpawnedObjectsOnReset_` is enabled (Settings),
    // capture instead stays continuous through pause too, so Reset Scene
    // preserves whatever was spawned/changed during play.
    bool running_ = false;
    bool everPlayedSinceCheckpoint_ = false;
    float fixedDt_ = 1.0f / 120.0f;
    float timeScale_ = 1.0f;
    float accumulator_ = 0.0f;

    // --- Settings menu ---------------------------------------------------
    bool showSettings_ = false;
    bool keepSpawnedObjectsOnReset_ = false;
    // Circles otherwise look identical at every rotation angle -- this small
    // radial line is what makes a circle's current rotation visible at all.
    // Some scenes don't want the clutter (many overlapping small circles),
    // hence the toggle rather than always drawing it.
    bool showCircleDirectionLines_ = true;
    // Applied to every body the Spawn tool creates (spawnAt()) -- see
    // Settings' "Default Matter Kind" dropdown. OptiMatter by default (best
    // out-of-the-box performance for a new scene); Body::matterKind's own
    // field default stays Rigidbody for bodies created any other way
    // (scripts, joints), which don't go through this setting at all.
    MatterKind defaultSpawnMatterKind_ = MatterKind::OptiMatter;

    // --- Background: a flat color always, an optional image drawn over it
    // (tiled, anchored to world space, or stretched to the current view) --
    // see Renderer::drawBackground(). Persisted with the rest of the scene
    // (see saveSceneToDisk()/loadSceneFromDisk()).
    uint8_t backgroundColorR_ = 25, backgroundColorG_ = 25, backgroundColorB_ = 30;
    std::string backgroundTexturePath_;
    bool backgroundTiled_ = true;

    struct BodySnapshot {
        ShapeData shape;
        Vec2 position;
        float rotation = 0.0f;
        Vec2 velocity;
        float angularVelocity = 0.0f;
        float density = 1.0f, restitution = 0.3f, friction = 0.3f;
        float linearDamping = 0.0f, angularDamping = 0.0f, gravityScale = 1.0f;
        bool fixedRotation = false, isSensor = false;
        BodyType type = BodyType::Dynamic;
        uint8_t colorR = 200, colorG = 200, colorB = 200;
        std::string texturePath;
        MatterKind matterKind = MatterKind::Rigidbody;
        bool isUiElement = false;
        std::string uiKind = "button";
        std::string uiText;
        float uiValue = 0.0f, uiMin = 0.0f, uiMax = 1.0f;
        bool uiHideText = false;
        float uiOverlayX = -1.0f, uiOverlayY = -1.0f;
        std::string name;
        std::string scriptPath;
    };
    std::vector<BodySnapshot> playSnapshot_;
    bool hasSnapshot_ = false;

    // --- Scene persistence -------------------------------------------------
    // Autosaves the current edit-mode checkpoint (the same data
    // captureSnapshot() already builds every edit-mode frame -- see
    // playSnapshot_/softBodySnapshot_/springJointSnapshot_) to
    // `projectDir_/scene.json` every few seconds, and loads it back on the
    // next time this project is opened -- so bodies, their properties, and
    // whatever scripts were attached to them are exactly where you left
    // them, instead of every reopen starting from the hardcoded blank
    // sandbox. See saveSceneToDisk()/loadSceneFromDisk().
    std::filesystem::path sceneFilePath_;
    float sceneAutosaveTimer_ = 0.0f;

    // --- Soft bodies -------------------------------------------------------
    // Each SoftBody's `particles` are ordinary Body* living in world_ (so
    // world_.clear()/restoreSnapshot() invalidate them) -- see
    // captureSnapshot()/restoreSnapshot() for how spring topology survives
    // a Play/Stop/Reset round-trip by re-resolving particles by name rather
    // than storing raw pointers in the snapshot.
    std::vector<SoftBody> softBodies_;
    int nextSoftBodyId_ = 0;

    struct SoftBodySpringSnapshot {
        int a = 0, b = 0;
        float restLength = 0.0f, stiffness = 0.0f, damping = 0.0f;
    };
    struct SoftBodySnapshot {
        std::vector<std::string> particleNames;
        std::vector<SoftBodySpringSnapshot> springs;
    };
    std::vector<SoftBodySnapshot> softBodySnapshot_;

    // --- Spring connections ------------------------------------------------
    // Ties any two *existing* bodies together (any shape, any BodyType)
    // without replacing either one -- unlike softBodies_ above, which are
    // whole clusters of auto-generated particles. This is the general
    // mechanism the Inspector's Connections section and Box Select's
    // "Connect Selected with Springs" use, and what world:create_spring()
    // exposes to scripts. Persists through Play/Stop/Reset the same way
    // softBodies_ does: by body *name*, re-resolved after a restore rather
    // than storing raw pointers (see springJointSnapshot_).
    std::vector<SpringJoint> springJoints_;
    float springJointStiffness_ = 50.0f;
    float springJointDamping_ = 2.0f;
    // Inspector's Connections combo selection -- nulled in world_.onBodyRemoved
    // if the picked body is deleted before Connect is clicked.
    Body* connectTarget_ = nullptr;

    struct SpringJointSnapshot {
        std::string nameA, nameB;
        float restLength = 0.0f, stiffness = 0.0f, damping = 0.0f;
    };
    std::vector<SpringJointSnapshot> springJointSnapshot_;

    // --- Selection / spawn / drag ---------------------------------------
    Body* selected_ = nullptr;
    Body* lastInspected_ = nullptr;
    // Populated by the Box Select tool (see finishBoxSelect()); more than
    // one entry means a group is selected, highlighted together in the
    // viewport and moved together when any one of them is dragged.
    std::vector<Body*> multiSelected_;
    SpawnTool spawnTool_ = SpawnTool::None;
    SpawnShapeKind spawnShapeKind_ = SpawnShapeKind::Circle;
    float spawnRadius_ = 0.5f; // circumradius for Circle/Triangle/Pentagon/Hexagon
    float spawnHalfWidth_ = 0.5f;
    float spawnHalfHeight_ = 0.5f;

    Body* dragged_ = nullptr;
    Vec2 dragLocalAnchor_;
    Vec2 mouseWorld_;

    // --- Resize / Rotate tools: click a body then drag to reshape/spin it -
    Body* toolDragBody_ = nullptr;

    // --- Box Select marquee ----------------------------------------------
    bool boxSelecting_ = false;
    Vec2 boxSelectStart_;
    Vec2 boxSelectCurrent_;

    // --- Viewport Ctrl+C / Ctrl+V: copies the selected body (or the whole
    // Box Select group, preserving their relative layout) as a snapshot of
    // its properties -- reuses BodySnapshot, the same struct
    // captureSnapshot() already uses, rather than a separate type. Pasting
    // creates new bodies offset from the original so they don't land
    // exactly on top of it; repeated Ctrl+V cascades the offset further
    // each time (reset whenever a new Ctrl+C happens).
    std::vector<BodySnapshot> viewportClipboard_;
    int viewportPasteCount_ = 0;

    bool panning_ = false;
    sf::Vector2i lastMousePixel_;

    // --- Viewport panel (render-to-texture, docked like Godot's 2D view) -
    sf::RenderTexture viewportTexture_;
    sf::Vector2u viewportSize_{1, 1};
    sf::Vector2f viewportScreenPos_{0.0f, 0.0f}; // top-left, in OS window pixel coords
    bool viewportHovered_ = false;
    // True only when the Viewport TAB is the currently-active/visible one in
    // its dock node (it shares a node with Code Editor -- see
    // buildDockLayout()) -- ImGui::Begin() returns false for an inactive
    // docked tab, but drawViewportPanel() previously ignored that return
    // value, so viewportScreenPos_ went stale while Code Editor was active
    // and drawUiOverlay() kept drawing the on_gui HUD at that stale
    // position on top of the Code Editor. Gates drawUiOverlay() now.
    bool viewportTabVisible_ = false;
    bool dockLayoutBuilt_ = false;
    bool returnToProjectManager_ = false;

    struct LogLine {
        std::string text;
        bool isError;
    };
    std::deque<LogLine> consoleLines_;

    char scriptPathBuffer_[256] = {};
    std::vector<std::string> availableScripts_; // paths (relative to scriptsDir_) found in the project

    // Editing buffer for the Inspector's "UI Text" field (Body::uiText) --
    // same fixed-char-buffer convention as scriptPathBuffer_ above (this
    // project doesn't pull in imgui_stdlib for a std::string-backed
    // InputText), resynced from the selected body whenever selection changes.
    char uiTextBuffer_[256] = {};

    // Editing buffer for the Inspector's "Texture" field (Body::texturePath)
    // -- same convention as scriptPathBuffer_/uiTextBuffer_ above.
    char texturePathBuffer_[256] = {};

    // Editing buffer for the Settings panel's "Background Texture" field
    // (backgroundTexturePath_).
    char backgroundTexturePathBuffer_[256] = {};

    // --- Project folder (Godot-like: a workspace on disk, separate from
    // the engine's own bundled example scripts, that this app scaffolds on
    // first run and that the FileSystem panel below lets you browse/edit) -
    std::filesystem::path projectDir_;
    std::filesystem::path scriptsDir_;

    // --- FileSystem panel state ------------------------------------------
    bool showNewFolderPopup_ = false;
    bool showNewScriptPopup_ = false;
    std::filesystem::path newItemParentDir_;
    char newItemNameBuffer_[128] = {};
    int nextUiElementId_ = 0; // for unique host-body names/positions, see createUiElement()
    // "Kind" combo for the Hierarchy panel's "Create" button -- index into
    // kCreateKinds (see EditorApp.cpp): physics shapes (Circle/Box/Triangle/
    // Pentagon/Hexagon, spawned at world (0, 0), NOT a UI Element) followed
    // by UI kinds (Button/Text/Checkbox/Slider/Input Field/Panel, created as
    // a UI Element with no script attached -- see createObjectInScene()).
    // There's no such combo in the FileSystem panel -- that one's "New
    // Script" popup keeps its own separate, script-focused Kind combo (see
    // newScriptKindIdx_ below).
    int createKindIdx_ = 0;
    // "..." popup next to Create: only meaningful for physics-shape kinds
    // (ignored for UI kinds, which are never batched or given a Matter/Body
    // Type -- see createObjectInScene()).
    bool showCreateOptionsPopup_ = false;
    MatterKind createMatterKind_ = MatterKind::Rigidbody;
    BodyType createBodyType_ = BodyType::Dynamic;
    // Index into kCreateQuantities (EditorApp.cpp): 1, 5, 10, 25, 50, 100,
    // 500, 1000 -- how many of the selected physics shape Create spawns at
    // once (spread in a small grid around world (0, 0), not stacked
    // exactly on top of each other).
    int createQuantityIdx_ = 0;
    // "Kind" combo for the FileSystem panel's (single, merged) "New Script"
    // popup -- index into kNewScriptKinds (EditorApp.cpp): Plain Script,
    // Button, Text, Checkbox, Slider. Index 0 ("Plain Script") writes an
    // ordinary on_start/on_update stub with no host body, same as this
    // popup always did before UI Elements existed; any other index defers
    // to createUiElement() exactly like the old dedicated button did.
    int newScriptKindIdx_ = 0;
    // Currently selected file/folder (Ctrl+C / Delete act on this) and the
    // last-copied path (Ctrl+V pastes this into the selected folder, or next
    // to the selected file, or into scriptsDir_ root if nothing's selected).
    std::filesystem::path selectedFsPath_;
    std::filesystem::path fsClipboardPath_;

    // --- Code editor: closed by default, docked as a tab alongside the
    // Viewport (same dock node) so only one of the two is ever visible at
    // once -- see buildDockLayout(). -------------------------------------
    TextEditor codeEditor_;
    bool showCodeEditor_ = false;
    bool codeEditorDirty_ = false;
    std::filesystem::path openScriptPath_;
    // Debounced autosave: reset to 0 on every keystroke, and once it's been
    // this many seconds since the last edit with no further changes, the
    // script is written to disk automatically (see drawCodeEditorPanel()).
    float codeEditorAutosaveTimer_ = 0.0f;

    void resetScene();
    // Writes the current edit-mode checkpoint to sceneFilePath_ as JSON.
    void saveSceneToDisk();
    // Reads sceneFilePath_ and rebuilds world_/softBodies_/springJoints_
    // from it (including re-attaching scripts by their saved path). Returns
    // false (and leaves world_ untouched) if the file doesn't exist or
    // fails to parse -- the caller falls back to resetScene() in that case.
    bool loadSceneFromDisk();
    void stepSimulation(float dt);
    void applyDragInteraction();
    Body* pickBodyAt(const Vec2& worldPos) const;
    void spawnAt(const Vec2& worldPos);
    // Assigns stable, unique-per-particle names (so it round-trips through
    // captureSnapshot()/restoreSnapshot(), see softBodySnapshot_) and folds
    // `sb` into softBodies_. Returns a reference to the stored copy.
    SoftBody& registerSoftBody(SoftBody sb);
    void log(const std::string& msg, bool isError);

    // Creates a spring between `a` and `b` (rest length = their current
    // distance apart) -- ties them together without replacing or otherwise
    // modifying either body, unlike a SoftBody cluster. Used by the
    // Inspector's Connections section and world:create_spring().
    void connectWithSpring(Body* a, Body* b, float stiffness, float damping);
    // Connects every consecutive pair in `bodies`, plus skip-one cross
    // braces (same bracing idea as makeSoftBodyRing, so the group resists
    // collapsing flat) -- for Box Select's "Connect Selected with Springs".
    void connectGroupWithSprings(const std::vector<Body*>& bodies, float stiffness, float damping);
    // Removes every SpringJoint touching `b` -- called from onBodyRemoved
    // so a deleted body never leaves a dangling Body* in springJoints_.
    void removeSpringJointsTouching(Body* b);

    void captureSnapshot();
    void restoreSnapshot();
    void rebindScriptEngine();
    void refreshAvailableScripts();
    void bindUiApi(); // registers the `ui.*` Lua table scripts use for on_gui

    void openScriptInEditor(const std::filesystem::path& path);
    void saveCodeEditor();

    void finishBoxSelect();

    void copyViewportSelection();
    void pasteViewportClipboard();

    void copyFsSelection();
    void pasteFsClipboard();
    void deleteFsSelection();
    // Creates `dir/name.lua` (a `kind`-specific on_gui() template -- see
    // uiElementTemplateScript()) and a live, invisible sensor host body in
    // the scene with that script already attached -- so "New UI Element" in
    // the FileSystem panel gives you both a ready-to-edit file AND a
    // selectable object in one action, instead of requiring you to
    // separately create a script and then manually build a host body for it.
    void createUiElement(const std::filesystem::path& dir, const std::string& name, const std::string& kind);
    // The Hierarchy panel's "Create" button. `kind` is one of kCreateKindIds
    // (EditorApp.cpp): a physics shape ("circle"/"box"/"triangle"/
    // "pentagon"/"hexagon"), spawned at world (0, 0) as an ordinary body --
    // NOT a UI Element -- using createMatterKind_/createBodyType_ and
    // however many createQuantityIdx_ currently selects (spread in a small
    // grid, not stacked exactly on top of each other); or a UI kind
    // ("button"/"text"/"checkbox"/"slider"/"input_field"/"panel"), which
    // creates a UI Element host body with NO script attached at all --
    // unlike createUiElement() below, native kinds render themselves
    // directly (see drawNativeUiElement()) rather than needing generated
    // Lua. Never asks for a position: neither a UI Element nor a batch of
    // shapes dropped at a fixed origin needs one the way the Spawn tool's
    // click-to-place shapes do.
    void createObjectInScene(const std::string& kind);
    // Shared by createUiElement() (FileSystem's script-generating flow) and
    // createObjectInScene()'s UI-kind branch -- creates the host body
    // (Static, isSensor, isUiElement) at `pos` with the given circle radius,
    // and sets uiKind plus kind-appropriate default uiValue/uiMin/uiMax.
    // `scriptPath` may be empty (createObjectInScene()'s native UI kinds),
    // in which case no script is attached and the element renders itself
    // natively -- see drawUiOverlay()/drawNativeUiElement().
    Body* spawnUiElementHost(const Vec2& pos, float radius, const std::filesystem::path& scriptPath,
                             const std::string& kind);
    // One physics shape ("circle"/"box"/"triangle"/"pentagon"/"hexagon") at
    // `pos`, with the given BodyType/MatterKind -- shared by
    // createObjectInScene()'s physics-shape branch (which may call this
    // several times for a batch) and spawnAt() (the Spawn tool), which
    // always passes BodyType::Dynamic/defaultSpawnMatterKind_ for just one.
    Body* spawnShapeAt(const Vec2& pos, const std::string& shapeKind, BodyType type, MatterKind matterKind);
    // Draws a UI Element's widget directly in C++ (button/text/checkbox/
    // slider/input_field/panel), reading/writing body.ui_text/ui_value/
    // ui_min/ui_max/texture_path exactly like the equivalent generated Lua
    // template would -- used by drawUiOverlay() for any isUiElement body
    // that has no script attached (created via createObjectInScene()'s UI
    // kinds). A body WITH a script still runs that instead (see
    // drawUiOverlay()); this is only the no-script fallback.
    void drawNativeUiElement(Body& body);
    // Called right after scriptEngine_.attachScript(b, path) at every
    // Inspector attach site (Script Library combo, drag-drop, Attach/Reload
    // button) -- if `path` carries the machine-readable UI-kind marker
    // uiElementTemplateScript() writes at the top of a generated UI Element
    // script (see readUiKindMarker() in EditorApp.cpp), tags `b` as that
    // kind of UI Element so the Inspector's UI Text/Value/Min/Max/Hide Text
    // fields show up immediately -- without this, attaching a marked script
    // to a body that *wasn't* created through createUiElement()/
    // createObjectInScene() would leave it running the right code but
    // missing those fields. Never clears isUiElement/uiKind when no marker
    // is found, so attaching a plain script over an already-configured UI
    // Element body doesn't silently strip its settings.
    void applyUiKindMarkerOnAttach(Body& b, const std::string& path);

public:
    // Copies every recognized image file among `paths` into this project's
    // scripts/ folder (the same root the FileSystem panel browses -- see
    // drawFileTreeNode()), so it shows up there immediately, selectable and
    // manageable like any other file. Anything not a recognized image
    // extension is logged and skipped. Called from main.cpp with whatever
    // FileDropWatcher::poll() reports each frame (files dragged in from the
    // OS file manager) -- kept as a plain path list here so this class
    // doesn't need to know anything about X11/XDND itself.
    void importDroppedFiles(const std::vector<std::filesystem::path>& paths);

private:
    // Shared by importDroppedFiles() (OS drag-and-drop) and the FileSystem
    // panel's "Upload Texture" popup (typed path instead): validates `src`
    // is a regular, recognized image file, then copies it into `destDir`
    // under its own filename, appending "_1"/"_2"/... on a collision rather
    // than overwriting. Logs and returns false on any failure (not a
    // recognized image, doesn't exist, copy error); true on success.
    bool importImageFile(const std::filesystem::path& src, const std::filesystem::path& destDir);
    // Opens the native OS "Open File" dialog (tinyfiledialogs), filtered to
    // kImageExtensions, and importImageFile()s whatever the user picks into
    // `destDir`. No-op (just returns) if the dialog is cancelled.
    void uploadTextureVia(const std::filesystem::path& destDir);
    Vec2 viewportPixelToWorld(sf::Vector2i windowPixel) const;

    void buildDockLayout(unsigned int dockspaceId);
    void drawToolbar();
    void drawSettings();
    void drawViewportPanel();
    void drawHierarchy();
    void drawCreateOptionsPopup(); // Hierarchy's "..." button -- Matter Kind/Body Type/Quantity for Create
    void drawInspector();
    void drawConsole();
    void drawFileSystemPanel();
    void drawFileTreeNode(const std::filesystem::path& dir);
    void drawFileSystemPopups();
    void drawCodeEditorPanel();
    void drawScriptUiPanel();
    void drawUiOverlay(); // scriptable on_gui() rendered directly onto the Viewport
};

} // namespace p2d::app
