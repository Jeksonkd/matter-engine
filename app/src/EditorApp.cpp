#include "EditorApp.hpp"
#include "ProjectPaths.hpp"
#include "ScenePersistence.hpp"

#include <imgui-SFML.h> // ImGui::Image(sf::RenderTexture, ...) overload used by the viewport panel
#include <imgui.h>
#include <imgui_internal.h> // DockBuilder* -- programmatic default layout, standard practice
#include <tinyfiledialogs.h> // native "Open File" dialog for Upload Texture

#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/Graphics/VertexArray.hpp>
#include <SFML/Window/Mouse.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>

namespace p2d::app {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kAutosaveDelaySeconds = 2.0f;      // code editor autosave debounce
constexpr float kSceneAutosaveIntervalSeconds = 3.0f; // scene-to-disk autosave, see saveSceneToDisk()

// ImGui doesn't wrap a widget + its label onto two lines on its own -- if a
// panel (e.g. the Inspector, docked narrow) is too tight for both, the
// widget just keeps shrinking until the label overlaps it or clips past the
// window edge. This draws the label on its own line first and the widget
// (unlabeled -- the visible text already came from the line above) on the
// next when there isn't enough room for both side by side, matching how
// ImGui's own default "widget, then label to its right" layout looks when
// there IS room. `drawWidget` is called with whichever label it should
// actually pass to the real ImGui call.
template <typename Fn>
void responsiveField(const char* label, Fn&& drawWidget) {
    constexpr float kMinWidgetWidth = 100.0f;
    float labelWidth = ImGui::CalcTextSize(label).x;
    float avail = ImGui::GetContentRegionAvail().x;
    if (avail - labelWidth - ImGui::GetStyle().ItemInnerSpacing.x < kMinWidgetWidth) {
        ImGui::TextUnformatted(label);
        ImGui::SetNextItemWidth(-1.0f);
        std::string hiddenLabel = std::string("##") + label;
        drawWidget(hiddenLabel.c_str());
    } else {
        drawWidget(label);
    }
}

// Several Inspector rows chain more than one widget with ImGui::SameLine()
// (e.g. a slider then a button) -- plain SameLine() still overflows past
// the window edge if the next widget doesn't actually fit, same problem as
// above. Only calls SameLine() if `neededWidth` (a rough estimate of what
// the next widget will take) actually fits in what's left of the line;
// otherwise it starts on a fresh line by itself instead.
//
// NOTE: this deliberately does NOT use GetContentRegionAvail() -- by the
// time this runs (right after the previous widget, before the next one),
// the cursor has already auto-advanced to the START of the next line (that's
// what happens after any item unless SameLine() is called before the
// following one), so GetContentRegionAvail() at this point reports the
// FULL line width every time, not "what's left after the previous widget" --
// which made this always true and SameLine() always fire, regardless of fit
// (the bug this fixes: joint/connection controls were always crammed onto
// one line, invisible without widening the panel). The actual fix is the
// standard ImGui "wrapping buttons" idiom: predict where the next item's
// right edge WOULD land if placed beside the previous one (via
// GetItemRectMax(), the previous item's real screen-space bounds) and only
// commit to that with SameLine() if it'd still fit inside the window.
void sameLineIfFits(float neededWidth) {
    float windowVisibleRight = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
    float lastItemRight = ImGui::GetItemRectMax().x;
    float nextItemRight = lastItemRight + ImGui::GetStyle().ItemSpacing.x + neededWidth;
    if (nextItemRight < windowVisibleRight) {
        ImGui::SameLine();
    }
}

// Rough on-screen width of a button with this label, for sameLineIfFits().
float buttonWidthEstimate(const char* label) {
    return ImGui::CalcTextSize(label).x + ImGui::GetStyle().FramePadding.x * 2.0f;
}

// Image file extensions recognized as texture material -- shared by
// importDroppedFiles() (deciding what a drag-and-drop from the OS file
// manager is allowed to be) and the FileSystem panel (deciding which entries
// become an "IMAGE_PATH" drag source for the Inspector/Settings texture
// fields).
const std::vector<std::string> kImageExtensions = {".png", ".jpg", ".jpeg", ".bmp", ".gif", ".tga"};

// The Hierarchy panel's "Create" button Kind combo: physics shapes first
// (spawned at world (0, 0), NOT a UI Element), then UI kinds (created as a
// UI Element with no script attached -- see createObjectInScene()).
const char* kCreateKinds[] = {"Circle",    "Box",      "Triangle", "Pentagon",    "Hexagon",
                               "Button",    "Text",     "Checkbox", "Slider",      "Input Field", "Panel"};
const char* kCreateKindIds[] = {"circle",     "box",        "triangle",     "pentagon", "hexagon",
                                 "button",     "text",       "checkbox",     "slider",   "input_field", "panel"};
constexpr int kCreateKindCount = 11;

bool isUiElementCreateKind(const std::string& kind) {
    return kind == "button" || kind == "text" || kind == "checkbox" || kind == "slider" || kind == "input_field" ||
           kind == "panel";
}

// Presets for the Create "..." popup's Quantity combo -- how many of the
// selected physics shape a single Create click spawns (spread in a small
// grid around world (0, 0)). Deliberately skips 2-4: either you want
// exactly one, or a real batch.
const int kCreateQuantities[] = {1, 5, 10, 25, 50, 100, 500, 1000};
const char* kCreateQuantityLabels[] = {"1", "5", "10", "25", "50", "100", "500", "1000"};
constexpr int kCreateQuantityCount = 8;

// The FileSystem panel's (single, merged) "New Script" popup's Kind combo --
// index 0 ("Plain Script", an empty kind id) writes an ordinary
// on_start/on_update stub with no host body; any other index defers to
// createUiElement() exactly like the old dedicated "New UI Element" button
// did, just from the same popup instead of a separate one.
const char* kNewScriptKinds[] = {"Plain Script", "Button", "Text", "Checkbox", "Slider"};
const char* kNewScriptKindIds[] = {"", "button", "text", "checkbox", "slider"};
constexpr int kNewScriptKindCount = 5;

// Written as the literal first line of every script uiElementTemplateScript()
// generates -- a machine-readable tag (deliberately NOT the same as the
// human-readable "-- A UI element (kind: ...)" comment below it) that
// readUiKindMarker() looks for, so the FileSystem panel can badge a UI
// Element script at a glance and applyUiKindMarkerOnAttach() can recognize
// one even when it's dragged onto a body that wasn't created through
// createUiElement() (this is the FileSystem panel's script-generating flow
// only now -- the Hierarchy panel's "Create" button/createObjectInScene()
// creates UI kinds natively, with no script/marker at all -- see
// drawNativeUiElement()).
const char* kUiKindMarkerPrefix = "-- @p2d_ui_kind: ";

// Used by createUiElement() (FileSystem panel's "New Script" popup, when its
// Kind combo picks something other than Plain Script) -- writes this
// kind-specific template to a script file AND attaches it to a host body.
// Every kind reads its label from body.ui_text
// (unless body.ui_hide_text is set -- see Inspector's "Hide Text", not
// offered for "text" itself since its whole purpose is the label) and, if
// body.texture_path is set (Inspector's Appearance > Texture field), draws
// that image via ui.image/ui.image_button instead of a plain widget.
// checkbox/slider additionally read/write body.ui_value (and, for slider,
// body.ui_min/ui_max) every frame, so toggling/dragging the live widget and
// editing the Inspector's fields both land in the same place.
std::string uiElementTemplateScript(const std::string& kind) {
    std::string marker = std::string(kUiKindMarkerPrefix) + kind + "\n";
    std::string header = "-- A UI element (kind: " + kind +
                          "): on_gui(body) runs once per rendered frame.\n"
                          "-- body.ui_text is editable from the Inspector's \"UI Text\" field, so its\n"
                          "-- label can be customized without editing this script. body.texture_path\n"
                          "-- (Appearance > Texture) swaps in an image; body.ui_hide_text (\"Hide Text\")\n"
                          "-- then drops the label alongside it.\n";
    if (kind == "text") {
        return marker + header +
               "function on_gui(body)\n"
               "    if body.texture_path ~= \"\" then\n"
               "        ui.image(body.texture_path, 24)\n"
               "        ui.same_line()\n"
               "    end\n"
               "    ui.text(body.ui_text)\n"
               "end\n";
    }
    if (kind == "checkbox") {
        return marker + header +
               "-- body.ui_value is the checked state (0.0/1.0) -- also editable from the\n"
               "-- Inspector's \"UI Value\" field.\n"
               "function on_gui(body)\n"
               "    if body.texture_path ~= \"\" then\n"
               "        ui.image(body.texture_path, 24)\n"
               "        ui.same_line()\n"
               "    end\n"
               "    local label = body.ui_hide_text and (\"##\" .. body.name) or body.ui_text\n"
               "    body.ui_value = ui.checkbox(label, body.ui_value > 0.5) and 1.0 or 0.0\n"
               "end\n";
    }
    if (kind == "slider") {
        return marker + header +
               "-- body.ui_value/ui_min/ui_max are all editable from the Inspector.\n"
               "function on_gui(body)\n"
               "    if body.texture_path ~= \"\" then\n"
               "        ui.image(body.texture_path, 24)\n"
               "        ui.same_line()\n"
               "    end\n"
               "    local label = body.ui_hide_text and (\"##\" .. body.name) or body.ui_text\n"
               "    body.ui_value = ui.slider_float(label, body.ui_value, body.ui_min, body.ui_max)\n"
               "end\n";
    }
    // "button", and the default for anything unrecognized.
    return marker + header +
           "function on_gui(body)\n"
           "    local clicked\n"
           "    if body.texture_path ~= \"\" then\n"
           "        clicked = ui.image_button(body.texture_path, body.name, 32)\n"
           "        if not body.ui_hide_text then\n"
           "            ui.same_line()\n"
           "            ui.text(body.ui_text)\n"
           "        end\n"
           "    else\n"
           "        clicked = ui.button(body.ui_text)\n"
           "    end\n"
           "    if clicked then\n"
           "        print(\"Clicked!\")\n"
           "    end\n"
           "end\n";
}

// Written once into a brand-new project's scripts folder by resetScene()
// (see its "5 starter balls" comment) so a fresh project always has a
// working example of the same on_gui/ui.button pattern uiElementTemplateScript()
// generates, but with real behavior instead of just printing "Clicked!" --
// same marker convention (so it's badged/recognized like any other UI
// Element script if dragged onto another body later).
std::string ballSpawnerScript() {
    return std::string(kUiKindMarkerPrefix) + "button\n" +
           "-- Starter-project ball spawner: spawns one more ball, at a random\n"
           "-- position/size/velocity, each time the button below is clicked.\n"
           "-- body.ui_text is editable from the Inspector's \"UI Text\" field, so its\n"
           "-- label can be customized without editing this script. body.texture_path\n"
           "-- (Appearance > Texture) swaps in an image; body.ui_hide_text (\"Hide Text\")\n"
           "-- then drops the label alongside it.\n"
           "function on_gui(body)\n"
           "    local clicked\n"
           "    if body.texture_path ~= \"\" then\n"
           "        clicked = ui.image_button(body.texture_path, body.name, 32)\n"
           "        if not body.ui_hide_text then\n"
           "            ui.same_line()\n"
           "            ui.text(body.ui_text)\n"
           "        end\n"
           "    else\n"
           "        clicked = ui.button(body.ui_text)\n"
           "    end\n"
           "\n"
           "    if clicked then\n"
           "        local x = (math.random() - 0.5) * 12.0\n"
           "        local radius = 0.25 + math.random() * 0.25\n"
           "        local ball = world:create_circle(x, 9.0, radius, BodyType.Dynamic)\n"
           "        ball.restitution = 0.5\n"
           "        ball.friction = 0.3\n"
           "        ball:set_velocity((math.random() - 0.5) * 2.0, 0.0)\n"
           "    end\n"
           "end\n";
}

// Peeks the first line of `path` for the kUiKindMarkerPrefix marker
// uiElementTemplateScript() writes -- returns the kind ("button"/"text"/
// "checkbox"/"slider") if found, empty otherwise. Cheap (one line read, not
// a full parse) since it's called both for every visible FileSystem .lua
// entry each frame and on every script attach.
std::string readUiKindMarker(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) return {};
    std::string firstLine;
    std::getline(in, firstLine);
    size_t prefixLen = std::char_traits<char>::length(kUiKindMarkerPrefix);
    if (firstLine.compare(0, prefixLen, kUiKindMarkerPrefix) != 0) return {};
    return firstLine.substr(prefixLen);
}
}

EditorApp::EditorApp(std::filesystem::path projectDir) : projectDir_(std::move(projectDir)) {
    // Set up the callbacks/hook exactly once: they capture `this` (stable
    // for the app's lifetime) and don't depend on which ScriptEngine::reset()
    // generation is currently live, so there's no need to redo this on every
    // scene reset.
    scriptEngine_.onLog = [this](const std::string& m) { log(m, false); };
    scriptEngine_.onError = [this](const std::string& m) { log(m, true); };
    world_.onBodyRemoved = [this](Body* b) {
        if (!b) return;
        scriptEngine_.detachScript(*b);

        // If this was a soft-body particle, drop it (and any spring
        // touching it) from its SoftBody so applySpringForces() never
        // dereferences the now-dangling Body* -- e.g. deleting a single
        // particle via the Inspector's "Delete Body".
        for (auto& sb : softBodies_) {
            auto it = std::find(sb.particles.begin(), sb.particles.end(), b);
            if (it == sb.particles.end()) continue;
            int idx = static_cast<int>(it - sb.particles.begin());
            sb.particles.erase(it);
            for (auto sIt = sb.springs.begin(); sIt != sb.springs.end();) {
                if (sIt->a == idx || sIt->b == idx) {
                    sIt = sb.springs.erase(sIt);
                } else {
                    if (sIt->a > idx) --sIt->a;
                    if (sIt->b > idx) --sIt->b;
                    ++sIt;
                }
            }
            break;
        }

        // A deleted body may also be one end of a standalone spring
        // connection (see springJoints_) -- drop those too rather than
        // leave a dangling Body*.
        removeSpringJointsTouching(b);
        if (connectTarget_ == b) connectTarget_ = nullptr;
    };
    // Soft body / spring-joint forces MUST be recomputed every internal
    // substep (not just once before world_.step()) -- see onPreSubstep's
    // doc comment in World.hpp for why applying them only once let a soft
    // body gain energy indefinitely instead of settling once it moved fast
    // enough to trigger substepping.
    world_.onPreSubstep = [this](float) {
        for (auto& sb : softBodies_) sb.applySpringForces();
        for (auto& j : springJoints_) j.applyForce();
    };

    codeEditor_.SetLanguageDefinition(TextEditor::LanguageDefinition::Lua());
    codeEditor_.SetPalette(TextEditor::GetDarkPalette());
    codeEditor_.SetTabSize(4);

    scriptsDir_ = projectDir_ / "scripts";
    ensureProjectScaffold(scriptsDir_, [this](const std::string& msg, bool isErr) { log(msg, isErr); });

    refreshAvailableScripts();
    sceneFilePath_ = projectDir_ / "scene.json";
    if (!loadSceneFromDisk()) resetScene(); // no saved scene yet (or it failed to load) -- blank sandbox
    captureSnapshot(); // seed a valid restore point immediately (see Reset Scene)
}

EditorApp::~EditorApp() {
    // Only the edit-mode state is ever meant to persist (see
    // saveSceneToDisk()'s callers elsewhere) -- if the app is closed while
    // mid-Play, the last pre-Play autosave (captured continuously by
    // update() before Play started) is the right thing left on disk, not
    // whatever the live simulation happens to look like at this instant.
    if (!running_) saveSceneToDisk();
}

void EditorApp::rebindScriptEngine() {
    scriptEngine_.reset();
    scriptEngine_.bindWorld(world_);
    bindUiApi();
}

void EditorApp::refreshAvailableScripts() {
    availableScripts_.clear();
    std::error_code ec;
    auto opts = std::filesystem::directory_options::skip_permission_denied;
    for (auto& entry : std::filesystem::recursive_directory_iterator(scriptsDir_, opts, ec)) {
        if (ec) break;
        if (entry.is_regular_file() && entry.path().extension() == ".lua") {
            std::error_code relEc;
            auto rel = std::filesystem::relative(entry.path(), scriptsDir_, relEc);
            availableScripts_.push_back(relEc ? entry.path().filename().string() : rel.string());
        }
    }
    std::sort(availableScripts_.begin(), availableScripts_.end());
}

void EditorApp::bindUiApi() {
    sol::state& lua = scriptEngine_.lua();
    sol::table uiTable = lua.create_table();

    // Deliberately maps 1:1 onto immediate-mode ImGui idioms (a script
    // calls ui.button("Spawn Cube") once per frame from on_gui, and it
    // returns true exactly on the frame the button is clicked) rather than
    // a retained-mode "add this widget once" API -- this keeps the Lua side
    // as simple as calling ImGui directly would be in C++.
    uiTable.set_function("button", [](const std::string& label) { return ImGui::Button(label.c_str()); });
    uiTable.set_function("text", [](const std::string& text) { ImGui::TextUnformatted(text.c_str()); });
    uiTable.set_function("same_line", []() { ImGui::SameLine(); });
    uiTable.set_function("separator", []() { ImGui::Separator(); });
    uiTable.set_function("checkbox", [](const std::string& label, bool value) {
        ImGui::Checkbox(label.c_str(), &value);
        return value;
    });
    uiTable.set_function("slider_float", [](const std::string& label, float value, float min, float max) {
        ImGui::SliderFloat(label.c_str(), &value, min, max);
        return value;
    });

    // Textured widgets -- shares Renderer's own texture cache (getOrLoadTexture
    // is public for exactly this) rather than loading a second copy of the
    // same file. `path` empty or unloadable falls back to a plain disabled-
    // looking placeholder (image()) or a plain ui.button() (image_button()),
    // rather than silently drawing nothing where a script expects a widget.
    uiTable.set_function("image", [this](const std::string& path, float size) {
        sf::Texture* tex = renderer_.getOrLoadTexture(path);
        if (!tex) {
            ImGui::TextDisabled("[missing image]");
            return;
        }
        ImGui::Image(*tex, sf::Vector2f(size, size));
    });
    uiTable.set_function("image_button", [this](const std::string& path, const std::string& id, float size) {
        sf::Texture* tex = renderer_.getOrLoadTexture(path);
        if (!tex) return ImGui::Button(id.c_str());
        return ImGui::ImageButton(id.c_str(), *tex, sf::Vector2f(size, size));
    });

    // Generic interaction detection on whatever widget was drawn last (any
    // ui.* call above) -- e.g. `ui.text("Click me")` isn't its own button,
    // but `if ui.is_item_clicked() then ... end` right after it detects a
    // click the same way ui.button()'s return value does for an actual
    // button. is_item_active additionally distinguishes "still being held
    // down" from a single completed click/hover.
    uiTable.set_function("is_item_hovered", []() { return ImGui::IsItemHovered(); });
    uiTable.set_function("is_item_clicked", []() { return ImGui::IsItemClicked(); });
    uiTable.set_function("is_item_active", []() { return ImGui::IsItemActive(); });

    lua["ui"] = uiTable;

    // Soft body spawning from scripts -- a separate global table (not
    // world:create_*) because a soft body needs EditorApp's own bookkeeping
    // (registerSoftBody(), see softBodies_) to actually simulate/persist
    // through Play-Stop-Reset, which plain World-level bindings can't reach.
    // Each returns a 1-indexed table of every particle Body created (the hub
    // is particles[1] for a jelly), so a script can style/inspect/attach
    // scripts to them the same way it would any other body.
    sol::table softBodyTable = lua.create_table();
    auto particlesToTable = [&lua](const SoftBody& sb) {
        sol::table t = lua.create_table(static_cast<int>(sb.particles.size()), 0);
        int i = 1;
        for (Body* p : sb.particles) t[i++] = p;
        return t;
    };
    softBodyTable.set_function("create_ring", [this, particlesToTable](float x, float y, float radius,
                                                                        int segments) {
        SoftBody& sb = registerSoftBody(makeSoftBodyRing(world_, Vec2(x, y), radius, segments));
        log("Script spawned a soft body ring (" + std::to_string(sb.particles.size()) + " particles).", false);
        return particlesToTable(sb);
    });
    softBodyTable.set_function("create_jelly", [this, particlesToTable](float x, float y, float radius,
                                                                         int segments) {
        SoftBody& sb = registerSoftBody(makeSoftBodyJelly(world_, Vec2(x, y), radius, segments));
        log("Script spawned a soft body jelly (" + std::to_string(sb.particles.size()) + " particles).", false);
        return particlesToTable(sb);
    });
    softBodyTable.set_function("create_cloth", [this, particlesToTable](float x, float y, int cols, int rows,
                                                                         float spacing, bool pinTop) {
        SoftBody& sb = registerSoftBody(makeSoftBodyCloth(world_, Vec2(x, y), cols, rows, spacing, 0.15f, 50.0f,
                                                           2.0f, pinTop));
        log("Script spawned a " + std::to_string(cols) + "x" + std::to_string(rows) + " cloth.", false);
        return particlesToTable(sb);
    });
    lua["soft_body"] = softBodyTable;

    // Ties any two existing bodies together with a spring -- the general
    // mechanism behind Cloth/Jelly's own particle connections, but usable
    // on ordinary objects already in the scene (any shape, any BodyType)
    // without replacing either one. A bare global function (like
    // attach_script) rather than a World method, for the same reason
    // soft_body.* is: it needs EditorApp's springJoints_ bookkeeping to
    // actually simulate/persist through Play-Stop-Reset.
    lua.set_function("create_spring", [this](Body* a, Body* b, float stiffness, float damping) {
        connectWithSpring(a, b, stiffness, damping);
    });
}

void EditorApp::resetScene() {
    world_.clear();
    selected_ = nullptr;
    lastInspected_ = nullptr;
    dragged_ = nullptr;
    multiSelected_.clear();
    toolDragBody_ = nullptr;
    accumulator_ = 0.0f;
    running_ = false;
    everPlayedSinceCheckpoint_ = false;
    hasSnapshot_ = false;
    playSnapshot_.clear();
    softBodies_.clear(); // world_.clear() just invalidated every particle Body* they held
    softBodySnapshot_.clear();
    springJoints_.clear();
    springJointSnapshot_.clear();

    rebindScriptEngine();

    // A small starter sandbox: a floor and two walls to build on, 5 balls
    // already resting/dropping in it, and a "Spawn Ball" button (its
    // script written into this project's own scripts folder, so it's a
    // real, editable file from the very first launch) -- something to
    // press Play on immediately, rather than a completely empty scene.
    world_.gravity = Vec2(0.0f, -9.81f);

    Body* floor = world_.createBox({0.0f, 0.0f}, 9.0f, 0.5f, BodyType::Static);
    floor->name = "floor";
    Body* wallL = world_.createBox({-8.5f, 5.0f}, 0.5f, 5.0f, BodyType::Static);
    wallL->name = "wall_left";
    Body* wallR = world_.createBox({8.5f, 5.0f}, 0.5f, 5.0f, BodyType::Static);
    wallR->name = "wall_right";

    for (int i = 0; i < 5; ++i) {
        float x = -4.0f + static_cast<float>(i) * 2.0f;
        Body* ball = world_.createCircle({x, 3.0f + static_cast<float>(i) * 0.5f}, 0.4f, BodyType::Dynamic);
        ball->name = "ball_" + std::to_string(i + 1);
        ball->restitution = 0.5f;
        ball->friction = 0.3f;
    }

    // Only written the first time a project is ever created -- if it
    // already exists (a later resetScene() call, or the user's own edits
    // to it), it's reused as-is rather than clobbered.
    std::filesystem::path spawnerPath = scriptsDir_ / "spawn_balls.lua";
    std::error_code existsEc;
    if (!std::filesystem::exists(spawnerPath, existsEc)) {
        std::ofstream out(spawnerPath);
        out << ballSpawnerScript();
        out.close();
        refreshAvailableScripts();
    }
    Body* spawner = spawnUiElementHost({0.0f, 200.0f}, 0.01f, spawnerPath, "button");
    spawner->name = "ball_spawner";
    spawner->uiText = "Spawn Ball";

    log("Scene reset: floor, two walls, 5 balls, and a Spawn Ball button. Press Play, or drag\n"
        "any body to place it first.",
        false);
}

void EditorApp::captureSnapshot() {
    playSnapshot_.clear();
    for (auto& bp : world_.bodies()) {
        Body& b = *bp;
        BodySnapshot snap;
        snap.shape = b.shape;
        snap.position = b.position;
        snap.rotation = b.rotation;
        snap.velocity = b.velocity;
        snap.angularVelocity = b.angularVelocity;
        snap.density = b.density;
        snap.restitution = b.restitution;
        snap.friction = b.friction;
        snap.linearDamping = b.linearDamping;
        snap.angularDamping = b.angularDamping;
        snap.gravityScale = b.gravityScale;
        snap.fixedRotation = b.fixedRotation;
        snap.isSensor = b.isSensor;
        snap.type = b.type;
        snap.colorR = b.colorR;
        snap.colorG = b.colorG;
        snap.colorB = b.colorB;
        snap.texturePath = b.texturePath;
        snap.matterKind = b.matterKind;
        snap.isUiElement = b.isUiElement;
        snap.uiKind = b.uiKind;
        snap.uiText = b.uiText;
        snap.uiValue = b.uiValue;
        snap.uiMin = b.uiMin;
        snap.uiMax = b.uiMax;
        snap.uiHideText = b.uiHideText;
        snap.uiOverlayX = b.uiOverlayX;
        snap.uiOverlayY = b.uiOverlayY;
        snap.name = b.name;
        snap.scriptPath = b.scriptPath;
        playSnapshot_.push_back(std::move(snap));
    }
    hasSnapshot_ = true;

    // Soft body spring topology can't be stored as raw Body* (restoring
    // recreates every Body from scratch), so it's captured by particle
    // *name* instead and re-resolved via World::findByName() on restore.
    softBodySnapshot_.clear();
    for (auto& sb : softBodies_) {
        SoftBodySnapshot snap;
        snap.particleNames.reserve(sb.particles.size());
        for (Body* p : sb.particles) snap.particleNames.push_back(p->name);
        snap.springs.reserve(sb.springs.size());
        for (auto& s : sb.springs) {
            snap.springs.push_back({s.a, s.b, s.restLength, s.stiffness, s.damping});
        }
        softBodySnapshot_.push_back(std::move(snap));
    }

    // Same name-based approach for standalone spring connections -- see
    // springJoints_'s doc comment. (Relies on names being unique enough to
    // round-trip correctly, same caveat as World::findByName/find_body
    // generally: an ambiguous duplicate name reconnects to whichever body
    // happens to match first.)
    springJointSnapshot_.clear();
    for (auto& j : springJoints_) {
        if (!j.a || !j.b) continue;
        springJointSnapshot_.push_back({j.a->name, j.b->name, j.restLength, j.stiffness, j.damping});
    }
}

void EditorApp::restoreSnapshot() {
    if (!hasSnapshot_) return;

    world_.clear();
    selected_ = nullptr;
    lastInspected_ = nullptr;
    dragged_ = nullptr;
    multiSelected_.clear();
    toolDragBody_ = nullptr;
    accumulator_ = 0.0f;
    softBodies_.clear(); // world_.clear() just invalidated every particle Body* they held
    springJoints_.clear();

    rebindScriptEngine();

    for (auto& snap : playSnapshot_) {
        Body* b = world_.createBody(snap.shape, snap.position, snap.type);
        b->rotation = snap.rotation;
        b->velocity = snap.velocity;
        b->angularVelocity = snap.angularVelocity;
        b->density = snap.density;
        b->fixedRotation = snap.fixedRotation;
        b->computeMass();
        b->restitution = snap.restitution;
        b->friction = snap.friction;
        b->linearDamping = snap.linearDamping;
        b->angularDamping = snap.angularDamping;
        b->gravityScale = snap.gravityScale;
        b->isSensor = snap.isSensor;
        b->colorR = snap.colorR;
        b->colorG = snap.colorG;
        b->colorB = snap.colorB;
        b->texturePath = snap.texturePath;
        b->matterKind = snap.matterKind;
        b->isUiElement = snap.isUiElement;
        b->uiKind = snap.uiKind;
        b->uiText = snap.uiText;
        b->uiValue = snap.uiValue;
        b->uiMin = snap.uiMin;
        b->uiMax = snap.uiMax;
        b->uiHideText = snap.uiHideText;
        b->uiOverlayX = snap.uiOverlayX;
        b->uiOverlayY = snap.uiOverlayY;
        b->name = snap.name;
        if (!snap.scriptPath.empty()) {
            scriptEngine_.attachScript(*b, snap.scriptPath);
        }
    }

    // Re-resolve soft body particles by name now that they exist again.
    for (auto& sbSnap : softBodySnapshot_) {
        SoftBody sb;
        sb.particles.reserve(sbSnap.particleNames.size());
        bool ok = true;
        for (auto& name : sbSnap.particleNames) {
            Body* p = world_.findByName(name);
            if (!p) {
                ok = false;
                break;
            }
            sb.particles.push_back(p);
        }
        if (!ok) continue; // shouldn't happen; skip this soft body rather than risk a dangling ref
        sb.springs.reserve(sbSnap.springs.size());
        for (auto& s : sbSnap.springs) sb.springs.push_back({s.a, s.b, s.restLength, s.stiffness, s.damping});
        softBodies_.push_back(std::move(sb));
    }

    // Re-resolve standalone spring connections by name now that bodies
    // exist again.
    for (auto& jSnap : springJointSnapshot_) {
        Body* a = world_.findByName(jSnap.nameA);
        Body* b = world_.findByName(jSnap.nameB);
        if (!a || !b) continue; // shouldn't happen; skip rather than risk a dangling ref
        springJoints_.push_back({a, b, jSnap.restLength, jSnap.stiffness, jSnap.damping});
    }

    log("Stopped. Scene restored to the state before Play.", false);
}

void EditorApp::saveSceneToDisk() {
    nlohmann::json j = saveScene(world_, softBodies_, springJoints_);
    // Background settings live on EditorApp, not World, so they're stored as
    // extra top-level keys here rather than through ScenePersistence's public
    // API -- that module is deliberately kept free of anything EditorApp/UI-
    // specific so it stays headlessly testable (see ScenePersistence.hpp).
    j["backgroundColorR"] = backgroundColorR_;
    j["backgroundColorG"] = backgroundColorG_;
    j["backgroundColorB"] = backgroundColorB_;
    j["backgroundTexturePath"] = backgroundTexturePath_;
    j["backgroundTiled"] = backgroundTiled_;
    std::ofstream out(sceneFilePath_, std::ios::trunc);
    if (!out) {
        log("Could not autosave scene to " + sceneFilePath_.string(), true);
        return;
    }
    out << j.dump(2);
}

bool EditorApp::loadSceneFromDisk() {
    std::ifstream in(sceneFilePath_);
    if (!in) return false; // no saved scene yet -- not an error, just a brand new project

    nlohmann::json j;
    try {
        in >> j;
    } catch (const std::exception& e) {
        log("Saved scene file is corrupt (" + std::string(e.what()) + "); starting with a blank scene.", true);
        return false;
    }

    selected_ = nullptr;
    lastInspected_ = nullptr;
    dragged_ = nullptr;
    multiSelected_.clear();
    toolDragBody_ = nullptr;
    accumulator_ = 0.0f;
    rebindScriptEngine();

    bool ok = loadScene(j, world_, softBodies_, springJoints_,
                        [this](Body& b, const std::string& path) {
                            if (std::filesystem::exists(path)) scriptEngine_.attachScript(b, path);
                        });
    if (!ok) {
        log("Saved scene file is malformed; starting with a blank scene.", true);
        return false;
    }

    // Falls back to the current value (rather than a hardcoded default) so a
    // scene saved before these keys existed loads with the background it
    // already had instead of silently resetting it.
    backgroundColorR_ = j.value("backgroundColorR", backgroundColorR_);
    backgroundColorG_ = j.value("backgroundColorG", backgroundColorG_);
    backgroundColorB_ = j.value("backgroundColorB", backgroundColorB_);
    backgroundTexturePath_ = j.value("backgroundTexturePath", backgroundTexturePath_);
    backgroundTiled_ = j.value("backgroundTiled", backgroundTiled_);

    log("Loaded saved scene from " + sceneFilePath_.filename().string() + " -- scripts reattached as they were.",
        false);
    return true;
}


void EditorApp::stepSimulation(float dt) {
    scriptEngine_.update(world_, dt);
    world_.step(dt); // soft body spring forces are applied via world_.onPreSubstep, set up in the constructor
}

void EditorApp::applyDragInteraction() {
    if (!dragged_) return;

    if (running_) {
        Vec2 anchorWorld = dragged_->toWorldPoint(dragLocalAnchor_);
        Vec2 dir = mouseWorld_ - anchorWorld;
        float m = std::max(dragged_->mass, 0.1f);
        Vec2 spring = dir * (60.0f * m);
        Vec2 damping = dragged_->velocity * (-4.0f * m);
        dragged_->applyForceAtPoint(spring + damping, anchorWorld);
    } else {
        // Edit mode: place the body directly, like moving a node in Godot's editor.
        Vec2 newPos = mouseWorld_ - rotate(dragLocalAnchor_, dragged_->rotation);
        Vec2 delta = newPos - dragged_->position;
        dragged_->position = newPos;
        dragged_->velocity = Vec2(0.0f, 0.0f);
        dragged_->angularVelocity = 0.0f;
        dragged_->wake();

        // Group move: dragging any one member of a multi-selection carries
        // the rest of the group along by the same delta.
        if (multiSelected_.size() > 1) {
            for (Body* b : multiSelected_) {
                if (b == dragged_) continue;
                b->position = b->position + delta;
                b->velocity = Vec2(0.0f, 0.0f);
                b->wake();
            }
        }
    }
}

Body* EditorApp::pickBodyAt(const Vec2& worldPos) const {
    const auto& bodies = world_.bodies();
    for (auto it = bodies.rbegin(); it != bodies.rend(); ++it) {
        Body& b = **it;
        if (b.shape.type == ShapeType::Circle) {
            if ((worldPos - b.position).length() <= b.shape.radius) return &b;
        } else {
            Vec2 local = rotate(worldPos - b.position, -b.rotation);
            bool inside = true;
            for (size_t i = 0; i < b.shape.vertices.size(); ++i) {
                if (b.shape.normals[i].dot(local - b.shape.vertices[i]) > 0.0f) {
                    inside = false;
                    break;
                }
            }
            if (inside) return &b;
        }
    }
    return nullptr;
}

void EditorApp::spawnAt(const Vec2& worldPos) {
    Body* b = nullptr;
    switch (spawnShapeKind_) {
        case SpawnShapeKind::Circle: b = world_.createCircle(worldPos, spawnRadius_, BodyType::Dynamic); break;
        case SpawnShapeKind::Box:
            b = world_.createBox(worldPos, spawnHalfWidth_, spawnHalfHeight_, BodyType::Dynamic);
            break;
        case SpawnShapeKind::Triangle:
            b = world_.createBody(ShapeData::MakeRegularPolygon(3, spawnRadius_), worldPos, BodyType::Dynamic);
            break;
        case SpawnShapeKind::Pentagon:
            b = world_.createBody(ShapeData::MakeRegularPolygon(5, spawnRadius_), worldPos, BodyType::Dynamic);
            break;
        case SpawnShapeKind::Hexagon:
            b = world_.createBody(ShapeData::MakeRegularPolygon(6, spawnRadius_), worldPos, BodyType::Dynamic);
            break;
    }
    if (b) {
        b->name = "spawned_" + std::to_string(b->id);
        b->matterKind = defaultSpawnMatterKind_;
        selected_ = b;
        multiSelected_.clear();
    }
}

SoftBody& EditorApp::registerSoftBody(SoftBody sb) {
    int id = nextSoftBodyId_++;
    for (size_t i = 0; i < sb.particles.size(); ++i) {
        sb.particles[i]->name = "softbody" + std::to_string(id) + "_p" + std::to_string(i);
    }
    softBodies_.push_back(std::move(sb));
    return softBodies_.back();
}

void EditorApp::connectWithSpring(Body* a, Body* b, float stiffness, float damping) {
    if (!a || !b || a == b) return;
    float restLength = (b->position - a->position).length();
    springJoints_.push_back({a, b, restLength, stiffness, damping});
    log("Connected " + (a->name.empty() ? "Body " + std::to_string(a->id) : a->name) + " and " +
            (b->name.empty() ? "Body " + std::to_string(b->id) : b->name) + " with a spring.",
        false);
}

void EditorApp::connectGroupWithSprings(const std::vector<Body*>& bodies, float stiffness, float damping) {
    int n = static_cast<int>(bodies.size());
    if (n < 2) return;
    int connectionsMade = 0;
    for (int i = 0; i < n; ++i) {
        int next = i + 1;
        if (next < n) {
            Body* a = bodies[static_cast<size_t>(i)];
            Body* b = bodies[static_cast<size_t>(next)];
            springJoints_.push_back({a, b, (b->position - a->position).length(), stiffness, damping});
            ++connectionsMade;
        }
        // Skip-one cross brace (not wrapped around, unlike a closed ring --
        // an arbitrary selection isn't necessarily meant to close a loop).
        int acrossOne = i + 2;
        if (acrossOne < n) {
            Body* a = bodies[static_cast<size_t>(i)];
            Body* b = bodies[static_cast<size_t>(acrossOne)];
            springJoints_.push_back({a, b, (b->position - a->position).length(), stiffness * 0.5f, damping});
            ++connectionsMade;
        }
    }
    log("Connected " + std::to_string(n) + " selected bodies with " + std::to_string(connectionsMade) +
            " springs.",
        false);
}

void EditorApp::removeSpringJointsTouching(Body* b) {
    springJoints_.erase(
        std::remove_if(springJoints_.begin(), springJoints_.end(),
                        [b](const SpringJoint& j) { return j.a == b || j.b == b; }),
        springJoints_.end());
}

void EditorApp::finishBoxSelect() {
    Vec2 lo(std::min(boxSelectStart_.x, boxSelectCurrent_.x), std::min(boxSelectStart_.y, boxSelectCurrent_.y));
    Vec2 hi(std::max(boxSelectStart_.x, boxSelectCurrent_.x), std::max(boxSelectStart_.y, boxSelectCurrent_.y));

    multiSelected_.clear();
    for (auto& bodyPtr : world_.bodies()) {
        Body* b = bodyPtr.get();
        if (b->position.x >= lo.x && b->position.x <= hi.x && b->position.y >= lo.y && b->position.y <= hi.y) {
            multiSelected_.push_back(b);
        }
    }
    selected_ = multiSelected_.empty() ? nullptr : multiSelected_.front();
    lastInspected_ = nullptr;
    log("Box-selected " + std::to_string(multiSelected_.size()) + " body(ies).", false);
}

void EditorApp::copyViewportSelection() {
    std::vector<Body*> toCopy;
    if (multiSelected_.size() > 1) {
        toCopy = multiSelected_;
    } else if (selected_) {
        toCopy.push_back(selected_);
    }
    if (toCopy.empty()) return;

    viewportClipboard_.clear();
    viewportPasteCount_ = 0;
    for (Body* b : toCopy) {
        BodySnapshot snap;
        snap.shape = b->shape;
        snap.position = b->position;
        snap.rotation = b->rotation;
        snap.velocity = b->velocity;
        snap.angularVelocity = b->angularVelocity;
        snap.density = b->density;
        snap.restitution = b->restitution;
        snap.friction = b->friction;
        snap.linearDamping = b->linearDamping;
        snap.angularDamping = b->angularDamping;
        snap.gravityScale = b->gravityScale;
        snap.fixedRotation = b->fixedRotation;
        snap.isSensor = b->isSensor;
        snap.type = b->type;
        snap.colorR = b->colorR;
        snap.colorG = b->colorG;
        snap.colorB = b->colorB;
        snap.texturePath = b->texturePath;
        snap.matterKind = b->matterKind;
        snap.isUiElement = b->isUiElement;
        snap.uiKind = b->uiKind;
        snap.uiText = b->uiText;
        snap.uiValue = b->uiValue;
        snap.uiMin = b->uiMin;
        snap.uiMax = b->uiMax;
        snap.uiHideText = b->uiHideText;
        snap.uiOverlayX = b->uiOverlayX;
        snap.uiOverlayY = b->uiOverlayY;
        snap.name = b->name;
        snap.scriptPath = b->scriptPath;
        viewportClipboard_.push_back(std::move(snap));
    }
    log("Copied " + std::to_string(viewportClipboard_.size()) + " bod" +
            (viewportClipboard_.size() == 1 ? "y" : "ies") + ".",
        false);
}

void EditorApp::pasteViewportClipboard() {
    if (viewportClipboard_.empty()) return;

    // Cascades further out each repeated Ctrl+V so pasted copies don't
    // stack exactly on top of each other (or the original) -- reset back
    // to the first offset whenever a fresh Ctrl+C happens.
    ++viewportPasteCount_;
    Vec2 offset(0.6f * static_cast<float>(viewportPasteCount_), 0.6f * static_cast<float>(viewportPasteCount_));

    std::vector<Body*> pasted;
    pasted.reserve(viewportClipboard_.size());
    for (auto& snap : viewportClipboard_) {
        Body* b = world_.createBody(snap.shape, snap.position + offset, snap.type);
        b->rotation = snap.rotation;
        b->velocity = snap.velocity;
        b->angularVelocity = snap.angularVelocity;
        b->density = snap.density;
        b->fixedRotation = snap.fixedRotation;
        b->computeMass();
        b->restitution = snap.restitution;
        b->friction = snap.friction;
        b->linearDamping = snap.linearDamping;
        b->angularDamping = snap.angularDamping;
        b->gravityScale = snap.gravityScale;
        b->isSensor = snap.isSensor;
        b->colorR = snap.colorR;
        b->colorG = snap.colorG;
        b->colorB = snap.colorB;
        b->texturePath = snap.texturePath;
        b->matterKind = snap.matterKind;
        b->isUiElement = snap.isUiElement;
        b->uiKind = snap.uiKind;
        b->uiText = snap.uiText;
        b->uiValue = snap.uiValue;
        b->uiMin = snap.uiMin;
        b->uiMax = snap.uiMax;
        b->uiHideText = snap.uiHideText;
        // Reset rather than inherit: a pasted copy's on_gui window (if any)
        // should cascade to its own new default spot in drawUiOverlay(),
        // not land stacked exactly on top of the original's.
        b->uiOverlayX = -1.0f;
        b->uiOverlayY = -1.0f;
        b->name = snap.name.empty() ? ("spawned_" + std::to_string(b->id)) : (snap.name + "_copy");
        if (!snap.scriptPath.empty() && std::filesystem::exists(snap.scriptPath)) {
            scriptEngine_.attachScript(*b, snap.scriptPath);
        }
        pasted.push_back(b);
    }

    selected_ = pasted.front();
    multiSelected_ = (pasted.size() > 1) ? pasted : std::vector<Body*>{};
    lastInspected_ = nullptr;
    log("Pasted " + std::to_string(pasted.size()) + " bod" + (pasted.size() == 1 ? "y" : "ies") + ".", false);
}

void EditorApp::log(const std::string& msg, bool isError) {
    consoleLines_.push_back({msg, isError});
    if (consoleLines_.size() > 500) consoleLines_.pop_front();
}

void EditorApp::openScriptInEditor(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        log("Could not open " + path.string() + " for editing.", true);
        return;
    }
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    codeEditor_.SetText(content);
    openScriptPath_ = path;
    codeEditorDirty_ = false;
    codeEditorAutosaveTimer_ = 0.0f;
    showCodeEditor_ = true;
    ImGui::SetWindowFocus("Code Editor###CodeEditor");
}

void EditorApp::saveCodeEditor() {
    if (openScriptPath_.empty()) return;

    std::ofstream out(openScriptPath_, std::ios::trunc);
    if (!out) {
        log("Could not save " + openScriptPath_.string(), true);
        return;
    }
    out << codeEditor_.GetText();
    out.close();
    codeEditorDirty_ = false;
    log("Saved " + openScriptPath_.filename().string(), false);

    if (selected_ && selected_->scriptPath == openScriptPath_.string()) {
        scriptEngine_.attachScript(*selected_, openScriptPath_.string());
        log("Re-attached updated script to " + selected_->name, false);
    }
    refreshAvailableScripts();
}

// A dedicated host body for a UI Element -- Static + isSensor so it never
// physically interacts, and isUiElement=true just marks it for the
// Hierarchy tag / Inspector's uiKind-dependent fields (see Body::isUiElement).
// Not a physical object meant to be seen or placed: always off-screen and
// tiny, same "ui_controls" convention as resetScene() uses for its own
// script host -- neither this nor createObjectInScene()/createUiElement()
// take a position for that reason (unlike the Spawn tool's actual shapes).
// `scriptPath` may be empty -- no script gets attached in that case, and the
// element renders itself natively instead (see drawNativeUiElement()).
Body* EditorApp::spawnUiElementHost(const Vec2& pos, float radius, const std::filesystem::path& scriptPath,
                                    const std::string& kind) {
    int id = nextUiElementId_++;
    Body* host = world_.createCircle(pos, radius, BodyType::Static);
    host->name = "UIElement_" + std::to_string(id);
    host->isSensor = true;
    host->isUiElement = true;
    host->uiKind = kind;
    if (kind == "text") host->uiText = "New UI Element";
    else if (kind == "input_field" || kind == "panel") host->uiText.clear();
    else host->uiText = "Click Me";
    if (kind == "slider") {
        host->uiValue = 0.5f;
        host->uiMin = 0.0f;
        host->uiMax = 1.0f;
    }
    if (kind == "panel") {
        // uiMin/uiMax repurposed as width/height in pixels for this kind
        // only -- see drawNativeUiElement() -- same idea as Slider
        // repurposing them as its value range.
        host->uiMin = 220.0f;
        host->uiMax = 140.0f;
    }
    if (!scriptPath.empty()) scriptEngine_.attachScript(*host, scriptPath.string());
    return host;
}

void EditorApp::applyUiKindMarkerOnAttach(Body& b, const std::string& path) {
    std::string kind = readUiKindMarker(path);
    if (kind.empty()) return;
    b.isUiElement = true;
    b.uiKind = kind;
}

void EditorApp::createUiElement(const std::filesystem::path& dir, const std::string& nameIn, const std::string& kind) {
    std::string name = nameIn;
    if (name.size() < 4 || name.compare(name.size() - 4, 4, ".lua") != 0) name += ".lua";

    std::filesystem::path fullPath = dir / name;
    std::error_code existsEc;
    if (std::filesystem::exists(fullPath, existsEc)) {
        log("A file named " + name + " already exists there.", true);
        return;
    }

    std::ofstream out(fullPath);
    out << uiElementTemplateScript(kind);
    out.close();
    refreshAvailableScripts();

    Body* host = spawnUiElementHost({0.0f, 200.0f + static_cast<float>(nextUiElementId_)}, 0.01f, fullPath, kind);

    selected_ = host;
    multiSelected_.clear();
    lastInspected_ = nullptr;
    log("Created UI element '" + name + "' with a host body (" + host->name + "), now selected.", false);
}

Body* EditorApp::spawnShapeAt(const Vec2& pos, const std::string& shapeKind, BodyType type, MatterKind matterKind) {
    Body* b = nullptr;
    if (shapeKind == "circle") {
        b = world_.createCircle(pos, spawnRadius_, type);
    } else if (shapeKind == "box") {
        b = world_.createBox(pos, spawnHalfWidth_, spawnHalfHeight_, type);
    } else if (shapeKind == "triangle") {
        b = world_.createBody(ShapeData::MakeRegularPolygon(3, spawnRadius_), pos, type);
    } else if (shapeKind == "pentagon") {
        b = world_.createBody(ShapeData::MakeRegularPolygon(5, spawnRadius_), pos, type);
    } else if (shapeKind == "hexagon") {
        b = world_.createBody(ShapeData::MakeRegularPolygon(6, spawnRadius_), pos, type);
    }
    if (b) {
        b->name = "created_" + std::to_string(b->id);
        b->matterKind = matterKind;
    }
    return b;
}

void EditorApp::createObjectInScene(const std::string& kind) {
    if (isUiElementCreateKind(kind)) {
        // Native UI element -- no script written or attached (unlike
        // createUiElement()'s FileSystem flow); it renders itself directly,
        // see drawNativeUiElement(). nextUiElementId_ is read before
        // spawnUiElementHost() consumes/increments it just for the log line
        // below matching the body's actual name.
        int id = nextUiElementId_;
        Body* host = spawnUiElementHost({0.0f, 200.0f + static_cast<float>(id)}, 0.01f, {}, kind);
        selected_ = host;
        multiSelected_.clear();
        lastInspected_ = nullptr;
        log("Created UI element '" + host->name + "' (" + kind + "), now selected. No script attached -- attach\n"
            "one later (Inspector > Script path) for custom behavior.",
            false);
        return;
    }

    // A physics shape: spawned at world (0, 0), NOT a UI Element, using
    // whatever Matter Kind/Body Type/Quantity the "..." popup currently has
    // set. A batch (quantity > 1) is spread across a small grid centered on
    // the origin rather than stacked exactly on top of itself -- same
    // reasoning as scripts/spawn_by_matter_kind.lua's own grid spawn.
    int quantity = std::max(1, kCreateQuantities[std::clamp(createQuantityIdx_, 0, kCreateQuantityCount - 1)]);
    int cols = std::min(40, std::max(1, static_cast<int>(std::ceil(std::sqrt(static_cast<float>(quantity))))));
    float extent = std::max({spawnRadius_, spawnHalfWidth_, spawnHalfHeight_});
    float spacing = extent * 2.5f;
    float startX = -static_cast<float>(cols - 1) * spacing * 0.5f;

    Body* last = nullptr;
    for (int i = 0; i < quantity; ++i) {
        int row = i / cols;
        int col = i % cols;
        Vec2 pos(startX + static_cast<float>(col) * spacing, static_cast<float>(row) * spacing);
        Body* b = spawnShapeAt(pos, kind, createBodyType_, createMatterKind_);
        if (b) last = b;
    }
    if (last) {
        selected_ = last;
        multiSelected_.clear();
        lastInspected_ = nullptr;
        log("Created " + std::to_string(quantity) + " " + kind + (quantity == 1 ? "" : "(s)") + " at (0, 0).", false);
    }
}

void EditorApp::copyFsSelection() {
    std::error_code ec;
    if (selectedFsPath_.empty() || !std::filesystem::exists(selectedFsPath_, ec)) return;
    fsClipboardPath_ = selectedFsPath_;
    log("Copied " + selectedFsPath_.filename().string(), false);
}

void EditorApp::pasteFsClipboard() {
    std::error_code existsEc;
    if (fsClipboardPath_.empty() || !std::filesystem::exists(fsClipboardPath_, existsEc)) {
        log("Nothing to paste (clipboard is empty or its source no longer exists).", true);
        fsClipboardPath_.clear();
        return;
    }

    // Paste into the selected folder, next to the selected file, or into the
    // scripts root if nothing's selected.
    std::filesystem::path targetDir = scriptsDir_;
    if (!selectedFsPath_.empty()) {
        std::error_code isDirEc;
        targetDir =
            std::filesystem::is_directory(selectedFsPath_, isDirEc) ? selectedFsPath_ : selectedFsPath_.parent_path();
    }

    std::string stem = fsClipboardPath_.stem().string();
    std::string ext = fsClipboardPath_.extension().string();
    std::filesystem::path dest = targetDir / fsClipboardPath_.filename();
    std::error_code destExistsEc;
    for (int suffix = 1; std::filesystem::exists(dest, destExistsEc); ++suffix) {
        std::string candidate = stem + "_copy" + (suffix > 1 ? std::to_string(suffix) : "") + ext;
        dest = targetDir / candidate;
    }

    std::error_code copyEc;
    std::filesystem::copy(fsClipboardPath_, dest,
                           std::filesystem::copy_options::recursive | std::filesystem::copy_options::copy_symlinks,
                           copyEc);
    if (copyEc) {
        log("Paste failed: " + copyEc.message(), true);
    } else {
        log("Pasted as " + dest.filename().string(), false);
        selectedFsPath_ = dest;
        refreshAvailableScripts();
    }
}

void EditorApp::deleteFsSelection() {
    if (selectedFsPath_.empty()) return;
    std::error_code isDirEc;
    bool isDir = std::filesystem::is_directory(selectedFsPath_, isDirEc);
    std::string name = selectedFsPath_.filename().string();

    std::error_code delEc;
    if (isDir) {
        std::filesystem::remove_all(selectedFsPath_, delEc);
    } else {
        std::filesystem::remove(selectedFsPath_, delEc);
    }

    if (delEc) {
        log("Delete failed: " + delEc.message(), true);
    } else {
        log("Deleted " + name, false);
    }
    if (fsClipboardPath_ == selectedFsPath_) fsClipboardPath_.clear();
    selectedFsPath_.clear();
    refreshAvailableScripts();
}

bool EditorApp::importImageFile(const std::filesystem::path& src, const std::filesystem::path& destDir) {
    std::error_code existsEc;
    if (!std::filesystem::is_regular_file(src, existsEc)) {
        log("Not a file: " + src.string(), true);
        return false;
    }

    std::string ext = src.extension().string();
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    bool isImage = std::find(kImageExtensions.begin(), kImageExtensions.end(), ext) != kImageExtensions.end();
    if (!isImage) {
        log("Not a recognized image type: " + src.filename().string(), true);
        return false;
    }

    // Same "don't clobber an existing file" naming rule pasteFsClipboard()
    // uses, just with a plain numeric suffix instead of "_copy" -- this is a
    // new file coming in from outside the project, not a copy of something
    // already in it.
    std::string stem = src.stem().string();
    std::filesystem::path dest = destDir / src.filename();
    std::error_code destExistsEc;
    for (int suffix = 1; std::filesystem::exists(dest, destExistsEc); ++suffix) {
        dest = destDir / (stem + "_" + std::to_string(suffix) + ext);
    }

    std::error_code copyEc;
    std::filesystem::copy_file(src, dest, copyEc);
    if (copyEc) {
        log("Could not import " + src.filename().string() + ": " + copyEc.message(), true);
        return false;
    }
    log("Imported " + dest.filename().string(), false);
    refreshAvailableScripts();
    return true;
}

void EditorApp::uploadTextureVia(const std::filesystem::path& destDir) {
    std::vector<std::string> patterns;
    for (const auto& ext : kImageExtensions) patterns.push_back("*" + ext);
    std::vector<const char*> patternPtrs;
    for (const auto& p : patterns) patternPtrs.push_back(p.c_str());

    char* picked = tinyfd_openFileDialog("Upload Texture", "", static_cast<int>(patternPtrs.size()),
                                          patternPtrs.data(), "Image files", 0);
    if (picked) importImageFile(picked, destDir);
}

void EditorApp::importDroppedFiles(const std::vector<std::filesystem::path>& paths) {
    for (const auto& src : paths) importImageFile(src, scriptsDir_);
}

Vec2 EditorApp::viewportPixelToWorld(sf::Vector2i windowPixel) const {
    sf::Vector2f local(static_cast<float>(windowPixel.x) - viewportScreenPos_.x,
                        static_cast<float>(windowPixel.y) - viewportScreenPos_.y);
    return camera_.screenToWorld(local, viewportSize_);
}

void EditorApp::handleEvent(const sf::Event& event) {
    // NOTE: io.WantCaptureMouse is true whenever the mouse merely hovers
    // *any* ImGui window -- including the Viewport panel itself, since it's
    // just a window containing an Image. Gating on it here would mean the
    // viewport can never receive clicks. viewportHovered_ (set from
    // IsItemHovered() on the viewport image each frame) is the correct
    // signal for "should this click go to the physics scene".
    if (event.type == sf::Event::MouseWheelScrolled) {
        sf::Vector2i pixel(event.mouseWheelScroll.x, event.mouseWheelScroll.y);
        if (viewportHovered_) {
            float factor = (event.mouseWheelScroll.delta > 0) ? 1.1f : (1.0f / 1.1f);
            sf::Vector2f local(static_cast<float>(pixel.x) - viewportScreenPos_.x,
                                static_cast<float>(pixel.y) - viewportScreenPos_.y);
            camera_.zoomAt(local, viewportSize_, factor);
        }
    } else if (event.type == sf::Event::MouseButtonPressed) {
        sf::Vector2i pixel(event.mouseButton.x, event.mouseButton.y);

        if (event.mouseButton.button == sf::Mouse::Middle || event.mouseButton.button == sf::Mouse::Right) {
            if (viewportHovered_) {
                panning_ = true;
                lastMousePixel_ = pixel;
            }
        } else if (event.mouseButton.button == sf::Mouse::Left && viewportHovered_) {
            Vec2 worldPos = viewportPixelToWorld(pixel);
            if (spawnTool_ == SpawnTool::Spawn) {
                spawnAt(worldPos);
            } else if (spawnTool_ == SpawnTool::Resize || spawnTool_ == SpawnTool::Rotate) {
                Body* hit = pickBodyAt(worldPos);
                selected_ = hit;
                multiSelected_.clear();
                toolDragBody_ = hit;
            } else if (spawnTool_ == SpawnTool::BoxSelect) {
                boxSelecting_ = true;
                boxSelectStart_ = worldPos;
                boxSelectCurrent_ = worldPos;
            } else {
                Body* hit = pickBodyAt(worldPos);
                bool hitIsInGroup =
                    hit && std::find(multiSelected_.begin(), multiSelected_.end(), hit) != multiSelected_.end();
                if (!hitIsInGroup) multiSelected_.clear();
                selected_ = hit;
                if (hit) {
                    bool canDrag = running_ ? (hit->type == BodyType::Dynamic) : true;
                    if (canDrag) {
                        dragged_ = hit;
                        dragLocalAnchor_ = rotate(worldPos - hit->position, -hit->rotation);
                    }
                }
            }
        }
    } else if (event.type == sf::Event::MouseButtonReleased) {
        if (event.mouseButton.button == sf::Mouse::Left) {
            dragged_ = nullptr;
            toolDragBody_ = nullptr;
            if (boxSelecting_) {
                boxSelecting_ = false;
                finishBoxSelect();
            }
        }
        if (event.mouseButton.button == sf::Mouse::Middle || event.mouseButton.button == sf::Mouse::Right)
            panning_ = false;
    } else if (event.type == sf::Event::MouseMoved) {
        sf::Vector2i pixel(event.mouseMove.x, event.mouseMove.y);
        mouseWorld_ = viewportPixelToWorld(pixel);
        if (panning_) {
            int dx = pixel.x - lastMousePixel_.x;
            int dy = pixel.y - lastMousePixel_.y;
            camera_.center.x -= static_cast<float>(dx) / camera_.pixelsPerMeter;
            camera_.center.y += static_cast<float>(dy) / camera_.pixelsPerMeter;
            lastMousePixel_ = pixel;
        } else if (toolDragBody_ && spawnTool_ == SpawnTool::Resize) {
            Vec2 local = rotate(mouseWorld_ - toolDragBody_->position, -toolDragBody_->rotation);
            if (toolDragBody_->shape.type == ShapeType::Circle) {
                toolDragBody_->shape.radius = std::clamp((mouseWorld_ - toolDragBody_->position).length(), 0.05f, 50.0f);
            } else {
                float hw = std::clamp(std::fabs(local.x), 0.05f, 50.0f);
                float hh = std::clamp(std::fabs(local.y), 0.05f, 50.0f);
                toolDragBody_->shape = ShapeData::MakeBox(hw, hh);
            }
            toolDragBody_->computeMass();
            toolDragBody_->wake();
        } else if (toolDragBody_ && spawnTool_ == SpawnTool::Rotate) {
            Vec2 dir = mouseWorld_ - toolDragBody_->position;
            if (dir.length() > 0.001f) {
                toolDragBody_->rotation = std::atan2(dir.y, dir.x);
                toolDragBody_->wake();
            }
        } else if (boxSelecting_) {
            boxSelectCurrent_ = mouseWorld_;
        }
    }
}

void EditorApp::update(float realDt) {
    applyDragInteraction();

    // Continuously keep the restore point current while in genuine edit mode
    // -- but NOT merely because we're paused mid-play-session, so anything
    // spawned/changed since Play was pressed (including via a scripted UI
    // button) doesn't leak into the checkpoint just because you paused
    // before hitting Reset. keepSpawnedObjectsOnReset_ (Settings) opts back
    // into the old always-continuous behavior if you'd rather Reset keep
    // whatever was spawned during play.
    if (!running_) {
        if (!everPlayedSinceCheckpoint_ || keepSpawnedObjectsOnReset_) captureSnapshot();

        sceneAutosaveTimer_ += realDt;
        if (sceneAutosaveTimer_ >= kSceneAutosaveIntervalSeconds) {
            sceneAutosaveTimer_ = 0.0f;
            saveSceneToDisk();
        }
        return;
    }

    accumulator_ += realDt * timeScale_;
    const int maxStepsPerFrame = 8;
    int steps = 0;
    while (accumulator_ >= fixedDt_ && steps < maxStepsPerFrame) {
        stepSimulation(fixedDt_);
        accumulator_ -= fixedDt_;
        ++steps;
    }
    if (steps == maxStepsPerFrame) accumulator_ = 0.0f;
}

void EditorApp::renderViewport() {
    if (viewportSize_.x == 0 || viewportSize_.y == 0) return;

    if (viewportTexture_.getSize() != viewportSize_) {
        if (!viewportTexture_.create(viewportSize_.x, viewportSize_.y)) return;
    }

    renderer_.drawBackground(viewportTexture_, camera_, backgroundColorR_, backgroundColorG_, backgroundColorB_,
                              backgroundTexturePath_, backgroundTiled_);
    renderer_.drawGrid(viewportTexture_, camera_);

    // Soft body spring mesh, drawn under the particles themselves so it
    // reads as connective tissue rather than a tangle of lines on top.
    if (!softBodies_.empty()) {
        sf::Vector2u size = viewportTexture_.getSize();
        sf::VertexArray lines(sf::Lines);
        for (auto& sb : softBodies_) {
            for (auto& s : sb.springs) {
                if (s.a < 0 || s.b < 0 || static_cast<size_t>(s.a) >= sb.particles.size() ||
                    static_cast<size_t>(s.b) >= sb.particles.size()) {
                    continue;
                }
                sf::Vector2f pa = camera_.worldToScreen(sb.particles[static_cast<size_t>(s.a)]->position, size);
                sf::Vector2f pb = camera_.worldToScreen(sb.particles[static_cast<size_t>(s.b)]->position, size);
                lines.append(sf::Vertex(pa, sf::Color(120, 200, 150, 190)));
                lines.append(sf::Vertex(pb, sf::Color(120, 200, 150, 190)));
            }
        }
        viewportTexture_.draw(lines);
    }

    renderer_.drawWorld(viewportTexture_, world_, camera_, selected_, multiSelected_, showCircleDirectionLines_);

    // Standalone spring connections, drawn on top (unlike the soft-body
    // mesh above) since these tie together full-size, possibly widely
    // separated ordinary objects -- the connection itself is the point,
    // so it should stay visible rather than read as background texture.
    if (!springJoints_.empty()) {
        sf::Vector2u size = viewportTexture_.getSize();
        sf::VertexArray lines(sf::Lines);
        for (auto& j : springJoints_) {
            if (!j.a || !j.b) continue;
            sf::Vector2f pa = camera_.worldToScreen(j.a->position, size);
            sf::Vector2f pb = camera_.worldToScreen(j.b->position, size);
            lines.append(sf::Vertex(pa, sf::Color(230, 170, 60, 220)));
            lines.append(sf::Vertex(pb, sf::Color(230, 170, 60, 220)));
        }
        viewportTexture_.draw(lines);
    }

    if (boxSelecting_) {
        sf::Vector2u size = viewportTexture_.getSize();
        sf::Vector2f a = camera_.worldToScreen({boxSelectStart_.x, boxSelectStart_.y}, size);
        sf::Vector2f b = camera_.worldToScreen({boxSelectCurrent_.x, boxSelectCurrent_.y}, size);
        sf::RectangleShape rect(sf::Vector2f(b.x - a.x, b.y - a.y));
        rect.setPosition(a);
        rect.setFillColor(sf::Color(80, 140, 255, 40));
        rect.setOutlineColor(sf::Color(140, 190, 255, 220));
        rect.setOutlineThickness(1.5f);
        viewportTexture_.draw(rect);
    }

    viewportTexture_.display();
}

void EditorApp::drawUI() {
    ImGuiID dockspaceId =
        ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);

    if (!dockLayoutBuilt_) {
        dockLayoutBuilt_ = true;
        buildDockLayout(dockspaceId);
    }

    drawToolbar();
    drawSettings();
    drawViewportPanel();
    drawUiOverlay();
    drawCodeEditorPanel();
    drawHierarchy();
    drawFileSystemPanel();
    drawInspector();
    drawScriptUiPanel();
    drawConsole();
}

void EditorApp::buildDockLayout(unsigned int dockspaceIdRaw) {
    ImGuiID dockspaceId = static_cast<ImGuiID>(dockspaceIdRaw);

    ImGui::DockBuilderRemoveNode(dockspaceId);
    ImGui::DockBuilderAddNode(dockspaceId, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspaceId, ImGui::GetMainViewport()->WorkSize);

    ImGuiID dockMain = dockspaceId;
    ImGuiID dockTop = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Up, 0.17f, nullptr, &dockMain);
    ImGuiID dockRight = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Right, 0.22f, nullptr, &dockMain);
    ImGuiID dockLeft = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Left, 0.18f, nullptr, &dockMain);
    ImGuiID dockBottom = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Down, 0.25f, nullptr, &dockMain);
    // Godot-like left column: Scene/Hierarchy on top, FileSystem below it.
    ImGuiID dockLeftBottom = ImGui::DockBuilderSplitNode(dockLeft, ImGuiDir_Down, 0.45f, nullptr, &dockLeft);

    ImGui::DockBuilderDockWindow("Toolbar", dockTop);
    ImGui::DockBuilderDockWindow("Viewport", dockMain);
    // Pre-registered into the same dock node as Viewport (and never Begin()'d
    // until the user opens it, see drawCodeEditorPanel/showCodeEditor_) --
    // that makes it a tab alongside Viewport, so only one of the two is ever
    // visible at a time, exactly like Godot's 2D/Script view tabs.
    ImGui::DockBuilderDockWindow("Code Editor###CodeEditor", dockMain);
    ImGui::DockBuilderDockWindow("Hierarchy", dockLeft);
    ImGui::DockBuilderDockWindow("FileSystem", dockLeftBottom);
    ImGui::DockBuilderDockWindow("Inspector", dockRight);
    ImGui::DockBuilderDockWindow("Script UI", dockRight);
    ImGui::DockBuilderDockWindow("Console", dockBottom);

    ImGui::DockBuilderFinish(dockspaceId);
}

void EditorApp::drawToolbar() {
    ImGui::Begin("Toolbar");

    // --- Row 1: playback controls + live status -------------------------
    if (ImGui::Button("Projects")) returnToProjectManager_ = true;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Back to the project list. This project is saved on disk already.");
    ImGui::SameLine();
    ImGui::TextUnformatted("|");
    ImGui::SameLine();

    if (!running_) {
        if (ImGui::Button("Play")) {
            running_ = true;
            everPlayedSinceCheckpoint_ = true;
            log("Playing.", false);
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Start simulating. Your layout is remembered so Stop can restore it.");
    } else {
        if (ImGui::Button("Pause")) running_ = false;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Freeze the simulation without losing progress. Reset Scene still discards anything\n"
                "spawned/changed since Play, even while paused, unless Settings > Keep spawned objects\n"
                "after Reset is enabled.");
        ImGui::SameLine();
        if (ImGui::Button("Stop")) {
            running_ = false;
            restoreSnapshot();
            everPlayedSinceCheckpoint_ = false;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Stop and snap back to how the scene looked before you pressed Play.");
    }
    ImGui::SameLine();
    if (ImGui::Button("Step")) {
        running_ = false;
        everPlayedSinceCheckpoint_ = true;
        stepSimulation(fixedDt_);
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Advance exactly one physics step.");
    ImGui::SameLine();
    if (ImGui::Button("Reset Scene")) {
        running_ = false;
        if (hasSnapshot_) {
            restoreSnapshot();
        } else {
            resetScene();
        }
        everPlayedSinceCheckpoint_ = false;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "Reset to the scene as it looked the last time the project was neither playing nor "
            "paused -- discards anything spawned or changed since Play, whether you're still\n"
            "playing or have since paused.");

    ImGui::SameLine();
    ImGui::Dummy(ImVec2(16.0f, 0.0f));
    ImGui::SameLine();
    if (ImGui::Button("Settings")) showSettings_ = true;

    ImGui::SameLine();
    ImGui::Dummy(ImVec2(16.0f, 0.0f));
    ImGui::SameLine();
    ImGui::TextColored(running_ ? ImVec4(0.4f, 1.0f, 0.5f, 1.0f) : ImVec4(0.95f, 0.8f, 0.3f, 1.0f),
                        running_ ? "Playing" : "Editing");
    ImGui::SameLine();
    ImGui::Text("|  Bodies: %d", static_cast<int>(world_.bodies().size()));

    // --- Row 2: simulation parameters ------------------------------------
    ImGui::SetNextItemWidth(140.0f);
    ImGui::SliderFloat("Time Scale", &timeScale_, 0.1f, 3.0f);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(60.0f);
    if (ImGui::InputFloat("##timeScaleInput", &timeScale_, 0.0f, 0.0f, "%.2f")) {
        timeScale_ = std::clamp(timeScale_, 0.0f, 10.0f);
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Type an exact time scale value (0 pauses, 1 = normal speed).");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(160.0f);
    ImGui::SliderFloat("Gravity Y", &world_.gravity.y, -30.0f, 10.0f);
    // --- Row 3: tools ------------------------------------------------------
    ImGui::TextUnformatted("Tool:");
    ImGui::SameLine();
    if (ImGui::RadioButton("Select / Move", spawnTool_ == SpawnTool::None)) spawnTool_ = SpawnTool::None;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "Click a body to select it, drag to move it.\n"
            "While editing this teleports the body; while playing it pulls with a spring.\n"
            "If multiple bodies are selected (Box Select), dragging any one moves the whole group.");
    ImGui::SameLine();
    if (ImGui::RadioButton("Spawn", spawnTool_ == SpawnTool::Spawn)) spawnTool_ = SpawnTool::Spawn;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Click in the viewport to place a new body there. Pick which shape below.");
    ImGui::SameLine();
    if (ImGui::RadioButton("Resize", spawnTool_ == SpawnTool::Resize)) spawnTool_ = SpawnTool::Resize;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Click a body, then drag outward/inward to resize it (radius or half-extents).\n"
                           "Mass is recomputed automatically.");
    ImGui::SameLine();
    if (ImGui::RadioButton("Rotate", spawnTool_ == SpawnTool::Rotate)) spawnTool_ = SpawnTool::Rotate;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Click a body, then drag around it -- it rotates to face the mouse.");
    ImGui::SameLine();
    if (ImGui::RadioButton("Box Select", spawnTool_ == SpawnTool::BoxSelect)) spawnTool_ = SpawnTool::BoxSelect;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Drag a rectangle in the viewport to select every body inside it at once.");

    if (spawnTool_ == SpawnTool::Spawn) {
        ImGui::SameLine();
        ImGui::Dummy(ImVec2(12.0f, 0.0f));
        ImGui::SameLine();
        static const char* kShapeNames[] = {"Circle", "Box", "Triangle", "Pentagon", "Hexagon"};
        int shapeIdx = static_cast<int>(spawnShapeKind_);
        ImGui::SetNextItemWidth(150.0f);
        if (ImGui::Combo("Shape", &shapeIdx, kShapeNames, IM_ARRAYSIZE(kShapeNames))) {
            spawnShapeKind_ = static_cast<SpawnShapeKind>(shapeIdx);
        }

        if (spawnShapeKind_ == SpawnShapeKind::Box) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(90.0f);
            ImGui::SliderFloat("Half W", &spawnHalfWidth_, 0.1f, 2.0f);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(90.0f);
            ImGui::SliderFloat("Half H", &spawnHalfHeight_, 0.1f, 2.0f);
        } else {
            // Circle, Triangle, Pentagon, Hexagon all just need a circumradius.
            ImGui::SameLine();
            ImGui::SetNextItemWidth(100.0f);
            ImGui::SliderFloat("Size", &spawnRadius_, 0.1f, 2.0f);
        }
    }

    if (multiSelected_.size() > 1) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "|  %d bodies selected",
                            static_cast<int>(multiSelected_.size()));
    }

    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Left click: use the selected tool.\n"
            "Right or middle drag: pan the camera.\n"
            "Mouse wheel: zoom.\n"
            "Ctrl+C / Ctrl+V: copy/paste the selected body (or Box Select group).");
    }

    ImGui::End();
}

void EditorApp::drawSettings() {
    if (!showSettings_) return;

    ImGui::SetNextWindowSize(ImVec2(420.0f, 0.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Settings", &showSettings_, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }

    // Resynced only the frame this window (re)appears -- not every frame,
    // which would stomp whatever the user is mid-typing into the field.
    if (ImGui::IsWindowAppearing()) {
        std::snprintf(backgroundTexturePathBuffer_, sizeof(backgroundTexturePathBuffer_), "%s",
                      backgroundTexturePath_.c_str());
    }

    ImGui::TextUnformatted("Spawning");
    ImGui::Separator();
    {
        const char* matterKindNames[] = {"Rigidbody", "Matter", "OptiMatter"};
        int idx = static_cast<int>(defaultSpawnMatterKind_);
        ImGui::SetNextItemWidth(200.0f);
        ImGui::Combo("Default Matter Kind", &idx, matterKindNames, 3);
        defaultSpawnMatterKind_ = static_cast<MatterKind>(idx);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "The Matter Kind every new body gets from the Spawn tool (viewport click). Doesn't\n"
            "affect bodies that already exist -- change an individual body's kind from its own\n"
            "Inspector \"Matter Kind\" dropdown instead.\n\n"
            "Rigidbody: the ordinary baseline, unaffected by any of this.\n"
            "Matter: MORE accurate than Rigidbody -- stays awake longer, corrects overlaps more\n"
            "gently, and forces extra substeps sooner -- at real extra cost.\n"
            "OptiMatter (default): cheaper than Rigidbody -- sleeps sooner, corrects overlaps in\n"
            "bigger snappier steps, skips forced extra substeps, and is exempt from CCD even\n"
            "when it's on. Good for background/decorative bodies or large crowds.");

    ImGui::TextUnformatted("Reset Scene");
    ImGui::Separator();
    ImGui::Checkbox("Keep objects spawned during Play after Reset", &keepSpawnedObjectsOnReset_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "Off (default): Reset Scene discards anything spawned or changed since Play was\n"
            "pressed -- including bodies a scripted UI button spawned mid-simulation -- even if\n"
            "you've since paused. On: Reset Scene instead keeps whatever the scene currently\n"
            "looks like, the same as it behaved before this option existed.");

    ImGui::Separator();
    ImGui::TextUnformatted("Rendering");
    ImGui::Separator();
    ImGui::Checkbox("Show circle direction line", &showCircleDirectionLines_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The small radial line marking a circle's current rotation -- otherwise a circle\n"
                           "looks identical at every angle. Off hides it for scenes with many circles where\n"
                           "it just adds visual clutter.");

    ImGui::Separator();
    ImGui::TextUnformatted("Background");
    ImGui::Separator();
    float bgCol[3] = {backgroundColorR_ / 255.0f, backgroundColorG_ / 255.0f, backgroundColorB_ / 255.0f};
    if (ImGui::ColorEdit3("Color", bgCol)) {
        backgroundColorR_ = static_cast<uint8_t>(std::clamp(bgCol[0], 0.0f, 1.0f) * 255.0f);
        backgroundColorG_ = static_cast<uint8_t>(std::clamp(bgCol[1], 0.0f, 1.0f) * 255.0f);
        backgroundColorB_ = static_cast<uint8_t>(std::clamp(bgCol[2], 0.0f, 1.0f) * 255.0f);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Viewport backdrop color -- always shown; the texture below (if any) draws over\n"
                           "it, so this still matters for images with transparency.");

    ImGui::TextWrapped("Texture: %s", backgroundTexturePath_.empty() ? "(none)" : backgroundTexturePath_.c_str());
    if (ImGui::InputText("Background Texture path", backgroundTexturePathBuffer_, sizeof(backgroundTexturePathBuffer_))) {
        backgroundTexturePath_ = backgroundTexturePathBuffer_;
    }
    if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("IMAGE_PATH")) {
            std::snprintf(backgroundTexturePathBuffer_, sizeof(backgroundTexturePathBuffer_), "%s",
                          static_cast<const char*>(payload->Data));
            backgroundTexturePath_ = backgroundTexturePathBuffer_;
            log("Set " + std::filesystem::path(backgroundTexturePath_).filename().string() + " as background.",
                false);
        }
        ImGui::EndDragDropTarget();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Drag an image here from the FileSystem panel, or type/paste a path.");
    if (ImGui::SmallButton("Clear Background Texture")) {
        backgroundTexturePathBuffer_[0] = '\0';
        backgroundTexturePath_.clear();
    }

    ImGui::Checkbox("Tiled", &backgroundTiled_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("On: the image repeats every couple world units, panning/zooming with the scene.\n"
                           "Off: the image is stretched to exactly cover the current view instead.");

    ImGui::End();
}

void EditorApp::drawViewportPanel() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    // Begin() returns false when this tab isn't the active one in its dock
    // node (it shares one with Code Editor) -- previously ignored, which
    // let viewportScreenPos_/viewportHovered_ go stale while Code Editor
    // was showing instead, and drawUiOverlay() kept drawing the on_gui HUD
    // at that stale position on top of it regardless.
    viewportTabVisible_ =
        ImGui::Begin("Viewport", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();

    if (viewportTabVisible_) {
        ImVec2 avail = ImGui::GetContentRegionAvail();
        viewportSize_ = sf::Vector2u(static_cast<unsigned int>(std::max(1.0f, avail.x)),
                                      static_cast<unsigned int>(std::max(1.0f, avail.y)));

        ImVec2 cursor = ImGui::GetCursorScreenPos();
        viewportScreenPos_ = sf::Vector2f(cursor.x, cursor.y);

        if (viewportTexture_.getSize().x > 0 && viewportTexture_.getSize().y > 0) {
            ImGui::Image(viewportTexture_, avail);
        }
        viewportHovered_ = ImGui::IsItemHovered();

        // Copy/paste the selected body (or Box Select group). Gated on
        // focus OR hover, same reasoning as the FileSystem panel's
        // shortcuts: hover alone is enough to unambiguously mean "act on
        // the viewport", and is more forgiving than focus alone.
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows) || viewportHovered_) {
            ImGuiIO& io = ImGui::GetIO();
            if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_C, false)) copyViewportSelection();
            if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_V, false)) pasteViewportClipboard();
        }
    } else {
        viewportHovered_ = false;
    }

    ImGui::End();
}

void EditorApp::drawHierarchy() {
    ImGui::Begin("Hierarchy");

    // Create: picks a Kind (physics shape or UI kind) and makes one --
    // physics shapes spawn at world (0, 0) as an ordinary body (no viewport
    // click needed, unlike the Spawn tool); UI kinds create a UI Element
    // with no script attached. The "..." popup holds Matter Kind/Body
    // Type/Quantity, which only apply to physics shapes -- kept out of the
    // main row since there isn't room for all three there.
    ImGui::SetNextItemWidth(110.0f);
    ImGui::Combo("Kind", &createKindIdx_, kCreateKinds, kCreateKindCount);
    sameLineIfFits(buttonWidthEstimate("Create") + buttonWidthEstimate("...") + 8.0f);
    if (ImGui::Button("Create")) createObjectInScene(kCreateKindIds[createKindIdx_]);
    if (ImGui::IsItemHovered()) {
        if (isUiElementCreateKind(kCreateKindIds[createKindIdx_])) {
            ImGui::SetTooltip("Creates a UI Element with no script attached -- it renders itself directly;\n"
                               "attach a script later only if you need custom behavior.");
        } else {
            ImGui::SetTooltip("Spawns at world (0, 0) as an ordinary body, NOT a UI Element. Use the \"...\"\n"
                               "button to set Matter Kind, Body Type, or spawn a batch of them at once.");
        }
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("...")) showCreateOptionsPopup_ = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Matter Kind / Body Type / Quantity for physics shapes (ignored for UI kinds).");
    drawCreateOptionsPopup();
    ImGui::Separator();

    for (auto& bodyPtr : world_.bodies()) {
        Body* b = bodyPtr.get();
        std::string label = b->name.empty() ? ("Body " + std::to_string(b->id)) : b->name;
        if (b->isUiElement) label += " (UI)"; // distinguishes it from an ordinary body at a glance
        bool isSelected = (b == selected_) && multiSelected_.size() <= 1;
        if (ImGui::Selectable(label.c_str(), isSelected)) {
            selected_ = b;
            multiSelected_.clear();
        }
    }
    ImGui::End();
}

void EditorApp::drawCreateOptionsPopup() {
    if (showCreateOptionsPopup_) {
        ImGui::OpenPopup("Create Options##hierarchy");
        showCreateOptionsPopup_ = false;
    }
    if (!ImGui::BeginPopupModal("Create Options##hierarchy", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;

    if (isUiElementCreateKind(kCreateKindIds[createKindIdx_])) {
        ImGui::TextDisabled("These only apply to physics shapes -- the current Kind is a UI element,\n"
                             "which is never batched or given a Matter/Body Type.");
    } else {
        const char* matterKindNames[] = {"Rigidbody", "Matter", "OptiMatter"};
        int matterKindIdx = static_cast<int>(createMatterKind_);
        ImGui::SetNextItemWidth(150.0f);
        ImGui::Combo("Matter Kind", &matterKindIdx, matterKindNames, 3);
        createMatterKind_ = static_cast<MatterKind>(matterKindIdx);

        const char* bodyTypeNames[] = {"Static", "Kinematic", "Dynamic"};
        int bodyTypeIdx = static_cast<int>(createBodyType_);
        ImGui::SetNextItemWidth(150.0f);
        ImGui::Combo("Body Type", &bodyTypeIdx, bodyTypeNames, 3);
        createBodyType_ = static_cast<BodyType>(bodyTypeIdx);

        ImGui::SetNextItemWidth(150.0f);
        ImGui::Combo("Quantity", &createQuantityIdx_, kCreateQuantityLabels, kCreateQuantityCount);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("How many Create spawns at once, spread in a small grid around (0, 0).");
    }

    ImGui::Separator();
    if (ImGui::Button("Close")) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

void EditorApp::drawInspector() {
    ImGui::Begin("Inspector");

    if (multiSelected_.size() > 1) {
        ImGui::Text("%d bodies selected (Box Select)", static_cast<int>(multiSelected_.size()));
        ImGui::TextDisabled("Drag any one of them in the viewport to move the whole group together.");
        ImGui::Separator();
        if (ImGui::Button("Delete All Selected")) {
            for (Body* b : multiSelected_) {
                if (dragged_ == b) dragged_ = nullptr;
                if (toolDragBody_ == b) toolDragBody_ = nullptr;
                world_.removeBody(b); // world_.onBodyRemoved detaches any script for us
            }
            multiSelected_.clear();
            selected_ = nullptr;
            lastInspected_ = nullptr;
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear Selection")) {
            multiSelected_.clear();
            selected_ = nullptr;
        }
        ImGui::Separator();
        ImGui::SetNextItemWidth(80.0f);
        ImGui::SliderFloat("Stiffness##group", &springJointStiffness_, 5.0f, 200.0f);
        sameLineIfFits(80.0f);
        ImGui::SetNextItemWidth(80.0f);
        ImGui::SliderFloat("Damping##group", &springJointDamping_, 0.0f, 10.0f);
        sameLineIfFits(buttonWidthEstimate("Connect Selected with Springs"));
        if (ImGui::Button("Connect Selected with Springs")) {
            connectGroupWithSprings(multiSelected_, springJointStiffness_, springJointDamping_);
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Ties every selected body to its neighbors (in selection order) with springs --\n"
                "each one stays itself (own shape, own type), just wired together, the same way\n"
                "a Cloth's particles are.");
        ImGui::End();
        return;
    }

    if (!selected_) {
        ImGui::TextUnformatted("No body selected.");
        ImGui::End();
        return;
    }

    Body& b = *selected_;

    if (lastInspected_ != selected_) {
        lastInspected_ = selected_;
        std::snprintf(scriptPathBuffer_, sizeof(scriptPathBuffer_), "%s", b.scriptPath.c_str());
        std::snprintf(uiTextBuffer_, sizeof(uiTextBuffer_), "%s", b.uiText.c_str());
        std::snprintf(texturePathBuffer_, sizeof(texturePathBuffer_), "%s", b.texturePath.c_str());
    }

    ImGui::Text("%s (id %d)", b.name.c_str(), b.id);

    float pos[2] = {b.position.x, b.position.y};
    responsiveField("Position", [&](const char* lbl) {
        if (ImGui::DragFloat2(lbl, pos, 0.05f)) {
            b.position = Vec2(pos[0], pos[1]);
            b.wake();
        }
    });

    float rotDeg = b.rotation * 180.0f / kPi;
    responsiveField("Rotation (deg)", [&](const char* lbl) {
        if (ImGui::DragFloat(lbl, &rotDeg, 1.0f)) {
            b.rotation = rotDeg * kPi / 180.0f;
            b.wake();
        }
    });

    // Previously only adjustable by dragging with the viewport's Resize
    // tool -- these mirror exactly what that tool does (circle: radius;
    // anything else: rebuilt as a box from half-extents, same as Resize),
    // just as direct numeric Inspector fields.
    if (b.shape.type == ShapeType::Circle) {
        responsiveField("Radius", [&](const char* lbl) {
            if (ImGui::DragFloat(lbl, &b.shape.radius, 0.02f, 0.05f, 50.0f)) {
                b.computeMass();
                b.wake();
            }
        });
    } else {
        Vec2 half = b.shape.localHalfExtents();
        float halfExtents[2] = {half.x, half.y};
        responsiveField("Half Width / Height", [&](const char* lbl) {
            if (ImGui::DragFloat2(lbl, halfExtents, 0.02f, 0.05f, 50.0f)) {
                b.shape = ShapeData::MakeBox(halfExtents[0], halfExtents[1]);
                b.computeMass();
                b.wake();
            }
        });
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Rebuilds this shape as a box with these half-extents -- same as dragging with\n"
                               "the viewport's Resize tool (which does the same thing for any non-circle shape).");
    }

    if (b.isUiElement) {
        if (b.uiKind == "panel") {
            // Panel has no text at all -- width/height (repurposing
            // uiMin/uiMax, same idea as Slider's value range) instead.
            float size[2] = {b.uiMin, b.uiMax};
            responsiveField("Panel Size (W / H)", [&](const char* lbl) {
                if (ImGui::DragFloat2(lbl, size, 1.0f, 20.0f, 4000.0f)) {
                    b.uiMin = size[0];
                    b.uiMax = size[1];
                }
            });
        } else {
            responsiveField(b.uiKind == "input_field" ? "Field Content" : "UI Text", [&](const char* lbl) {
                if (ImGui::InputText(lbl, uiTextBuffer_, sizeof(uiTextBuffer_))) {
                    b.uiText = uiTextBuffer_;
                }
            });
            if (ImGui::IsItemHovered()) {
                if (b.uiKind == "input_field") {
                    ImGui::SetTooltip("This field's own live text content -- editing it here or typing into the\n"
                                       "live widget both land in the same body.ui_text.");
                } else {
                    ImGui::SetTooltip(
                        "The label this UI element shows (or, if scripted, reads via body.ui_text) --\n"
                        "see the attached script below (Script path) if it needs more than one line.");
                }
            }
        }

        if (b.uiKind != "text" && b.uiKind != "input_field" && b.uiKind != "panel") {
            ImGui::Checkbox("Hide Text", &b.uiHideText);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Drops the label above, leaving just the widget -- most useful once\n"
                                   "Appearance > Texture gives this element an icon that speaks for itself.\n"
                                   "Not offered for \"text\"/\"input field\"/\"panel\" elements.");
        }

        // Checkbox/Slider also read/write body.ui_value (and, for Slider,
        // body.ui_min/ui_max) every frame -- see uiElementTemplateScript()/
        // drawNativeUiElement() -- so editing these fields changes what the
        // live widget shows next frame, and toggling/dragging the widget
        // itself updates these back.
        if (b.uiKind == "checkbox") {
            bool checked = b.uiValue > 0.5f;
            responsiveField("UI Value (checked)",
                             [&](const char* lbl) {
                                 if (ImGui::Checkbox(lbl, &checked)) b.uiValue = checked ? 1.0f : 0.0f;
                             });
        } else if (b.uiKind == "slider") {
            responsiveField("UI Value", [&](const char* lbl) { ImGui::DragFloat(lbl, &b.uiValue, 0.02f); });
            float range[2] = {b.uiMin, b.uiMax};
            responsiveField("UI Min / Max", [&](const char* lbl) {
                if (ImGui::DragFloat2(lbl, range, 0.02f)) {
                    b.uiMin = range[0];
                    b.uiMax = range[1];
                }
            });
        }
    }

    float vel[2] = {b.velocity.x, b.velocity.y};
    responsiveField("Velocity", [&](const char* lbl) {
        if (ImGui::DragFloat2(lbl, vel, 0.05f)) {
            b.velocity = Vec2(vel[0], vel[1]);
            b.wake();
        }
    });

    responsiveField("Angular Velocity",
                     [&](const char* lbl) { if (ImGui::DragFloat(lbl, &b.angularVelocity, 0.05f)) b.wake(); });

    ImGui::Separator();
    ImGui::TextUnformatted("Material");
    responsiveField("Bounciness (Restitution)",
                     [&](const char* lbl) { ImGui::SliderFloat(lbl, &b.restitution, 0.0f, 15.0f); });
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("0 = no bounce, 1 = perfectly elastic, >1 = gains energy each bounce.");
    responsiveField("Friction", [&](const char* lbl) { ImGui::SliderFloat(lbl, &b.friction, 0.0f, 20.0f); });
    responsiveField("Density", [&](const char* lbl) {
        if (ImGui::DragFloat(lbl, &b.density, 0.05f, 0.01f, 50.0f)) b.computeMass();
    });
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Mass per unit area; changing it recomputes mass/inertia.");

    ImGui::Separator();
    ImGui::TextUnformatted("Appearance");
    float col[3] = {b.colorR / 255.0f, b.colorG / 255.0f, b.colorB / 255.0f};
    responsiveField("Color", [&](const char* lbl) {
        if (ImGui::ColorEdit3(lbl, col)) {
            b.colorR = static_cast<uint8_t>(std::clamp(col[0], 0.0f, 1.0f) * 255.0f);
            b.colorG = static_cast<uint8_t>(std::clamp(col[1], 0.0f, 1.0f) * 255.0f);
            b.colorB = static_cast<uint8_t>(std::clamp(col[2], 0.0f, 1.0f) * 255.0f);
        }
    });
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("This body's viewport color -- fixed once set, independent of its BodyType or\n"
                           "whether it's currently asleep (see the Awake/Asleep indicator below instead).");

    ImGui::TextWrapped("Texture: %s", b.texturePath.empty() ? "(none)" : b.texturePath.c_str());
    responsiveField("Texture path", [&](const char* lbl) {
        if (ImGui::InputText(lbl, texturePathBuffer_, sizeof(texturePathBuffer_))) {
            b.texturePath = texturePathBuffer_;
        }
    });
    if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("IMAGE_PATH")) {
            std::snprintf(texturePathBuffer_, sizeof(texturePathBuffer_), "%s",
                          static_cast<const char*>(payload->Data));
            b.texturePath = texturePathBuffer_;
            log("Set " + std::filesystem::path(texturePathBuffer_).filename().string() + " as texture for " +
                    b.name,
                false);
        }
        ImGui::EndDragDropTarget();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Drag an image here from the FileSystem panel to texture this body, or\n"
                           "type/paste a path. Replaces the flat color fill above; clear to go back to it.");
    sameLineIfFits(buttonWidthEstimate("Clear Texture"));
    if (ImGui::SmallButton("Clear Texture")) {
        texturePathBuffer_[0] = '\0';
        b.texturePath.clear();
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Dynamics");
    responsiveField("Linear Damping", [&](const char* lbl) { ImGui::DragFloat(lbl, &b.linearDamping, 0.02f, 0.0f, 20.0f); });
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Drag that slows this body's movement over time. 0 = none.");
    responsiveField("Angular Damping", [&](const char* lbl) { ImGui::DragFloat(lbl, &b.angularDamping, 0.02f, 0.0f, 20.0f); });
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Drag that slows this body's spin over time. 0 = none.");
    responsiveField("Gravity Scale", [&](const char* lbl) { ImGui::DragFloat(lbl, &b.gravityScale, 0.02f, -5.0f, 5.0f); });
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Multiplies world gravity for this body. 0 = floats, negative = falls upward.");
    if (ImGui::Checkbox("Fixed Rotation", &b.fixedRotation)) b.computeMass();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Locks rotation: torque and off-center collisions can no longer spin this body.");
    ImGui::Checkbox("Is Sensor", &b.isSensor);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Still detects overlap (for scripts) but never physically collides -- for trigger zones.");

    ImGui::Separator();
    const char* typeNames[] = {"Static", "Kinematic", "Dynamic"};
    int typeIdx = static_cast<int>(b.type);
    responsiveField("Type", [&](const char* lbl) {
        if (ImGui::Combo(lbl, &typeIdx, typeNames, 3)) {
            b.type = static_cast<BodyType>(typeIdx);
            b.computeMass();
        }
    });

    const char* matterKindNames[] = {"Rigidbody", "Matter", "OptiMatter"};
    int matterKindIdx = static_cast<int>(b.matterKind);
    responsiveField("Matter Kind", [&](const char* lbl) {
        ImGui::Combo(lbl, &matterKindIdx, matterKindNames, 3);
        b.matterKind = static_cast<MatterKind>(matterKindIdx);
    });
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "Simulation-fidelity dial, independent of Type above. Rigidbody: an ordinary\n"
            "body (default). Matter: MORE accurate than Rigidbody -- tighter sleep/\n"
            "correction/substep handling, at real extra cost. OptiMatter: exempt from\n"
            "forced extra substeps, sleeps sooner with looser thresholds, and snaps out\n"
            "of overlap faster -- cheaper but less accurate, for scenes with many bodies\n"
            "where a little inaccuracy is an acceptable trade.");

    ImGui::TextWrapped("Mass: %.3f   Inertia: %.3f", b.mass, b.inertia);
    if (b.type == BodyType::Dynamic) {
        ImGui::TextColored(b.isAwake ? ImVec4(0.6f, 0.9f, 0.6f, 1.0f) : ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                            b.isAwake ? "Awake" : "Asleep (at rest)");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Sleeping bodies stop being simulated until disturbed -- a performance optimization.");
    }

    ImGui::Separator();
    ImGui::TextWrapped("Script: %s", b.scriptPath.empty() ? "(none)" : b.scriptPath.c_str());

    sameLineIfFits(buttonWidthEstimate("Refresh List"));
    if (ImGui::SmallButton("Refresh List")) refreshAvailableScripts();
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Rescan the project's scripts folder for .lua files.");

    std::string currentRel;
    if (scriptPathBuffer_[0] != '\0') {
        std::error_code relEc;
        auto rel = std::filesystem::relative(scriptPathBuffer_, scriptsDir_, relEc);
        if (!relEc) currentRel = rel.string();
    }
    const char* comboLabel = availableScripts_.empty() ? "(no .lua files in project)"
                              : currentRel.empty()      ? "(choose a script)"
                                                         : currentRel.c_str();
    responsiveField("Script Library", [&](const char* lbl) {
        if (ImGui::BeginCombo(lbl, comboLabel)) {
            for (auto& name : availableScripts_) {
                bool isSelected = (currentRel == name);
                if (ImGui::Selectable(name.c_str(), isSelected)) {
                    std::string full = (scriptsDir_ / name).string();
                    std::snprintf(scriptPathBuffer_, sizeof(scriptPathBuffer_), "%s", full.c_str());
                    scriptEngine_.attachScript(b, scriptPathBuffer_);
                    applyUiKindMarkerOnAttach(b, scriptPathBuffer_);
                    log("Attached " + name + " to " + b.name, false);
                }
            }
            ImGui::EndCombo();
        }
    });
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Pick a script from the project, or drag one in from the FileSystem panel below --\n"
                           "both attach immediately.");

    responsiveField("Script path", [&](const char* lbl) {
        ImGui::InputText(lbl, scriptPathBuffer_, sizeof(scriptPathBuffer_));
    });
    if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("SCRIPT_PATH")) {
            std::snprintf(scriptPathBuffer_, sizeof(scriptPathBuffer_), "%s",
                          static_cast<const char*>(payload->Data));
            scriptEngine_.attachScript(b, scriptPathBuffer_);
            applyUiKindMarkerOnAttach(b, scriptPathBuffer_);
            log("Attached " + std::filesystem::path(scriptPathBuffer_).filename().string() + " to " + b.name,
                false);
        }
        ImGui::EndDragDropTarget();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Drag a .lua file here from the FileSystem panel to attach it immediately, or\n"
                           "type/paste a path and click Attach / Reload.");

    if (ImGui::Button("Attach / Reload")) {
        scriptEngine_.attachScript(b, scriptPathBuffer_);
        applyUiKindMarkerOnAttach(b, scriptPathBuffer_);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Load (or re-load) the script at this path and run it on this body.");
    sameLineIfFits(buttonWidthEstimate("Detach"));
    if (ImGui::Button("Detach")) scriptEngine_.detachScript(b);
    sameLineIfFits(buttonWidthEstimate("Edit"));
    if (ImGui::Button("Edit")) {
        if (scriptPathBuffer_[0] != '\0') openScriptInEditor(scriptPathBuffer_);
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Open this script in the built-in code editor.");

    ImGui::Separator();
    ImGui::TextUnformatted("Connections (springs)");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "Ties this body to another with a spring -- both stay themselves (any shape, any\n"
            "type), unlike a soft body cluster which is made of its own generated particles.\n"
            "This is the same mechanism a Cloth/Jelly's particles use, just between whichever\n"
            "two objects you pick.");
    {
        const char* comboLabel = connectTarget_
                                      ? (connectTarget_->name.empty() ? "(unnamed body)" : connectTarget_->name.c_str())
                                      : "(pick a body)";
        ImGui::SetNextItemWidth(160.0f);
        responsiveField("Connect to", [&](const char* lbl) {
            if (ImGui::BeginCombo(lbl, comboLabel)) {
                for (auto& bp : world_.bodies()) {
                    Body* candidate = bp.get();
                    if (candidate == &b) continue;
                    std::string label =
                        candidate->name.empty() ? ("Body " + std::to_string(candidate->id)) : candidate->name;
                    if (ImGui::Selectable(label.c_str(), candidate == connectTarget_)) connectTarget_ = candidate;
                }
                ImGui::EndCombo();
            }
        });
        sameLineIfFits(80.0f);
        ImGui::SetNextItemWidth(80.0f);
        ImGui::SliderFloat("Stiffness", &springJointStiffness_, 5.0f, 200.0f);
        sameLineIfFits(80.0f);
        ImGui::SetNextItemWidth(80.0f);
        ImGui::SliderFloat("Damping", &springJointDamping_, 0.0f, 10.0f);
        sameLineIfFits(buttonWidthEstimate("Connect"));
        ImGui::BeginDisabled(connectTarget_ == nullptr);
        if (ImGui::Button("Connect")) {
            connectWithSpring(&b, connectTarget_, springJointStiffness_, springJointDamping_);
            connectTarget_ = nullptr;
        }
        ImGui::EndDisabled();
    }

    // List existing connections touching this body, each individually removable.
    for (size_t i = 0; i < springJoints_.size();) {
        SpringJoint& j = springJoints_[i];
        if (j.a != &b && j.b != &b) {
            ++i;
            continue;
        }
        Body* other = (j.a == &b) ? j.b : j.a;
        std::string otherName = (other && !other->name.empty()) ? other->name
                                 : other                          ? ("Body " + std::to_string(other->id))
                                                                   : "(deleted)";
        ImGui::PushID(static_cast<int>(i));
        ImGui::BulletText("%s", otherName.c_str());
        sameLineIfFits(buttonWidthEstimate("Disconnect"));
        bool erased = false;
        if (ImGui::SmallButton("Disconnect")) {
            springJoints_.erase(springJoints_.begin() + static_cast<long>(i));
            erased = true;
        }
        ImGui::PopID();
        if (!erased) ++i;
    }

    ImGui::Separator();
    if (ImGui::Button("Delete Body")) {
        if (dragged_ == &b) dragged_ = nullptr;
        selected_ = nullptr;
        lastInspected_ = nullptr;
        world_.removeBody(&b); // world_.onBodyRemoved detaches any script for us
        ImGui::End();
        return;
    }

    ImGui::End();
}

void EditorApp::drawConsole() {
    ImGui::Begin("Console");
    for (auto& line : consoleLines_) {
        ImVec4 color = line.isError ? ImVec4(1.0f, 0.4f, 0.4f, 1.0f) : ImVec4(0.85f, 0.85f, 0.85f, 1.0f);
        ImGui::TextColored(color, "%s", line.text.c_str());
    }
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f) ImGui::SetScrollHereY(1.0f);
    ImGui::End();
}

void EditorApp::drawFileSystemPanel() {
    ImGui::Begin("FileSystem");

    ImGui::TextDisabled("%s", projectDir_.string().c_str());
    ImGui::Separator();

    // The toolbar below used to be one unconditional row of SameLine()s,
    // which meant buttons silently ran off the right edge (invisible, not
    // just clipped) whenever this panel was narrower than the full row --
    // a real problem since it's a dockable panel the user resizes freely.
    // sameLineIfFits() reproduces imgui_demo's "manual wrapping" recipe:
    // measure the NEXT label (SmallButton only zeroes FramePadding.y, so
    // width is still CalcTextSize + FramePadding.x * 2), take the screen-
    // space right edge of the button just drawn (GetItemRectMax), and only
    // continue the current row if placing the next button there would
    // still land before the panel's right edge -- otherwise let it fall
    // through to a fresh row. GetContentRegionAvail() doesn't work for this:
    // called right after a widget with no SameLine() yet, it already
    // measures the NEXT (still empty) line, not what's left of the current
    // one, so a naive "avail > next width" check is nearly always true and
    // never actually wraps. Since this re-runs every frame, a wider panel
    // naturally collapses back down to fewer rows.
    ImGuiStyle& style = ImGui::GetStyle();
    float fsToolbarRightEdge = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
    auto sameLineIfFits = [&](const char* nextLabel) {
        float w = ImGui::CalcTextSize(nextLabel).x + style.FramePadding.x * 2.0f;
        float lastItemX2 = ImGui::GetItemRectMax().x;
        if (lastItemX2 + style.ItemSpacing.x + w < fsToolbarRightEdge) ImGui::SameLine();
    };

    if (ImGui::SmallButton("New Folder")) {
        newItemParentDir_ = scriptsDir_;
        newItemNameBuffer_[0] = '\0';
        showNewFolderPopup_ = true;
    }
    sameLineIfFits("New Script");
    if (ImGui::SmallButton("New Script")) {
        newItemParentDir_ = scriptsDir_;
        std::snprintf(newItemNameBuffer_, sizeof(newItemNameBuffer_), "new_script.lua");
        newScriptKindIdx_ = 0;
        showNewScriptPopup_ = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Pick a Kind in the popup to also create a live host body running it,\n"
                           "selected in the Inspector/Hierarchy immediately -- \"Plain Script\" is an\n"
                           "ordinary on_start/on_update stub with no host body, same as before UI\n"
                           "Elements existed.");
    sameLineIfFits("Upload Texture");
    if (ImGui::SmallButton("Upload Texture")) uploadTextureVia(scriptsDir_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Opens your OS's file picker; the image you choose is copied into this\n"
                           "project's scripts folder.");
    sameLineIfFits("Refresh");
    if (ImGui::SmallButton("Refresh")) refreshAvailableScripts();
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Rescan for files changed outside the editor.");

    // The 8px gap only makes sense as a same-row separator -- if Copy
    // wouldn't fit on this row anyway, skip straight to it on a new row
    // rather than leaving a dangling gap at the start of that row.
    {
        float copyW = ImGui::CalcTextSize("Copy").x + style.FramePadding.x * 2.0f;
        float lastItemX2 = ImGui::GetItemRectMax().x;
        float gapAndCopyX2 = lastItemX2 + style.ItemSpacing.x + 8.0f + style.ItemSpacing.x + copyW;
        if (gapAndCopyX2 < fsToolbarRightEdge) {
            ImGui::SameLine();
            ImGui::Dummy(ImVec2(8.0f, 0.0f));
            ImGui::SameLine();
        }
    }
    ImGui::BeginDisabled(selectedFsPath_.empty());
    if (ImGui::SmallButton("Copy")) copyFsSelection();
    ImGui::EndDisabled();
    sameLineIfFits("Paste");
    ImGui::BeginDisabled(fsClipboardPath_.empty());
    if (ImGui::SmallButton("Paste")) pasteFsClipboard();
    ImGui::EndDisabled();
    sameLineIfFits("Delete");
    ImGui::BeginDisabled(selectedFsPath_.empty());
    if (ImGui::SmallButton("Delete")) deleteFsSelection();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered() || ImGui::IsItemActive())
        ImGui::SetTooltip("Select a file/folder below, then Ctrl+C / Ctrl+V / Delete also work.");

    ImGui::Separator();
    drawFileTreeNode(scriptsDir_);

    drawFileSystemPopups();

    // Keyboard shortcuts, scoped to this panel having focus OR hover so
    // they don't fire while e.g. typing in the Code Editor or Inspector.
    // Focus alone turned out to be too strict in practice: clicking a
    // Selectable/TreeNodeEx row doesn't reliably hand this window keyboard
    // focus away from whatever previously had it (e.g. a text field left
    // over from a just-closed popup), so a plain click-then-Ctrl+C could
    // silently do nothing. Hover is a reasonable, safe fallback -- if the
    // mouse is over this panel, a keyboard shortcut is unambiguously meant
    // for it.
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows) ||
        ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows)) {
        ImGuiIO& io = ImGui::GetIO();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_C, false) && !selectedFsPath_.empty()) copyFsSelection();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_V, false) && !fsClipboardPath_.empty()) pasteFsClipboard();
        if (ImGui::IsKeyPressed(ImGuiKey_Delete, false) && !selectedFsPath_.empty()) deleteFsSelection();
    }

    ImGui::End();
}

void EditorApp::drawFileTreeNode(const std::filesystem::path& dir) {
    std::vector<std::filesystem::directory_entry> entries;
    std::error_code ec;
    for (auto& e : std::filesystem::directory_iterator(dir, ec)) {
        if (ec) break;
        entries.push_back(e);
    }
    std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) {
        if (a.is_directory() != b.is_directory()) return a.is_directory() > b.is_directory();
        return a.path().filename().string() < b.path().filename().string();
    });

    for (auto& entry : entries) {
        std::string name = entry.path().filename().string();
        ImGui::PushID(entry.path().c_str());

        bool isSelected = (selectedFsPath_ == entry.path());

        if (entry.is_directory()) {
            ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth;
            if (isSelected) flags |= ImGuiTreeNodeFlags_Selected;
            bool open = ImGui::TreeNodeEx(name.c_str(), flags);
            if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) selectedFsPath_ = entry.path();

            if (ImGui::BeginPopupContextItem()) {
                selectedFsPath_ = entry.path();
                if (ImGui::MenuItem("New Folder")) {
                    newItemParentDir_ = entry.path();
                    newItemNameBuffer_[0] = '\0';
                    showNewFolderPopup_ = true;
                }
                if (ImGui::MenuItem("New Script")) {
                    newItemParentDir_ = entry.path();
                    std::snprintf(newItemNameBuffer_, sizeof(newItemNameBuffer_), "new_script.lua");
                    newScriptKindIdx_ = 0;
                    showNewScriptPopup_ = true;
                }
                if (ImGui::MenuItem("Upload Texture")) uploadTextureVia(entry.path());
                ImGui::Separator();
                if (ImGui::MenuItem("Copy")) copyFsSelection();
                ImGui::BeginDisabled(fsClipboardPath_.empty());
                if (ImGui::MenuItem("Paste Into")) pasteFsClipboard();
                ImGui::EndDisabled();
                ImGui::Separator();
                if (ImGui::MenuItem("Delete Folder")) deleteFsSelection();
                ImGui::EndPopup();
            }

            if (open) {
                drawFileTreeNode(entry.path());
                ImGui::TreePop();
            }
        } else {
            bool isLua = entry.path().extension() == ".lua";
            std::string lowerExt = entry.path().extension().string();
            for (char& c : lowerExt) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            bool isImage =
                std::find(kImageExtensions.begin(), kImageExtensions.end(), lowerExt) != kImageExtensions.end();

            // Badges a UI Element script at a glance, even one that was
            // hand-renamed or never went through createUiElement() -- see
            // readUiKindMarker()/applyUiKindMarkerOnAttach().
            std::string displayName = name;
            if (isLua) {
                std::string kind = readUiKindMarker(entry.path());
                if (!kind.empty()) {
                    kind[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(kind[0])));
                    displayName += " [UI: " + kind + "]";
                }
            }

            // Dear ImGui has no italic/oblique font variant loaded (that
            // would need a second font atlas entry), so a distinct text
            // color is the practical stand-in for visually setting image
            // files apart from scripts at a glance in this tree.
            if (isImage) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.65f, 0.75f, 1.0f, 1.0f));
            bool clickedEntry = ImGui::Selectable(displayName.c_str(), isSelected);
            if (isImage) ImGui::PopStyleColor();
            if (clickedEntry) selectedFsPath_ = entry.path();

            if (isLua && ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                openScriptInEditor(entry.path());
            }

            if (isLua && ImGui::BeginDragDropSource(ImGuiDragDropFlags_None)) {
                std::string fullPath = entry.path().string();
                ImGui::SetDragDropPayload("SCRIPT_PATH", fullPath.c_str(), fullPath.size() + 1);
                ImGui::Text("%s", name.c_str());
                ImGui::EndDragDropSource();
            }
            if (isImage && ImGui::BeginDragDropSource(ImGuiDragDropFlags_None)) {
                std::string fullPath = entry.path().string();
                ImGui::SetDragDropPayload("IMAGE_PATH", fullPath.c_str(), fullPath.size() + 1);
                ImGui::Text("%s", name.c_str());
                ImGui::EndDragDropSource();
            }
            if (ImGui::IsItemHovered() && isLua)
                ImGui::SetTooltip("Double-click to edit. Drag onto the Inspector's Script path field to attach.\n"
                                   "Ctrl+C / Ctrl+V / Delete work once selected.");
            if (ImGui::IsItemHovered() && isImage)
                ImGui::SetTooltip(
                    "Drag onto the Inspector's Texture field to texture a body, or onto Settings'\n"
                    "Background Texture field. Ctrl+C / Ctrl+V / Delete work once selected.");

            if (ImGui::BeginPopupContextItem()) {
                selectedFsPath_ = entry.path();
                if (ImGui::MenuItem("Copy")) copyFsSelection();
                ImGui::Separator();
                if (ImGui::MenuItem("Delete File")) deleteFsSelection();
                ImGui::EndPopup();
            }
        }

        ImGui::PopID();
    }
}

void EditorApp::drawFileSystemPopups() {
    if (showNewFolderPopup_) {
        ImGui::OpenPopup("New Folder##fs");
        showNewFolderPopup_ = false;
    }
    if (showNewScriptPopup_) {
        ImGui::OpenPopup("New Script##fs");
        showNewScriptPopup_ = false;
    }
    if (ImGui::BeginPopupModal("New Folder##fs", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Folder name:");
        ImGui::InputText("##foldername", newItemNameBuffer_, sizeof(newItemNameBuffer_));
        if (ImGui::Button("Create")) {
            if (newItemNameBuffer_[0] != '\0') {
                std::error_code ec;
                std::filesystem::create_directory(newItemParentDir_ / newItemNameBuffer_, ec);
                if (ec) log("Could not create folder: " + ec.message(), true);
                refreshAvailableScripts();
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (ImGui::BeginPopupModal("New Script##fs", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Script name:");
        ImGui::InputText("##scriptname", newItemNameBuffer_, sizeof(newItemNameBuffer_));
        ImGui::TextDisabled("%s", ".lua is appended automatically if missing");
        ImGui::Combo("Kind", &newScriptKindIdx_, kNewScriptKinds, kNewScriptKindCount);
        if (newScriptKindIdx_ == 0) {
            ImGui::TextDisabled("An ordinary on_start/on_update stub.");
        } else {
            ImGui::TextDisabled("Creates the script AND a live host body running it, selected immediately.");
        }
        if (ImGui::Button("Create")) {
            const char* kind = kNewScriptKindIds[newScriptKindIdx_];
            if (newItemNameBuffer_[0] != '\0') {
                if (kind[0] == '\0') {
                    std::string name = newItemNameBuffer_;
                    if (name.size() < 4 || name.compare(name.size() - 4, 4, ".lua") != 0) name += ".lua";
                    std::filesystem::path fullPath = newItemParentDir_ / name;
                    std::error_code existsEc;
                    if (!std::filesystem::exists(fullPath, existsEc)) {
                        std::ofstream out(fullPath);
                        out << "function on_start(body)\nend\n\nfunction on_update(body, dt)\nend\n";
                    } else {
                        log("A file named " + name + " already exists there.", true);
                    }
                    refreshAvailableScripts();
                } else {
                    createUiElement(newItemParentDir_, newItemNameBuffer_, kind);
                }
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

}

void EditorApp::drawCodeEditorPanel() {
    // Closed by default (see showCodeEditor_'s initializer) -- only opened
    // by double-clicking a script in FileSystem or "Edit" in the Inspector.
    if (!showCodeEditor_) return;

    std::string title = "Code Editor";
    if (openScriptPath_.empty()) {
        title += " (no file open)";
    } else {
        title += " - " + openScriptPath_.filename().string();
        if (codeEditorDirty_) title += " *";
    }
    // The part after ### is the stable window/dock identity; the part
    // before it is just the visible label, which can safely change every
    // frame (dirty marker, filename) without breaking docking.
    title += "###CodeEditor";

    if (!ImGui::Begin(title.c_str(), &showCodeEditor_)) {
        ImGui::End();
        return;
    }

    if (ImGui::Button("Save")) {
        saveCodeEditor();
        codeEditorAutosaveTimer_ = 0.0f;
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Write changes to disk (and re-attach if this script is on the selected body).");
    ImGui::SameLine();
    if (openScriptPath_.empty()) {
        ImGui::TextDisabled("Double-click a .lua file in FileSystem, or use Edit in the Inspector.");
    } else {
        ImGui::TextDisabled("%s", openScriptPath_.string().c_str());
    }
    if (codeEditorDirty_) {
        ImGui::SameLine();
        ImGui::TextDisabled("(autosaving in %.0fs)", std::max(0.0f, kAutosaveDelaySeconds - codeEditorAutosaveTimer_));
    }

    ImGui::Separator();
    codeEditor_.Render("##CodeEditorBody", ImVec2(0.0f, 0.0f), false);
    if (codeEditor_.IsTextChanged()) {
        codeEditorDirty_ = true;
        codeEditorAutosaveTimer_ = 0.0f; // any new keystroke pushes the autosave back out
    }

    // Debounced autosave: once editing has been quiet for kAutosaveDelaySeconds,
    // write to disk automatically so work is never lost to a forgotten Save
    // click, without spamming disk writes on every keystroke.
    if (codeEditorDirty_ && !openScriptPath_.empty()) {
        codeEditorAutosaveTimer_ += ImGui::GetIO().DeltaTime;
        if (codeEditorAutosaveTimer_ >= kAutosaveDelaySeconds) {
            saveCodeEditor();
        }
    }

    ImGui::End();
}

void EditorApp::drawScriptUiPanel() {
    ImGui::Begin("Script UI");
    ImGui::TextDisabled("Controls defined by attached scripts' on_gui() appear below.");
    ImGui::Separator();
    scriptEngine_.updateGui(world_);
    ImGui::End();
}

// The no-script rendering path for a UI Element -- same per-kind behavior
// uiElementTemplateScript()'s generated Lua would give it, just issued as
// direct ImGui:: calls instead of through the ui.* Lua table. Only called
// for a body with no attached on_gui (see drawUiOverlay()); a scripted UI
// Element (including any pre-existing one from before this feature, or a
// custom script deliberately attached later) always keeps running its own
// script instead -- this is purely the fallback for a body created with no
// script at all.
void EditorApp::drawNativeUiElement(Body& body) {
    const std::string& kind = body.uiKind;
    std::string label = body.uiHideText ? ("##" + body.name) : body.uiText;

    if (kind == "text") {
        if (!body.texturePath.empty()) {
            sf::Texture* tex = renderer_.getOrLoadTexture(body.texturePath);
            if (tex) {
                ImGui::Image(*tex, sf::Vector2f(24.0f, 24.0f));
                ImGui::SameLine();
            }
        }
        ImGui::TextUnformatted(body.uiText.c_str());
    } else if (kind == "checkbox") {
        if (!body.texturePath.empty()) {
            sf::Texture* tex = renderer_.getOrLoadTexture(body.texturePath);
            if (tex) {
                ImGui::Image(*tex, sf::Vector2f(24.0f, 24.0f));
                ImGui::SameLine();
            }
        }
        bool checked = body.uiValue > 0.5f;
        if (ImGui::Checkbox(label.c_str(), &checked)) body.uiValue = checked ? 1.0f : 0.0f;
    } else if (kind == "slider") {
        if (!body.texturePath.empty()) {
            sf::Texture* tex = renderer_.getOrLoadTexture(body.texturePath);
            if (tex) {
                ImGui::Image(*tex, sf::Vector2f(24.0f, 24.0f));
                ImGui::SameLine();
            }
        }
        ImGui::SliderFloat(label.c_str(), &body.uiValue, body.uiMin, body.uiMax);
    } else if (kind == "input_field") {
        // uiText IS the field's own live content here (not a separate
        // label the way button/checkbox/slider use it) -- an empty ImGui
        // label keeps only the text box itself visible.
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s", body.uiText.c_str());
        std::string fieldLabel = "##" + body.name;
        if (ImGui::InputText(fieldLabel.c_str(), buf, sizeof(buf))) body.uiText = buf;
    } else if (kind == "panel") {
        // uiMin/uiMax are repurposed as width/height (pixels) for this kind
        // only -- see spawnUiElementHost(). texture_path draws an image
        // fill; otherwise a flat rect in the body's own Appearance color.
        float w = std::max(20.0f, body.uiMin);
        float h = std::max(20.0f, body.uiMax);
        sf::Texture* tex = body.texturePath.empty() ? nullptr : renderer_.getOrLoadTexture(body.texturePath);
        if (tex) {
            ImGui::Image(*tex, sf::Vector2f(w, h));
        } else {
            ImVec2 p0 = ImGui::GetCursorScreenPos();
            ImVec2 p1(p0.x + w, p0.y + h);
            ImU32 col = IM_COL32(body.colorR, body.colorG, body.colorB, 255);
            ImGui::GetWindowDrawList()->AddRectFilled(p0, p1, col);
            ImGui::Dummy(ImVec2(w, h));
        }
    } else {
        // "button", and the default for anything unrecognized. No script
        // means no custom action on click -- it's just a placeholder,
        // ready for a script to be attached later for real behavior.
        if (!body.texturePath.empty()) {
            sf::Texture* tex = renderer_.getOrLoadTexture(body.texturePath);
            if (tex) {
                ImGui::ImageButton(body.name.c_str(), *tex, sf::Vector2f(32.0f, 32.0f));
                if (!body.uiHideText) {
                    ImGui::SameLine();
                    ImGui::TextUnformatted(body.uiText.c_str());
                }
                return;
            }
        }
        ImGui::Button(body.uiText.c_str());
    }
}

void EditorApp::drawUiOverlay() {
    // Renders the same on_gui() widgets a second time, as floating HUD
    // windows drawn directly over the Viewport image -- so scripted
    // controls (e.g. "Spawn 1000") are usable without switching over to the
    // Script UI tab. ui.button() etc are plain ImGui calls, so calling
    // on_gui again here just issues them into these windows' own ID scope
    // instead; a click here and a click on the Script UI panel's copy of
    // the same button are entirely independent (different ImGui windows/ID
    // stacks).
    //
    // One small window PER body with an on_gui script, not one shared
    // window for all of them (that was the old behavior) -- each is
    // individually draggable so a UI Element can be arranged anywhere over
    // the viewport, Godot-Control-node style, instead of always stacking in
    // a fixed top-left column. Position is stored on the body itself
    // (Body::uiOverlayX/Y, screen pixels offset from the viewport's own
    // top-left) so it persists through scene save/load and survives the
    // viewport being resized/panned/zoomed (a screen offset, not a world
    // position).
    //
    // Two restrictions on the drag itself: it can't pull a window outside
    // the viewport image (clamped below, every frame -- a widget dragged
    // off into the Hierarchy/Inspector panels would be easy to lose track
    // of and hard to reach again), and it's disabled entirely while Playing
    // (ImGuiWindowFlags_NoMove added below) -- the buttons/checkboxes/
    // sliders themselves stay fully clickable either way, since NoMove only
    // affects dragging the window by its background, not interacting with
    // widgets inside it; this just stops an accidental drag mid-click from
    // relocating a control while you're actively testing the simulation.
    if (!viewportTabVisible_) return; // Code Editor tab is showing instead -- don't draw over it
    if (viewportSize_.x <= 1 || viewportSize_.y <= 1) return;

    // Snapshot up front -- same reentrancy concern ScriptEngine::updateGui()
    // already documents: a button's on_gui can spawn/remove bodies mid-call,
    // which would otherwise invalidate whatever this loop is iterating.
    // Includes native (scriptless) UI Elements now, not just scripted ones --
    // a body created via createObjectInScene()'s UI kinds has isUiElement
    // set but no attached script, and draws itself via drawNativeUiElement()
    // below instead of scriptEngine_.callGui().
    std::vector<Body*> guiBodies;
    for (auto& bp : world_.bodies()) {
        if (scriptEngine_.hasGui(*bp) || bp->isUiElement) guiBodies.push_back(bp.get());
    }
    if (guiBodies.empty()) return; // nothing to show -- skip drawing anything

    float viewportW = static_cast<float>(viewportSize_.x);
    float viewportH = static_cast<float>(viewportSize_.y);

    int cascadeIndex = 0;
    for (Body* body : guiBodies) {
        if (body->uiOverlayX < 0.0f || body->uiOverlayY < 0.0f) {
            // Never positioned before -- seed a cascaded default (not all
            // stacked at the same spot) rather than leaving it at -1/-1.
            body->uiOverlayX = 12.0f + 26.0f * static_cast<float>(cascadeIndex);
            body->uiOverlayY = 12.0f + 26.0f * static_cast<float>(cascadeIndex);
            ++cascadeIndex;
        }

        // ImGuiCond_FirstUseEver, not Always: applies body->uiOverlay{X,Y}
        // only the first time THIS window ID appears (a fresh session, or
        // a body ImGui hasn't seen before) -- once the user drags it, ImGui
        // owns the position from then on and this stops re-applying it, so
        // the drag actually sticks instead of snapping back every frame.
        ImGui::SetNextWindowPos(
            ImVec2(viewportScreenPos_.x + body->uiOverlayX, viewportScreenPos_.y + body->uiOverlayY),
            ImGuiCond_FirstUseEver);
        // Panel is a background/decoration element, so it keeps the old
        // translucent look (see body->uiKind); every other kind is fully
        // opaque now, so it doesn't look like a ghost floating over the
        // viewport -- see the "Panel" doc entry in the README for why this
        // one kind is the exception.
        ImGui::SetNextWindowBgAlpha(body->uiKind == "panel" ? 0.6f : 1.0f);
        ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                                  ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize |
                                  ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;
        if (running_) flags |= ImGuiWindowFlags_NoMove; // Playing -- click away, but don't drag away
        std::string windowId = "##UiElemOverlay_" + std::to_string(body->id);
        if (ImGui::Begin(windowId.c_str(), nullptr, flags)) {
            if (scriptEngine_.hasGui(*body)) {
                scriptEngine_.callGui(*body);
            } else {
                drawNativeUiElement(*body);
            }

            // Clamp so a drag (or a stale position from a since-shrunk
            // viewport) can't leave the window partially or fully outside
            // the viewport image -- snap it back in immediately, same
            // frame, rather than just letting it hang off the edge.
            ImVec2 winSize = ImGui::GetWindowSize();
            float maxX = std::max(0.0f, viewportW - winSize.x);
            float maxY = std::max(0.0f, viewportH - winSize.y);
            ImVec2 winPos = ImGui::GetWindowPos();
            float localX = std::clamp(winPos.x - viewportScreenPos_.x, 0.0f, maxX);
            float localY = std::clamp(winPos.y - viewportScreenPos_.y, 0.0f, maxY);
            ImVec2 clampedPos(viewportScreenPos_.x + localX, viewportScreenPos_.y + localY);
            if (std::abs(clampedPos.x - winPos.x) > 0.5f || std::abs(clampedPos.y - winPos.y) > 0.5f) {
                ImGui::SetWindowPos(clampedPos);
            }
            body->uiOverlayX = localX;
            body->uiOverlayY = localY;
        }
        ImGui::End();
    }
}

} // namespace p2d::app
