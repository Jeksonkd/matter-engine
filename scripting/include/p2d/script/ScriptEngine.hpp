#pragma once

#include "p2d/World.hpp"

#include <sol/sol.hpp>

#include <functional>
#include <string>
#include <unordered_map>

namespace p2d::script {

// Embeds Lua (via sol2) and binds the physics API so .lua files can be
// attached to individual bodies to drive their behavior. Each attached
// script runs in its own sandboxed environment so scripts don't clobber
// each other's globals, and script errors are reported through callbacks
// rather than crashing the host app.
class ScriptEngine {
public:
    ScriptEngine();

    std::function<void(const std::string&)> onLog;
    std::function<void(const std::string&)> onError;

    // Registers the Vec2/Body/World usertypes and exposes `world` as a
    // global so scripts can spawn bodies (e.g. world:create_circle(...)).
    void bindWorld(World& world);

    // Discards all script attachments and starts a fresh Lua state. Must be
    // used instead of replacing the whole ScriptEngine object: attachments_
    // holds sol2 references into lua_, so it has to be torn down *before*
    // lua_ is destroyed/replaced, not after (member destruction order would
    // get that backwards and use-after-free the old Lua state).
    void reset();

    // (Re)loads `path` and attaches it to `body`. Calls on_start(body) once
    // if the script defines it.
    void attachScript(Body& body, const std::string& path);
    void detachScript(Body& body);
    bool hasScript(const Body& body) const;

    // Calls on_update(body, dt) for every currently attached script whose
    // body still exists in `world`.
    void update(World& world, float dt);

    // Calls on_gui(body) for every currently attached script that defines
    // it, once per rendered frame (not per physics step, unlike on_update).
    // Meant to be invoked from inside an active ImGui window, since a host
    // (see EditorApp::bindUiApi) typically binds a `ui.*` Lua table backed
    // by raw ImGui:: calls -- those need a window context to land in.
    void updateGui(World& world);

    // True if at least one currently attached script defines on_gui() --
    // lets a host skip drawing an otherwise-empty GUI container (see
    // EditorApp's Viewport overlay).
    bool hasGuiAttachments() const;

    // Single-body counterparts to updateGui()/hasGuiAttachments() above --
    // let a host draw one body's on_gui() into a container of its own
    // (EditorApp's Viewport overlay gives each UI Element its own small,
    // individually draggable window rather than stacking every attached
    // script's widgets into one shared one; see EditorApp::drawUiOverlay()).
    bool hasGui(const Body& body) const;
    void callGui(Body& body);

    sol::state& lua() { return lua_; }

private:
    struct Attached {
        std::string path;
        sol::environment env;
        sol::protected_function onUpdate;
        sol::protected_function onGui;
    };

    sol::state lua_;
    std::unordered_map<int, Attached> attachments_;

    void log(const std::string& msg);
    void reportError(const sol::error& e, const std::string& context);
};

} // namespace p2d::script
