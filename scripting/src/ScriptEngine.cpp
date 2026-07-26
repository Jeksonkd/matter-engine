#include "p2d/script/ScriptEngine.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace p2d::script {

ScriptEngine::ScriptEngine() = default;

void ScriptEngine::reset() {
    // Order matters: attachments_ holds sol2 references into lua_, so they
    // must be destroyed while the old lua_ is still alive. Replacing lua_
    // first (e.g. via `*this = ScriptEngine()`) destroys those references'
    // backing state before they're released -- a use-after-free.
    attachments_.clear();
    lua_ = sol::state();
}

void ScriptEngine::log(const std::string& msg) {
    if (onLog) onLog(msg);
    else std::printf("%s\n", msg.c_str());
}

void ScriptEngine::reportError(const sol::error& e, const std::string& context) {
    std::string msg = "[lua] " + context + ": " + e.what();
    if (onError) onError(msg);
    else std::fprintf(stderr, "%s\n", msg.c_str());
}

void ScriptEngine::bindWorld(World& world) {
    lua_.open_libraries(sol::lib::base, sol::lib::math, sol::lib::string, sol::lib::table, sol::lib::os);

    lua_.set_function("print", [this](sol::variadic_args va) {
        std::string line;
        sol::function tostring = lua_["tostring"];
        bool first = true;
        for (auto v : va) {
            if (!first) line += "\t";
            first = false;
            line += tostring(v).get<std::string>();
        }
        log(line);
    });

    lua_.new_usertype<Vec2>("Vec2",
        sol::constructors<Vec2(), Vec2(float, float)>(),
        "x", &Vec2::x,
        "y", &Vec2::y,
        "length", &Vec2::length,
        "normalized", &Vec2::normalized,
        "dot", &Vec2::dot,
        sol::meta_function::addition, [](const Vec2& a, const Vec2& b) { return a + b; },
        sol::meta_function::subtraction, [](const Vec2& a, const Vec2& b) { return a - b; },
        sol::meta_function::multiplication,
        sol::overload([](const Vec2& a, float s) { return a * s; },
                       [](float s, const Vec2& a) { return a * s; }),
        sol::meta_function::to_string,
        [](const Vec2& v) { return "(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ")"; });

    lua_.new_enum("BodyType", "Static", BodyType::Static, "Kinematic", BodyType::Kinematic, "Dynamic",
                  BodyType::Dynamic);

    lua_.new_usertype<Body>("Body",
        "position", &Body::position,
        "rotation", &Body::rotation,
        "velocity", &Body::velocity,
        "angular_velocity", &Body::angularVelocity,
        "restitution", &Body::restitution,
        "friction", &Body::friction,
        "name", &Body::name,
        // Setting type changes what computeMass() produces (Static/Kinematic
        // always get infinite mass) -- wrapped so a script assignment
        // recomputes mass/inertia immediately, same as the Inspector's Type
        // dropdown does, rather than leaving stale mass data until
        // something else happens to call computeMass().
        "type",
        sol::property([](const Body& b) { return b.type; },
                       [](Body& b, BodyType t) {
                           b.type = t;
                           b.computeMass();
                       }),
        "color_r", &Body::colorR,
        "color_g", &Body::colorG,
        "color_b", &Body::colorB,
        "set_color",
        [](Body& b, int r, int g, int bl) {
            b.colorR = static_cast<uint8_t>(std::clamp(r, 0, 255));
            b.colorG = static_cast<uint8_t>(std::clamp(g, 0, 255));
            b.colorB = static_cast<uint8_t>(std::clamp(bl, 0, 255));
        },
        "ui_text", &Body::uiText,
        "ui_value", &Body::uiValue,
        "ui_min", &Body::uiMin,
        "ui_max", &Body::uiMax,
        "ui_hide_text", &Body::uiHideText,
        "texture_path", &Body::texturePath,
        "matter_kind", &Body::matterKind,
        // Read: 0 for a non-circle shape, same convention as the getter
        // always had. Write: only takes effect if this body is ALREADY a
        // circle (silently ignored otherwise, same as the Inspector's
        // Radius field only being shown for circles) -- use set_circle()
        // below to convert a non-circle shape instead.
        "radius",
        sol::property(
            [](const Body& b) { return b.shape.type == ShapeType::Circle ? b.shape.radius : 0.0f; },
            [](Body& b, float r) {
                if (b.shape.type != ShapeType::Circle) return;
                b.shape.radius = r;
                b.computeMass();
                b.wake();
            }),
        // Shape-kind conversions -- rebuild this body's shape from scratch
        // (any previous shape is discarded) and recompute mass/inertia to
        // match, same as the viewport's Resize tool/Inspector's Half Width
        // / Height field do for box <-> box resizing, generalized to also
        // cover switching shape KIND (e.g. box -> circle), which neither of
        // those existing paths offers.
        "set_circle",
        [](Body& b, float r) {
            b.shape = ShapeData::MakeCircle(r);
            b.computeMass();
            b.wake();
        },
        "set_box",
        [](Body& b, float halfWidth, float halfHeight) {
            b.shape = ShapeData::MakeBox(halfWidth, halfHeight);
            b.computeMass();
            b.wake();
        },
        // A regular N-gon (sides >= 3) -- see World::create_polygon's same
        // reasoning for covering triangle/pentagon/hexagon/etc. with one
        // function rather than a named one per shape.
        "set_polygon",
        [](Body& b, int sides, float circumradius) {
            b.shape = ShapeData::MakeRegularPolygon(sides, circumradius);
            b.computeMass();
            b.wake();
        },
        "linear_damping", &Body::linearDamping,
        "angular_damping", &Body::angularDamping,
        "gravity_scale", &Body::gravityScale,
        "is_sensor", &Body::isSensor,
        "fixed_rotation",
        sol::property([](const Body& b) { return b.fixedRotation; },
                       [](Body& b, bool v) {
                           b.fixedRotation = v;
                           b.computeMass();
                       }),
        "density",
        sol::property([](const Body& b) { return b.density; },
                       [](Body& b, float d) {
                           b.density = d;
                           b.computeMass();
                       }),
        "mass", sol::readonly_property([](const Body& b) { return b.mass; }),
        "inertia", sol::readonly_property([](const Body& b) { return b.inertia; }),
        "is_awake", sol::readonly_property([](const Body& b) { return b.isAwake; }),
        "apply_force", [](Body& b, const Vec2& f) { b.applyForce(f); },
        "apply_force_at_point", [](Body& b, const Vec2& f, const Vec2& p) { b.applyForceAtPoint(f, p); },
        "apply_impulse", [](Body& b, const Vec2& imp) { b.applyLinearImpulse(imp); },
        "apply_torque", [](Body& b, float t) { b.applyTorque(t); },
        "wake", [](Body& b) { b.wake(); },
        "set_velocity",
        [](Body& b, float x, float y) {
            b.velocity = Vec2(x, y);
            b.wake();
        },
        "set_position",
        [](Body& b, float x, float y) {
            b.position = Vec2(x, y);
            b.wake();
        });

    lua_.new_enum("MatterKind", "Rigidbody", MatterKind::Rigidbody, "Matter", MatterKind::Matter, "OptiMatter",
                  MatterKind::OptiMatter);

    // Matter: a point-mass particle, genuinely separate from Body (no
    // rotation/torque/inertia at all -- see Matter.hpp) rather than a
    // rigidbody with a cheaper preset. Deliberately a smaller surface than
    // Body's: no .rotation/.angular_velocity/.type/.apply_torque, since
    // none of those mean anything for something that never rotates.
    lua_.new_usertype<Matter>("Matter",
        "position", &Matter::position,
        "velocity", &Matter::velocity,
        "restitution", &Matter::restitution,
        "friction", &Matter::friction,
        "linear_damping", &Matter::linearDamping,
        "gravity_scale", &Matter::gravityScale,
        "name", &Matter::name,
        "matter_kind", &Matter::kind,
        // Wrapped (unlike Body's plain member-pointer fields) so changing
        // it recomputes mass immediately, matching density's own setter
        // just below -- a plain property here would leave mass stale until
        // density also happened to be touched.
        "radius",
        sol::property([](const Matter& m) { return m.radius; },
                       [](Matter& m, float r) {
                           m.radius = r;
                           m.computeMass();
                       }),
        "texture_path", &Matter::texturePath,
        "color_r", &Matter::colorR,
        "color_g", &Matter::colorG,
        "color_b", &Matter::colorB,
        "set_color",
        [](Matter& m, int r, int g, int b) {
            m.colorR = static_cast<uint8_t>(std::clamp(r, 0, 255));
            m.colorG = static_cast<uint8_t>(std::clamp(g, 0, 255));
            m.colorB = static_cast<uint8_t>(std::clamp(b, 0, 255));
        },
        "is_awake", sol::readonly_property([](const Matter& m) { return m.isAwake; }),
        "mass", sol::readonly_property([](const Matter& m) { return m.mass; }),
        "density",
        sol::property([](const Matter& m) { return m.density; },
                       [](Matter& m, float d) {
                           m.density = d;
                           m.computeMass();
                       }),
        "apply_force", [](Matter& m, const Vec2& f) { m.applyForce(f); },
        "apply_impulse", [](Matter& m, const Vec2& imp) { m.applyLinearImpulse(imp); },
        "wake", [](Matter& m) { m.wake(); },
        "set_velocity",
        [](Matter& m, float x, float y) {
            m.velocity = Vec2(x, y);
            m.wake();
        },
        "set_position",
        [](Matter& m, float x, float y) {
            m.position = Vec2(x, y);
            m.wake();
        });

    lua_.new_usertype<World>("World",
        "gravity", &World::gravity,
        "create_circle",
        [](World& w, float x, float y, float radius, BodyType type) {
            return w.createCircle({x, y}, radius, type);
        },
        "create_box",
        [](World& w, float x, float y, float hw, float hh, BodyType type) {
            return w.createBox({x, y}, hw, hh, type);
        },
        // A regular N-gon -- covers triangle (sides=3), pentagon (5),
        // hexagon (6), and beyond with one function, same shape the
        // editor's Spawn tool places (ShapeData::MakeRegularPolygon).
        "create_polygon",
        [](World& w, float x, float y, int sides, float circumradius, BodyType type) {
            return w.createBody(ShapeData::MakeRegularPolygon(sides, circumradius), {x, y}, type);
        },
        "find_body", [](World& w, const std::string& name) { return w.findByName(name); },
        "count", [](World& w) { return static_cast<int>(w.bodies().size()); },
        "remove_body", [](World& w, Body* b) { if (b) w.removeBody(b); },
        // Returns every body currently in the world as a plain Lua array
        // (1-indexed, use ipairs) -- lets a script inspect the existing
        // scene (e.g. "what's the highest ball already resting here?")
        // instead of only ever creating new bodies blind to what's there.
        "bodies", [this](World& w) {
            sol::table t = lua_.create_table(static_cast<int>(w.bodies().size()), 0);
            int i = 1;
            for (auto& b : w.bodies()) t[i++] = b.get();
            return t;
        },
        // Tracking helpers: count (or fetch every match as a 1-indexed
        // table, same convention as bodies() above) bodies sharing a name,
        // a BodyType, or a MatterKind -- e.g. a HUD script showing "N
        // Matter bodies" via count_by_matter_kind, or a cleanup script
        // removing every body named "debris" via find_bodies_by_name.
        // Names aren't required to be unique (unlike find_body(), which
        // returns only the first match), hence the _by_name variants here.
        "count_by_name",
        [](World& w, const std::string& name) {
            int n = 0;
            for (auto& b : w.bodies())
                if (b->name == name) ++n;
            return n;
        },
        "count_by_type",
        [](World& w, BodyType type) {
            int n = 0;
            for (auto& b : w.bodies())
                if (b->type == type) ++n;
            return n;
        },
        "count_by_matter_kind",
        [](World& w, MatterKind kind) {
            int n = 0;
            for (auto& b : w.bodies())
                if (b->matterKind == kind) ++n;
            return n;
        },
        "find_bodies_by_name",
        [this](World& w, const std::string& name) {
            sol::table t = lua_.create_table();
            int i = 1;
            for (auto& b : w.bodies())
                if (b->name == name) t[i++] = b.get();
            return t;
        },
        "find_bodies_by_type",
        [this](World& w, BodyType type) {
            sol::table t = lua_.create_table();
            int i = 1;
            for (auto& b : w.bodies())
                if (b->type == type) t[i++] = b.get();
            return t;
        },
        "find_bodies_by_matter_kind",
        [this](World& w, MatterKind kind) {
            sol::table t = lua_.create_table();
            int i = 1;
            for (auto& b : w.bodies())
                if (b->matterKind == kind) t[i++] = b.get();
            return t;
        },
        // Matter: a point-mass particle, not a rigidbody -- see Matter.hpp
        // and the Matter usertype above for what's different about it.
        "create_matter",
        [](World& w, float x, float y, float radius, MatterKind kind) { return w.createMatter({x, y}, radius, kind); },
        "find_matter", [](World& w, const std::string& name) { return w.findMatterByName(name); },
        "matter_count", [](World& w) { return static_cast<int>(w.matter().size()); },
        "remove_matter", [](World& w, Matter* m) { if (m) w.removeMatter(m); },
        "matter", [this](World& w) {
            sol::table t = lua_.create_table(static_cast<int>(w.matter().size()), 0);
            int i = 1;
            for (auto& m : w.matter()) t[i++] = m.get();
            return t;
        },
        // Same tracking helpers as above, for Matter particles -- only by
        // kind (Matter/OptiMatter; Rigidbody never applies to a particle)
        // since matter() above already covers "give me all of them" and
        // find_matter()/matter_count() already cover the name/total cases.
        "count_matter_by_kind",
        [](World& w, MatterKind kind) {
            int n = 0;
            for (auto& m : w.matter())
                if (m->kind == kind) ++n;
            return n;
        },
        "find_matter_by_kind",
        [this](World& w, MatterKind kind) {
            sol::table t = lua_.create_table();
            int i = 1;
            for (auto& m : w.matter())
                if (m->kind == kind) t[i++] = m.get();
            return t;
        });

    lua_["world"] = &world;

    // Lets a script re-attach itself to a body it just spawned (e.g. a
    // reproducing/self-replicating script) without needing to know its own
    // file path from outside -- SCRIPT_PATH is injected per-attachment below.
    lua_.set_function("attach_script", [this](Body* b, const std::string& path) {
        if (b) attachScript(*b, path);
    });
}

void ScriptEngine::attachScript(Body& body, const std::string& path) {
    detachScript(body);

    sol::environment env(lua_, sol::create, lua_.globals());
    env["SCRIPT_PATH"] = path;
    sol::protected_function_result result = lua_.script_file(path, env, sol::script_pass_on_error);
    if (!result.valid()) {
        sol::error err = result;
        reportError(err, "loading " + path);
        return;
    }

    Attached att;
    att.path = path;
    att.env = env;

    sol::object updateObj = env["on_update"];
    if (updateObj.get_type() == sol::type::function) {
        att.onUpdate = updateObj.as<sol::protected_function>();
    }

    sol::object guiObj = env["on_gui"];
    if (guiObj.get_type() == sol::type::function) {
        att.onGui = guiObj.as<sol::protected_function>();
    }

    body.scriptPath = path;
    attachments_[body.id] = std::move(att);

    sol::object startObj = env["on_start"];
    if (startObj.get_type() == sol::type::function) {
        auto startFn = startObj.as<sol::protected_function>();
        sol::protected_function_result r = startFn(&body);
        if (!r.valid()) {
            sol::error err = r;
            reportError(err, "on_start in " + path);
        }
    }
}

void ScriptEngine::detachScript(Body& body) {
    attachments_.erase(body.id);
    body.scriptPath.clear();
}

bool ScriptEngine::hasScript(const Body& body) const {
    return attachments_.find(body.id) != attachments_.end();
}

void ScriptEngine::update(World& world, float dt) {
    // Snapshot the ids up front: a script's on_update can attach new scripts
    // (self-replication) or remove bodies (death), both of which mutate
    // attachments_ mid-call. Iterating the map directly while it's being
    // mutated is undefined behavior (insert can rehash and invalidate the
    // very iterator driving this loop). Re-look-up by id each time instead,
    // which stays correct across rehashes/erases.
    std::vector<int> ids;
    ids.reserve(attachments_.size());
    for (auto& [id, att] : attachments_) ids.push_back(id);

    for (int id : ids) {
        auto it = attachments_.find(id);
        if (it == attachments_.end()) continue; // removed earlier in this same pass
        if (!it->second.onUpdate.valid()) continue;

        // Copy out before calling: the call itself may erase this very
        // entry (e.g. the script removes its own body), which would leave
        // a reference into the map dangling for the error-reporting below.
        sol::protected_function onUpdateFn = it->second.onUpdate;
        std::string path = it->second.path;

        Body* body = nullptr;
        for (auto& b : world.bodies()) {
            if (b->id == id) {
                body = b.get();
                break;
            }
        }
        if (!body) continue;

        sol::protected_function_result r = onUpdateFn(body, dt);
        if (!r.valid()) {
            sol::error err = r;
            reportError(err, "on_update in " + path);
        }
    }
}

void ScriptEngine::updateGui(World& world) {
    // Same reentrancy-safe snapshot pattern as update() -- a button's
    // on_gui can just as easily spawn/attach/remove bodies mid-call.
    std::vector<int> ids;
    ids.reserve(attachments_.size());
    for (auto& [id, att] : attachments_) ids.push_back(id);

    for (int id : ids) {
        auto it = attachments_.find(id);
        if (it == attachments_.end()) continue;
        if (!it->second.onGui.valid()) continue;

        sol::protected_function onGuiFn = it->second.onGui;
        std::string path = it->second.path;

        Body* body = nullptr;
        for (auto& b : world.bodies()) {
            if (b->id == id) {
                body = b.get();
                break;
            }
        }
        if (!body) continue;

        sol::protected_function_result r = onGuiFn(body);
        if (!r.valid()) {
            sol::error err = r;
            reportError(err, "on_gui in " + path);
        }
    }
}

bool ScriptEngine::hasGuiAttachments() const {
    for (auto& [id, att] : attachments_) {
        if (att.onGui.valid()) return true;
    }
    return false;
}

bool ScriptEngine::hasGui(const Body& body) const {
    auto it = attachments_.find(body.id);
    return it != attachments_.end() && it->second.onGui.valid();
}

void ScriptEngine::callGui(Body& body) {
    auto it = attachments_.find(body.id);
    if (it == attachments_.end() || !it->second.onGui.valid()) return;

    sol::protected_function onGuiFn = it->second.onGui;
    std::string path = it->second.path;

    sol::protected_function_result r = onGuiFn(&body);
    if (!r.valid()) {
        sol::error err = r;
        reportError(err, "on_gui in " + path);
    }
}

} // namespace p2d::script
