#pragma once

#include "p2d/SoftBody.hpp"
#include "p2d/SpringJoint.hpp"
#include "p2d/World.hpp"

#include <nlohmann/json.hpp>

#include <functional>
#include <vector>

namespace p2d::app {

// Scene save/load: deliberately has zero dependency on SFML/ImGui/
// ScriptEngine/EditorApp, so it can be exercised headlessly in
// tests/scene_persistence_test.cpp against a plain p2d::World the same way
// every other engine-level test does -- this is a real, easy-to-get-subtly-
// wrong feature (JSON schema, reconstruction order, script re-attachment),
// worth verifying the same way as everything else in engine/, not just via
// compilation and code review.

// Serializes every body in `world`, plus the given soft bodies and spring
// joints, to a JSON value suitable for writing to disk and reconstructing
// later via loadScene(). Soft bodies/spring joints are passed in explicitly
// (rather than derived from `world`) since World has no concept of either
// -- they're constructs layered on top by the editor.
nlohmann::json saveScene(const World& world, const std::vector<SoftBody>& softBodies,
                         const std::vector<SpringJoint>& springJoints);

// Rebuilds `world` (cleared first) from previously-saved JSON, calling
// `attachScript(body, path)` for any body with a saved, non-empty script
// path, and populating `outSoftBodies`/`outSpringJoints` by re-resolving
// particle/body names via World::findByName() (a fresh World has all new
// Body instances, so soft body/spring joint topology can't be restored via
// raw pointers). Returns false, leaving `world` and the outputs untouched,
// if `j` doesn't parse as a valid scene.
bool loadScene(const nlohmann::json& j, World& world, std::vector<SoftBody>& outSoftBodies,
               std::vector<SpringJoint>& outSpringJoints,
               const std::function<void(Body&, const std::string& scriptPath)>& attachScript);

} // namespace p2d::app
