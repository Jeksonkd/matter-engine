#include "ScenePersistence.hpp"

#include <filesystem>

namespace p2d::app {

namespace {

nlohmann::json vec2ToJson(const Vec2& v) { return {{"x", v.x}, {"y", v.y}}; }
Vec2 vec2FromJson(const nlohmann::json& j) { return Vec2(j.value("x", 0.0f), j.value("y", 0.0f)); }

} // namespace

nlohmann::json saveScene(const World& world, const std::vector<SoftBody>& softBodies,
                         const std::vector<SpringJoint>& springJoints) {
    nlohmann::json j;
    j["gravity"] = vec2ToJson(world.gravity);

    nlohmann::json bodiesJson = nlohmann::json::array();
    for (auto& bp : world.bodies()) {
        const Body& b = *bp;
        nlohmann::json bj;
        bj["name"] = b.name;
        bj["type"] = static_cast<int>(b.type);
        bj["position"] = vec2ToJson(b.position);
        bj["rotation"] = b.rotation;
        bj["velocity"] = vec2ToJson(b.velocity);
        bj["angularVelocity"] = b.angularVelocity;
        bj["density"] = b.density;
        bj["restitution"] = b.restitution;
        bj["friction"] = b.friction;
        bj["linearDamping"] = b.linearDamping;
        bj["angularDamping"] = b.angularDamping;
        bj["gravityScale"] = b.gravityScale;
        bj["fixedRotation"] = b.fixedRotation;
        bj["isSensor"] = b.isSensor;
        bj["colorR"] = b.colorR;
        bj["colorG"] = b.colorG;
        bj["colorB"] = b.colorB;
        bj["texturePath"] = b.texturePath;
        bj["matterKind"] = static_cast<int>(b.matterKind);
        bj["isUiElement"] = b.isUiElement;
        bj["uiKind"] = b.uiKind;
        bj["uiText"] = b.uiText;
        bj["uiValue"] = b.uiValue;
        bj["uiMin"] = b.uiMin;
        bj["uiMax"] = b.uiMax;
        bj["uiHideText"] = b.uiHideText;
        bj["uiOverlayX"] = b.uiOverlayX;
        bj["uiOverlayY"] = b.uiOverlayY;
        bj["scriptPath"] = b.scriptPath;

        nlohmann::json shapeJson;
        if (b.shape.type == ShapeType::Circle) {
            shapeJson["type"] = "circle";
            shapeJson["radius"] = b.shape.radius;
        } else {
            shapeJson["type"] = "polygon";
            nlohmann::json verts = nlohmann::json::array();
            for (auto& v : b.shape.vertices) verts.push_back(nlohmann::json::array({v.x, v.y}));
            shapeJson["vertices"] = verts;
        }
        bj["shape"] = shapeJson;
        bodiesJson.push_back(std::move(bj));
    }
    j["bodies"] = bodiesJson;

    nlohmann::json softBodiesJson = nlohmann::json::array();
    for (auto& sb : softBodies) {
        nlohmann::json sbj;
        nlohmann::json namesJson = nlohmann::json::array();
        for (Body* p : sb.particles) namesJson.push_back(p ? p->name : std::string());
        sbj["particleNames"] = namesJson;

        nlohmann::json springsJson = nlohmann::json::array();
        for (auto& s : sb.springs) {
            springsJson.push_back({{"a", s.a},
                                    {"b", s.b},
                                    {"restLength", s.restLength},
                                    {"stiffness", s.stiffness},
                                    {"damping", s.damping}});
        }
        sbj["springs"] = springsJson;
        softBodiesJson.push_back(std::move(sbj));
    }
    j["softBodies"] = softBodiesJson;

    nlohmann::json jointsJson = nlohmann::json::array();
    for (auto& sj : springJoints) {
        if (!sj.a || !sj.b) continue;
        jointsJson.push_back({{"nameA", sj.a->name},
                               {"nameB", sj.b->name},
                               {"restLength", sj.restLength},
                               {"stiffness", sj.stiffness},
                               {"damping", sj.damping}});
    }
    j["springJoints"] = jointsJson;

    return j;
}

bool loadScene(const nlohmann::json& j, World& world, std::vector<SoftBody>& outSoftBodies,
              std::vector<SpringJoint>& outSpringJoints,
              const std::function<void(Body&, const std::string&)>& attachScript) {
    if (!j.is_object()) return false;

    try {
        world.clear();
        outSoftBodies.clear();
        outSpringJoints.clear();

        if (j.contains("gravity")) world.gravity = vec2FromJson(j["gravity"]);

        for (auto& bj : j.value("bodies", nlohmann::json::array())) {
            auto shapeJson = bj.at("shape");
            ShapeData shape;
            if (shapeJson.value("type", "circle") == "circle") {
                shape = ShapeData::MakeCircle(shapeJson.value("radius", 0.5f));
            } else {
                std::vector<Vec2> verts;
                for (auto& v : shapeJson.at("vertices")) {
                    verts.emplace_back(v.at(0).get<float>(), v.at(1).get<float>());
                }
                shape = ShapeData::MakePolygon(std::move(verts));
            }

            Vec2 pos = vec2FromJson(bj.value("position", nlohmann::json::object()));
            auto type = static_cast<BodyType>(bj.value("type", static_cast<int>(BodyType::Dynamic)));
            Body* b = world.createBody(shape, pos, type);

            b->rotation = bj.value("rotation", 0.0f);
            b->velocity = vec2FromJson(bj.value("velocity", nlohmann::json::object()));
            b->angularVelocity = bj.value("angularVelocity", 0.0f);
            b->density = bj.value("density", 1.0f);
            b->fixedRotation = bj.value("fixedRotation", false);
            b->computeMass();
            b->restitution = bj.value("restitution", 0.3f);
            b->friction = bj.value("friction", 0.3f);
            b->linearDamping = bj.value("linearDamping", 0.0f);
            b->angularDamping = bj.value("angularDamping", 0.0f);
            b->gravityScale = bj.value("gravityScale", 1.0f);
            b->isSensor = bj.value("isSensor", false);
            // Falls back to whatever createBody() just defaulted by type
            // (not a hardcoded value) so a scene saved before this field
            // existed loads with the same look it always had, rather than
            // every body silently turning the same fallback color.
            b->colorR = bj.value("colorR", b->colorR);
            b->colorG = bj.value("colorG", b->colorG);
            b->colorB = bj.value("colorB", b->colorB);
            b->texturePath = bj.value("texturePath", std::string());
            b->matterKind =
                static_cast<MatterKind>(bj.value("matterKind", static_cast<int>(MatterKind::Rigidbody)));
            b->isUiElement = bj.value("isUiElement", false);
            b->uiKind = bj.value("uiKind", std::string("button"));
            b->uiText = bj.value("uiText", std::string());
            b->uiValue = bj.value("uiValue", 0.0f);
            b->uiMin = bj.value("uiMin", 0.0f);
            b->uiMax = bj.value("uiMax", 1.0f);
            b->uiHideText = bj.value("uiHideText", false);
            b->uiOverlayX = bj.value("uiOverlayX", -1.0f);
            b->uiOverlayY = bj.value("uiOverlayY", -1.0f);
            b->name = bj.value("name", std::string());

            std::string scriptPath = bj.value("scriptPath", std::string());
            if (!scriptPath.empty() && attachScript) attachScript(*b, scriptPath);
        }

        for (auto& sbj : j.value("softBodies", nlohmann::json::array())) {
            SoftBody sb;
            bool ok = true;
            for (auto& nameJson : sbj.value("particleNames", nlohmann::json::array())) {
                Body* p = world.findByName(nameJson.get<std::string>());
                if (!p) {
                    ok = false;
                    break;
                }
                sb.particles.push_back(p);
            }
            if (!ok) continue; // referenced a particle name that no longer exists -- skip rather than crash
            for (auto& sj : sbj.value("springs", nlohmann::json::array())) {
                sb.springs.push_back({sj.value("a", 0), sj.value("b", 0), sj.value("restLength", 0.0f),
                                      sj.value("stiffness", 0.0f), sj.value("damping", 0.0f)});
            }
            outSoftBodies.push_back(std::move(sb));
        }

        for (auto& jj : j.value("springJoints", nlohmann::json::array())) {
            Body* a = world.findByName(jj.value("nameA", std::string()));
            Body* b = world.findByName(jj.value("nameB", std::string()));
            if (!a || !b) continue;
            outSpringJoints.push_back(
                {a, b, jj.value("restLength", 0.0f), jj.value("stiffness", 0.0f), jj.value("damping", 0.0f)});
        }
    } catch (const std::exception&) {
        world.clear();
        outSoftBodies.clear();
        outSpringJoints.clear();
        return false;
    }

    return true;
}

} // namespace p2d::app
