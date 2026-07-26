#include "ProjectBundle.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>

namespace p2d::app {

namespace {

const char kBase64Chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64Encode(const std::string& data) {
    std::string out;
    out.reserve(((data.size() + 2) / 3) * 4);
    size_t i = 0;
    while (i + 2 < data.size()) {
        unsigned int chunk = (static_cast<unsigned char>(data[i]) << 16) |
                              (static_cast<unsigned char>(data[i + 1]) << 8) | static_cast<unsigned char>(data[i + 2]);
        out.push_back(kBase64Chars[(chunk >> 18) & 0x3F]);
        out.push_back(kBase64Chars[(chunk >> 12) & 0x3F]);
        out.push_back(kBase64Chars[(chunk >> 6) & 0x3F]);
        out.push_back(kBase64Chars[chunk & 0x3F]);
        i += 3;
    }
    size_t remaining = data.size() - i;
    if (remaining == 1) {
        unsigned int chunk = static_cast<unsigned char>(data[i]) << 16;
        out.push_back(kBase64Chars[(chunk >> 18) & 0x3F]);
        out.push_back(kBase64Chars[(chunk >> 12) & 0x3F]);
        out.push_back('=');
        out.push_back('=');
    } else if (remaining == 2) {
        unsigned int chunk = (static_cast<unsigned char>(data[i]) << 16) | (static_cast<unsigned char>(data[i + 1]) << 8);
        out.push_back(kBase64Chars[(chunk >> 18) & 0x3F]);
        out.push_back(kBase64Chars[(chunk >> 12) & 0x3F]);
        out.push_back(kBase64Chars[(chunk >> 6) & 0x3F]);
        out.push_back('=');
    }
    return out;
}

std::string base64Decode(const std::string& encoded) {
    int lookup[256];
    for (int i = 0; i < 256; ++i) lookup[i] = -1;
    for (int i = 0; i < 64; ++i) lookup[static_cast<unsigned char>(kBase64Chars[i])] = i;

    std::string out;
    out.reserve((encoded.size() / 4) * 3);
    unsigned int buffer = 0;
    int bits = 0;
    for (char c : encoded) {
        if (c == '=' || c == '\0') break;
        int val = lookup[static_cast<unsigned char>(c)];
        if (val < 0) continue; // skip whitespace/newlines, just in case
        buffer = (buffer << 6) | static_cast<unsigned int>(val);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<char>((buffer >> bits) & 0xFF));
        }
    }
    return out;
}

std::string readFileRaw(const std::filesystem::path& path, bool& ok) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        ok = false;
        return {};
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    ok = true;
    return ss.str();
}

bool isTextFile(const std::filesystem::path& path) { return path.extension() == ".lua"; }

// Recursively collects every regular file under `dir`, keyed by its path
// relative to `dir` (POSIX-style forward slashes, so the bundle is portable
// across platforms) into `filesJson`.
bool collectFilesInto(const std::filesystem::path& dir, const std::filesystem::path& relativeTo,
                      nlohmann::json& filesJson, const std::function<void(const std::string&, bool)>& log) {
    std::error_code ec;
    for (auto& entry : std::filesystem::recursive_directory_iterator(dir, ec)) {
        if (ec) {
            if (log) log("Could not walk " + dir.string() + ": " + ec.message(), true);
            return false;
        }
        if (!entry.is_regular_file()) continue;

        std::error_code relEc;
        std::string relPath = std::filesystem::relative(entry.path(), relativeTo, relEc).generic_string();
        if (relEc) continue;

        bool ok = false;
        std::string raw = readFileRaw(entry.path(), ok);
        if (!ok) {
            if (log) log("Could not read " + entry.path().string() + ", skipping.", true);
            continue;
        }

        nlohmann::json fileEntry;
        if (isTextFile(entry.path())) {
            fileEntry["encoding"] = "text";
            fileEntry["content"] = raw;
        } else {
            fileEntry["encoding"] = "base64";
            fileEntry["content"] = base64Encode(raw);
        }
        filesJson[relPath] = std::move(fileEntry);
    }
    return true;
}

} // namespace

bool saveProjectToFile(const std::filesystem::path& projectDir, const std::filesystem::path& destFile,
                       const std::function<void(const std::string&, bool)>& log) {
    std::filesystem::path sceneFilePath = projectDir / "scene.json";
    std::error_code existsEc;
    if (!std::filesystem::exists(sceneFilePath, existsEc)) {
        if (log) log("No scene.json in " + projectDir.string() + " -- nothing to bundle yet.", true);
        return false;
    }

    bool sceneOk = false;
    std::string sceneRaw = readFileRaw(sceneFilePath, sceneOk);
    if (!sceneOk) {
        if (log) log("Could not read " + sceneFilePath.string(), true);
        return false;
    }

    nlohmann::json bundle;
    bundle["p2d_project_bundle"] = 1;
    bundle["name"] = projectDir.filename().string();
    try {
        bundle["scene"] = nlohmann::json::parse(sceneRaw);
    } catch (const nlohmann::json::parse_error& e) {
        if (log) log("scene.json is not valid JSON: " + std::string(e.what()), true);
        return false;
    }

    nlohmann::json filesJson = nlohmann::json::object();
    std::filesystem::path scriptsDir = projectDir / "scripts";
    if (std::filesystem::exists(scriptsDir, existsEc)) {
        if (!collectFilesInto(scriptsDir, scriptsDir, filesJson, log)) return false;
    }
    bundle["files"] = std::move(filesJson);

    std::ofstream out(destFile);
    if (!out) {
        if (log) log("Could not write " + destFile.string(), true);
        return false;
    }
    out << bundle.dump(2);
    out.close();
    if (log) log("Saved project bundle to " + destFile.string(), false);
    return true;
}

bool loadProjectFromFile(const std::filesystem::path& srcFile, const std::filesystem::path& destProjectDir,
                         const std::function<void(const std::string&, bool)>& log) {
    std::error_code existsEc;
    if (std::filesystem::exists(destProjectDir, existsEc)) {
        if (log) log("A project already exists at " + destProjectDir.string(), true);
        return false;
    }

    bool srcOk = false;
    std::string raw = readFileRaw(srcFile, srcOk);
    if (!srcOk) {
        if (log) log("Could not read " + srcFile.string(), true);
        return false;
    }

    nlohmann::json bundle;
    try {
        bundle = nlohmann::json::parse(raw);
    } catch (const nlohmann::json::parse_error& e) {
        if (log) log(srcFile.string() + " is not valid JSON: " + std::string(e.what()), true);
        return false;
    }
    if (!bundle.contains("p2d_project_bundle") || !bundle.contains("scene")) {
        if (log) log(srcFile.string() + " doesn't look like a project bundle (missing expected keys).", true);
        return false;
    }

    std::error_code createEc;
    std::filesystem::create_directories(destProjectDir / "scripts", createEc);
    if (createEc) {
        if (log) log("Could not create " + destProjectDir.string() + ": " + createEc.message(), true);
        return false;
    }

    std::ofstream sceneOut(destProjectDir / "scene.json");
    if (!sceneOut) {
        if (log) log("Could not write scene.json into " + destProjectDir.string(), true);
        return false;
    }
    sceneOut << bundle["scene"].dump(2);
    sceneOut.close();

    if (bundle.contains("files") && bundle["files"].is_object()) {
        for (auto& [relPath, fileEntry] : bundle["files"].items()) {
            std::filesystem::path destPath = destProjectDir / "scripts" / std::filesystem::path(relPath);
            std::error_code parentEc;
            std::filesystem::create_directories(destPath.parent_path(), parentEc);

            std::string encoding = fileEntry.value("encoding", "text");
            std::string content = fileEntry.value("content", "");
            std::ofstream fileOut(destPath, std::ios::binary);
            if (!fileOut) {
                if (log) log("Could not write " + destPath.string() + ", skipping.", true);
                continue;
            }
            if (encoding == "base64") {
                std::string decoded = base64Decode(content);
                fileOut.write(decoded.data(), static_cast<std::streamsize>(decoded.size()));
            } else {
                fileOut << content;
            }
        }
    }

    if (log) log("Opened project bundle into " + destProjectDir.string(), false);
    return true;
}

} // namespace p2d::app
