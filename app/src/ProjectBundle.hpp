#pragma once

#include <filesystem>
#include <functional>
#include <string>

namespace p2d::app {

// Bundles an entire project (its scene.json plus every file under
// scripts/, recursively) into a single self-contained JSON file -- so a
// project can be shared, backed up, or moved as one file instead of the
// whole ~/Physics2DProjects/<name>/ directory tree, and the Project
// Manager's "Open Project File..." can rebuild a real project directory
// from it. .lua files are embedded as plain JSON strings (human-readable,
// diffable); anything else (images) is base64-encoded.

// Writes `projectDir` (must contain scene.json; scripts/ may be empty) to
// `destFile` as one JSON bundle. Returns false (and calls `log` with an
// error) on any failure -- missing scene.json, an unreadable file under
// scripts/, or a write failure on `destFile`.
bool saveProjectToFile(const std::filesystem::path& projectDir, const std::filesystem::path& destFile,
                       const std::function<void(const std::string&, bool)>& log);

// Reads a bundle written by saveProjectToFile() and recreates it as a new
// project directory `destProjectDir` (its parent must exist; the directory
// itself must NOT already exist -- this never overwrites an existing
// project). Returns false (and calls `log`) on any failure: not valid JSON,
// not a recognized bundle, or `destProjectDir` already exists.
bool loadProjectFromFile(const std::filesystem::path& srcFile, const std::filesystem::path& destProjectDir,
                         const std::function<void(const std::string&, bool)>& log);

} // namespace p2d::app
