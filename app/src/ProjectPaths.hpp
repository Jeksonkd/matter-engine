#pragma once

#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace p2d::app {

// The root folder containing every project, e.g. ~/Physics2DProjects (or
// ./Physics2DProjects if HOME isn't set). Each subdirectory directly under
// this is one project; a project's own scripts live in <project>/scripts.
// Kept separate from the engine's own bundled example scripts so a user's
// edits never collide with the source tree.
std::filesystem::path projectsRootPath();

struct ProjectInfo {
    std::string name;        // the directory's name, e.g. "MyProject"
    std::filesystem::path path; // full path to the project directory
};

// Lists every immediate subdirectory of `root` as a project, sorted by
// name. Returns an empty list (not an error) if `root` doesn't exist yet --
// callers create it lazily via ensureProjectScaffold when a project is
// actually made.
std::vector<ProjectInfo> listProjects(const std::filesystem::path& root);

// Ensures `scriptsDir` exists, creating it (and its parents) if needed. A
// new project's scripts folder starts empty -- no seeded example scripts
// (bacteria, spawners, ...) and no pre-attached scripts in the default
// scene, so a fresh project is a genuinely blank sandbox rather than a demo.
// Deliberately has no SFML/ImGui dependency so it can be exercised
// headlessly. `log`, if set, receives human-readable progress/error
// messages (isError as the second argument).
void ensureProjectScaffold(const std::filesystem::path& scriptsDir,
                           const std::function<void(const std::string&, bool)>& log = nullptr);

} // namespace p2d::app
