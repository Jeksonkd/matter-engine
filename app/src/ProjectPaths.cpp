#include "ProjectPaths.hpp"

#include <algorithm>
#include <cstdlib>

namespace p2d::app {

std::filesystem::path projectsRootPath() {
    const char* home = std::getenv("HOME");
    std::filesystem::path base = home ? std::filesystem::path(home) : std::filesystem::current_path();
    return base / "Physics2DProjects";
}

std::vector<ProjectInfo> listProjects(const std::filesystem::path& root) {
    std::vector<ProjectInfo> projects;
    std::error_code ec;
    if (!std::filesystem::exists(root, ec)) return projects;

    for (auto& entry : std::filesystem::directory_iterator(root, ec)) {
        if (ec) break;
        if (entry.is_directory()) {
            projects.push_back({entry.path().filename().string(), entry.path()});
        }
    }
    std::sort(projects.begin(), projects.end(),
              [](const ProjectInfo& a, const ProjectInfo& b) { return a.name < b.name; });
    return projects;
}

void ensureProjectScaffold(const std::filesystem::path& scriptsDir,
                           const std::function<void(const std::string&, bool)>& log) {
    std::error_code ec;
    if (std::filesystem::exists(scriptsDir, ec)) return;

    std::filesystem::create_directories(scriptsDir, ec);
    if (ec) {
        if (log) log("Could not create project folder at " + scriptsDir.string() + ": " + ec.message(), true);
        return;
    }
    if (log) log("Created project folder: " + scriptsDir.parent_path().string(), false);
}

} // namespace p2d::app
