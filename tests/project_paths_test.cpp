// Headless checks for the project-folder scaffolding logic used by the
// editor's FileSystem panel: creating the scripts/ directory on first run.
// A new project's scripts/ starts empty -- no seeded example scripts, so a
// fresh project is a genuinely blank sandbox, not a demo. Deliberately
// avoids constructing EditorApp itself -- SFML's RenderTexture eagerly
// creates a shared GL context (needs a real display) even before .create()
// is called, so a full EditorApp can't be exercised in a headless sandbox.
// This logic has no such dependency, so it's tested directly instead.

#include "../app/src/ProjectPaths.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace p2d::app;

namespace {

int g_failures = 0;

void check(bool condition, const char* description) {
    if (condition) {
        std::printf("[PASS] %s\n", description);
    } else {
        std::printf("[FAIL] %s\n", description);
        ++g_failures;
    }
}

} // namespace

int main() {
    check(!projectsRootPath().empty(), "projectsRootPath() returns a non-empty path");
    check(projectsRootPath().filename().string() == "Physics2DProjects",
          "projectsRootPath() is a Physics2DProjects workspace folder");

    // --- listProjects() ---------------------------------------------------
    {
        std::filesystem::path scratchRoot = std::filesystem::temp_directory_path() / "p2d_list_projects_test";
        std::error_code ec;
        std::filesystem::remove_all(scratchRoot, ec);

        check(listProjects(scratchRoot).empty(), "listProjects() on a nonexistent root returns an empty list");

        std::filesystem::create_directories(scratchRoot / "Zeta", ec);
        std::filesystem::create_directories(scratchRoot / "Alpha", ec);
        { std::ofstream out(scratchRoot / "not_a_project.txt"); out << "ignore me\n"; }

        auto projects = listProjects(scratchRoot);
        check(projects.size() == 2, "listProjects() lists only directories, not stray files");
        check(projects.size() == 2 && projects[0].name == "Alpha" && projects[1].name == "Zeta",
              "listProjects() is sorted by name");

        std::filesystem::remove_all(scratchRoot, ec);
    }

    // Scratch fixture: a project scripts dir that doesn't exist yet.
    std::filesystem::path scratch = std::filesystem::temp_directory_path() / "p2d_project_paths_test";
    std::error_code ec;
    std::filesystem::remove_all(scratch, ec);

    std::filesystem::path projectScripts = scratch / "project" / "scripts";

    std::vector<std::string> logs;
    auto logFn = [&](const std::string& msg, bool isErr) { logs.push_back((isErr ? "[err] " : "[ok] ") + msg); };

    ensureProjectScaffold(projectScripts, logFn);

    check(std::filesystem::exists(projectScripts), "ensureProjectScaffold creates the scripts directory");
    {
        std::error_code emptyEc;
        bool empty = std::filesystem::is_empty(projectScripts, emptyEc);
        check(!emptyEc && empty, "a new project's scripts/ starts empty -- no seeded example scripts");
    }
    check(!logs.empty(), "ensureProjectScaffold reports what it did via the log callback");

    // Simulate the user creating their own script, then re-running the
    // app: a second call must not touch (or clear) an existing project.
    {
        std::ofstream out(projectScripts / "my_script.lua");
        out << "function on_start(body) end\n";
    }
    ensureProjectScaffold(projectScripts, logFn);
    check(std::filesystem::exists(projectScripts / "my_script.lua"),
          "calling ensureProjectScaffold again does not touch an existing project's files");

    std::filesystem::remove_all(scratch, ec);

    if (g_failures == 0) {
        std::printf("\nAll project-paths tests passed.\n");
        return EXIT_SUCCESS;
    }
    std::printf("\n%d project-paths test check(s) FAILED.\n", g_failures);
    return EXIT_FAILURE;
}
