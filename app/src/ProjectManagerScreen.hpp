#pragma once

#include "ProjectPaths.hpp"

#include <filesystem>
#include <optional>
#include <vector>

namespace p2d::app {

// A Godot-like project browser shown before the editor: lists every project
// under projectsRootPath(), lets you create new ones (seeded from the
// bundled example scripts), open one, or delete one.
class ProjectManagerScreen {
public:
    ProjectManagerScreen();

    // Draws the screen for this frame. Returns the path of a project the
    // user chose to open (new or existing), or nullopt if still browsing.
    std::optional<std::filesystem::path> drawUI();

private:
    std::filesystem::path root_;
    std::vector<ProjectInfo> projects_;
    int selectedIndex_ = -1;

    bool showNewProjectPopup_ = false;
    char newProjectNameBuffer_[128] = {};

    bool showDeleteConfirmPopup_ = false;
    int pendingDeleteIndex_ = -1;

    // "Save Project As..." (bundles the selected project's scene.json +
    // scripts/ tree into one file, see ProjectBundle.hpp) and "Open Project
    // File..." (the reverse -- unpacks a bundle into a new project
    // directory under root_) -- there's no native OS file picker linked
    // into this project (see ProjectPaths.hpp's comment trail), so both are
    // typed-path popups rather than a real "Save As"/"Open" dialog.
    bool showSaveBundlePopup_ = false;
    char saveBundlePathBuffer_[512] = {};
    bool showOpenBundlePopup_ = false;
    char openBundlePathBuffer_[512] = {};
    char openBundleNameBuffer_[128] = {};
    std::string lastBundleMessage_;
    bool lastBundleMessageIsError_ = false;

    bool showDocumentation_ = false;

    void refresh();
    void drawDocumentation();
};

} // namespace p2d::app
