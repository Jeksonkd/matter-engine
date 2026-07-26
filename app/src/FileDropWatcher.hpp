#pragma once

#include <SFML/Window/WindowHandle.hpp>

#include <filesystem>
#include <vector>

namespace p2d::app {

// Watches the editor's native OS window for files dragged in from outside
// the app (e.g. an image dragged in from the desktop file manager) and
// reports their local paths. SFML has no FileDropped event of its own (not
// even in 2.6, the version this project uses), so this talks XDND (the X11
// drag-and-drop protocol) directly against the window's native handle --
// Linux/X11 only, since that's the only platform this project's build/run
// instructions cover (see README); a harmless no-op stub everywhere else.
//
// Opens its OWN Xlib connection to the same window rather than trying to
// share SFML's internal one (which isn't exposed publicly): X11 lets
// multiple client connections select events on the same window, so this
// runs entirely independently of, and never interferes with, SFML's own
// event pump (window.pollEvent()).
class FileDropWatcher {
public:
    // `windowHandle` is the SFML window's native handle
    // (sf::Window::getSystemHandle()), captured once after the window is
    // created.
    explicit FileDropWatcher(sf::WindowHandle windowHandle);
    ~FileDropWatcher();

    FileDropWatcher(const FileDropWatcher&) = delete;
    FileDropWatcher& operator=(const FileDropWatcher&) = delete;

    // Call once per frame (cheap: a non-blocking check for pending X11
    // events). Returns the local file paths dropped since the last call --
    // almost always empty.
    std::vector<std::filesystem::path> poll();

private:
    struct Impl;
    Impl* impl_ = nullptr;
};

} // namespace p2d::app
