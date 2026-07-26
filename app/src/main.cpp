#include "EditorApp.hpp"
#include "FileDropWatcher.hpp"
#include "ProjectManagerScreen.hpp"
#include "Theme.hpp"

#include <SFML/Graphics.hpp>
#include <imgui-SFML.h>
#include <imgui.h>

#include <memory>

int main() {
    sf::RenderWindow window(sf::VideoMode(1440, 900), "Matter Engine");
    window.setFramerateLimit(60);

    if (!ImGui::SFML::Init(window)) {
        return 1;
    }

    // Watches for files (e.g. images) dragged in from the desktop file
    // manager -- see FileDropWatcher's doc comment for why this needs its
    // own direct X11 handling rather than an sf::Event. Constructed once,
    // right after the native window exists, and polled every frame below;
    // a no-op everywhere it isn't supported (non-Linux, or no X server
    // reachable at all).
    p2d::app::FileDropWatcher fileDropWatcher(window.getSystemHandle());

    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    p2d::app::ApplyDarkTheme();

    // Two "screens" sharing one window: the project manager (a Godot-like
    // project list/create/delete browser) and the editor for whichever
    // project is currently open. Only one is alive at a time -- opening a
    // project constructs an EditorApp; clicking "Projects" in its toolbar
    // destroys it and drops back to the browser.
    p2d::app::ProjectManagerScreen projectManager;
    std::unique_ptr<p2d::app::EditorApp> editor;

    sf::Clock deltaClock;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);

            if (event.type == sf::Event::Closed) {
                window.close();
            } else if (event.type == sf::Event::Resized) {
                sf::FloatRect visibleArea(0.0f, 0.0f, static_cast<float>(event.size.width),
                                           static_cast<float>(event.size.height));
                window.setView(sf::View(visibleArea));
            }

            if (editor) editor->handleEvent(event);
        }

        // Polled unconditionally, whether or not a project is currently
        // open: XDND is a synchronous handshake with the dragging
        // application (see FileDropWatcher), so this needs to keep draining
        // and responding to it every frame regardless -- otherwise a drop
        // made while sitting on the Project Manager screen would leave the
        // OS file manager waiting on a response that never comes. With no
        // project open there's nowhere to import into, so the result is
        // just discarded in that case.
        auto dropped = fileDropWatcher.poll();
        if (editor && !dropped.empty()) editor->importDroppedFiles(dropped);

        sf::Time dt = deltaClock.restart();
        ImGui::SFML::Update(window, dt);

        if (editor) {
            editor->update(dt.asSeconds());
            editor->renderViewport();
            editor->drawUI();
            if (editor->wantsReturnToProjectManager()) {
                editor.reset();
            }
        } else {
            if (auto chosen = projectManager.drawUI()) {
                editor = std::make_unique<p2d::app::EditorApp>(*chosen);
            }
        }

        window.clear(sf::Color(15, 15, 18));
        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
    return 0;
}
