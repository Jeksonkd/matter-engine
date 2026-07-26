add_executable(p2d_editor
  src/main.cpp
  src/EditorApp.cpp
  src/Renderer.cpp
  src/Theme.cpp
  src/ProjectPaths.cpp
  src/ProjectManagerScreen.cpp
  src/ProjectBundle.cpp
  src/ScenePersistence.cpp
  src/FileDropWatcher.cpp
)

target_link_libraries(p2d_editor PRIVATE
  p2d_engine
  p2d_scripting
  sfml-graphics
  sfml-window
  sfml-system
  ImGui-SFML::ImGui-SFML
  imgui_texteditor
  nlohmann_json::nlohmann_json
)

# FileDropWatcher.cpp talks XDND (drag-and-drop from the desktop file
# manager) directly via Xlib on Linux -- SFML itself already links X11
# transitively for its own window/graphics modules there, but that's not
# something this target can rely on being exposed publicly, so link it
# explicitly for our own direct Xlib calls.
if(UNIX AND NOT APPLE)
  find_package(X11 REQUIRED)
  target_link_libraries(p2d_editor PRIVATE X11::X11)
endif()

target_compile_features(p2d_editor PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(p2d_editor PRIVATE /W4)
else()
  target_compile_options(p2d_editor PRIVATE -Wall -Wextra)
endif()
