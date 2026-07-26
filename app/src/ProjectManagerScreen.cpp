#include "ProjectManagerScreen.hpp"
#include "ProjectBundle.hpp"

#include <imgui.h>

#include <cstring>

namespace p2d::app {

ProjectManagerScreen::ProjectManagerScreen() {
    root_ = projectsRootPath();
    refresh();

    // First-ever launch: seed one default project so the list isn't empty
    // and there's something to explore with a single click, while still
    // presenting the full project browser rather than skipping straight
    // into an editor.
    if (projects_.empty()) {
        std::filesystem::path defaultProject = root_ / "MyProject";
        ensureProjectScaffold(defaultProject / "scripts", nullptr);
        refresh();
    }
}

void ProjectManagerScreen::refresh() {
    projects_ = listProjects(root_);
    if (selectedIndex_ >= static_cast<int>(projects_.size())) selectedIndex_ = -1;
}

std::optional<std::filesystem::path> ProjectManagerScreen::drawUI() {
    std::optional<std::filesystem::path> openRequest;

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                              ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus;
    ImGui::Begin("##ProjectManager", nullptr, flags);

    ImGui::TextUnformatted("Matter Engine Projects");
    ImGui::TextDisabled("%s", root_.string().c_str());
    ImGui::Separator();

    if (ImGui::Button("New Project")) {
        newProjectNameBuffer_[0] = '\0';
        showNewProjectPopup_ = true;
    }

    bool hasSelection = selectedIndex_ >= 0 && selectedIndex_ < static_cast<int>(projects_.size());
    ImGui::SameLine();
    ImGui::BeginDisabled(!hasSelection);
    if (ImGui::Button("Open")) {
        openRequest = projects_[static_cast<size_t>(selectedIndex_)].path;
    }
    ImGui::SameLine();
    if (ImGui::Button("Delete")) {
        pendingDeleteIndex_ = selectedIndex_;
        showDeleteConfirmPopup_ = true;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Refresh")) refresh();
    ImGui::SameLine();
    ImGui::Dummy(ImVec2(16.0f, 0.0f));
    ImGui::SameLine();
    ImGui::BeginDisabled(!hasSelection);
    if (ImGui::Button("Save Project As...")) {
        const std::string& name = projects_[static_cast<size_t>(selectedIndex_)].name;
        std::filesystem::path suggested = root_.parent_path() / (name + ".p2dproj");
        std::snprintf(saveBundlePathBuffer_, sizeof(saveBundlePathBuffer_), "%s", suggested.string().c_str());
        showSaveBundlePopup_ = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Bundles the selected project's scene + scripts into a single file, so it\n"
                           "can be shared or backed up without the whole ~/Physics2DProjects folder.");
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Open Project File...")) {
        openBundlePathBuffer_[0] = '\0';
        openBundleNameBuffer_[0] = '\0';
        showOpenBundlePopup_ = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Unpacks a project bundle file (see Save Project As...) into a new project\n"
                           "here, listed below like any other project once done.");
    ImGui::SameLine();
    if (ImGui::Button("Documentation")) showDocumentation_ = true;

    if (!lastBundleMessage_.empty()) {
        ImGui::TextColored(lastBundleMessageIsError_ ? ImVec4(1.0f, 0.5f, 0.5f, 1.0f) : ImVec4(0.6f, 0.9f, 0.6f, 1.0f),
                            "%s", lastBundleMessage_.c_str());
    }

    ImGui::Separator();

    if (projects_.empty()) {
        ImGui::TextDisabled("No projects yet -- click New Project to create one.");
    } else {
        ImGui::BeginChild("ProjectList", ImVec2(0.0f, 0.0f), true);
        for (size_t i = 0; i < projects_.size(); ++i) {
            bool isSelected = (selectedIndex_ == static_cast<int>(i));
            std::string label = projects_[i].name + "##project" + std::to_string(i);
            if (ImGui::Selectable(label.c_str(), isSelected, ImGuiSelectableFlags_AllowDoubleClick)) {
                selectedIndex_ = static_cast<int>(i);
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) openRequest = projects_[i].path;
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", projects_[i].path.string().c_str());
        }
        ImGui::EndChild();
    }

    // --- New Project popup -------------------------------------------------
    if (showNewProjectPopup_) {
        ImGui::OpenPopup("New Project##pm");
        showNewProjectPopup_ = false;
    }
    if (ImGui::BeginPopupModal("New Project##pm", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Project name:");
        ImGui::InputText("##projectname", newProjectNameBuffer_, sizeof(newProjectNameBuffer_));
        if (ImGui::Button("Create")) {
            std::string name = newProjectNameBuffer_;
            if (!name.empty()) {
                std::filesystem::path projectPath = root_ / name;
                std::error_code existsEc;
                if (!std::filesystem::exists(projectPath, existsEc)) {
                    ensureProjectScaffold(projectPath / "scripts", nullptr);
                    refresh();
                    openRequest = projectPath;
                }
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    // --- Delete confirmation popup -----------------------------------------
    if (showDeleteConfirmPopup_) {
        ImGui::OpenPopup("Delete Project?##pm");
        showDeleteConfirmPopup_ = false;
    }
    if (ImGui::BeginPopupModal("Delete Project?##pm", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (pendingDeleteIndex_ >= 0 && pendingDeleteIndex_ < static_cast<int>(projects_.size())) {
            ImGui::Text("Permanently delete \"%s\" and all its files?",
                        projects_[static_cast<size_t>(pendingDeleteIndex_)].name.c_str());
            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "This cannot be undone.");
            if (ImGui::Button("Delete")) {
                std::error_code ec;
                std::filesystem::remove_all(projects_[static_cast<size_t>(pendingDeleteIndex_)].path, ec);
                selectedIndex_ = -1;
                refresh();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
        }
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    // --- Save Project As... popup -------------------------------------------
    if (showSaveBundlePopup_) {
        ImGui::OpenPopup("Save Project As##pm");
        showSaveBundlePopup_ = false;
    }
    if (ImGui::BeginPopupModal("Save Project As##pm", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Bundle file path:");
        ImGui::SetNextItemWidth(450.0f);
        ImGui::InputText("##savebundlepath", saveBundlePathBuffer_, sizeof(saveBundlePathBuffer_));
        if (hasSelection) {
            if (ImGui::Button("Save")) {
                auto log = [this](const std::string& msg, bool isError) {
                    lastBundleMessage_ = msg;
                    lastBundleMessageIsError_ = isError;
                };
                saveProjectToFile(projects_[static_cast<size_t>(selectedIndex_)].path, saveBundlePathBuffer_, log);
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
        }
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    // --- Open Project File... popup -----------------------------------------
    if (showOpenBundlePopup_) {
        ImGui::OpenPopup("Open Project File##pm");
        showOpenBundlePopup_ = false;
    }
    if (ImGui::BeginPopupModal("Open Project File##pm", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Bundle file path:");
        ImGui::SetNextItemWidth(450.0f);
        ImGui::InputText("##openbundlepath", openBundlePathBuffer_, sizeof(openBundlePathBuffer_));
        ImGui::TextUnformatted("New project name:");
        ImGui::SetNextItemWidth(300.0f);
        ImGui::InputText("##openbundlename", openBundleNameBuffer_, sizeof(openBundleNameBuffer_));
        if (ImGui::Button("Open")) {
            if (openBundlePathBuffer_[0] != '\0' && openBundleNameBuffer_[0] != '\0') {
                auto log = [this](const std::string& msg, bool isError) {
                    lastBundleMessage_ = msg;
                    lastBundleMessageIsError_ = isError;
                };
                std::filesystem::path destDir = root_ / openBundleNameBuffer_;
                if (loadProjectFromFile(openBundlePathBuffer_, destDir, log)) {
                    refresh();
                    for (size_t i = 0; i < projects_.size(); ++i) {
                        if (projects_[i].path == destDir) {
                            selectedIndex_ = static_cast<int>(i);
                            break;
                        }
                    }
                }
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    drawDocumentation();

    ImGui::End();
    return openRequest;
}

void ProjectManagerScreen::drawDocumentation() {
    if (!showDocumentation_) return;

    ImGui::SetNextWindowSize(ImVec2(700.0f, 560.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Documentation", &showDocumentation_)) {
        ImGui::End();
        return;
    }

    ImGui::BeginChild("DocScroll");

    ImGui::TextUnformatted("Matter Engine -- Quick Reference");
    ImGui::Separator();

    if (ImGui::CollapsingHeader("Editor Basics", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::BulletText("Play / Pause / Stop / Step control the simulation, top-left of the Toolbar.");
        ImGui::BulletText("Reset Scene restores the scene to how it looked the last time you weren't\n"
                           "running (or were paused) -- it undoes whatever happened during Play.");
        ImGui::BulletText("Time Scale speeds up or slows down simulated time; Gravity Y changes world gravity.");
        ImGui::BulletText("Per-body realism/speed trade-offs live on the body itself now (Inspector's\n"
                           "\"Matter Kind\": Rigidbody/Matter/OptiMatter), not as a single scene-wide switch --\n"
                           "see \"Body::matterKind\" in the README.");
    }

    if (ImGui::CollapsingHeader("Viewport Tools", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::BulletText("Select / Move: click to select, drag to move. Teleports while editing,\n"
                           "pulls with a spring while playing.");
        ImGui::BulletText("Spawn: pick a Shape (Circle, Box, Triangle, Pentagon, Hexagon, Soft Body\n"
                           "Ring, Soft Body Jelly, or Cloth) from the dropdown, then click in the\n"
                           "viewport to place it.");
        ImGui::BulletText("Resize: click a body, then drag to resize it (radius or half-extents).\n"
                           "Mass is recomputed automatically.");
        ImGui::BulletText("Rotate: click a body, then drag around it -- it rotates to face the mouse.");
        ImGui::BulletText("Box Select: drag a rectangle to select every body inside it. Dragging any\n"
                           "selected body then moves the whole group together; the Inspector shows a\n"
                           "\"Delete All Selected\" button while a group is selected.");
        ImGui::BulletText("Right or middle drag pans the camera; the mouse wheel zooms.");
        ImGui::BulletText("Ctrl+C / Ctrl+V copy/paste the selected body (or the whole Box Select\n"
                           "group, keeping its layout) -- pasted copies land offset from the\n"
                           "original, cascading further with each repeated Ctrl+V.");
    }

    if (ImGui::CollapsingHeader("Soft Bodies")) {
        ImGui::TextWrapped(
            "A soft body is a group of small ordinary circle bodies connected by springs -- each one "
            "collides with everything else exactly like any other body, but Hooke's-law springs "
            "between them hold the group's shape instead of it being fully rigid.");
        ImGui::BulletText("Ring: hollow perimeter + cross-brace springs. Can fold/collapse under enough force.");
        ImGui::BulletText("Jelly: a Ring plus a center hub particle spoked to every ring particle -- "
                           "resists collapsing and holds its volume much better than a hollow ring.");
        ImGui::BulletText("Cloth: a grid of particles with structural + shear springs. Pin Top makes\n"
                           "the top row Static (immovable) so it hangs/drapes instead of falling freely.");
        ImGui::BulletText("This is a lightweight mass-spring approximation, not a full deformable-\n"
                           "body solver (no FEM/position-based dynamics) -- it won't exactly preserve\n"
                           "volume or guarantee no self-intersection, but it's cheap and reuses the\n"
                           "whole existing rigid-body pipeline unchanged.");
        ImGui::BulletText("Soft bodies survive Play/Stop/Reset Scene along with the rest of the scene.");
    }

    if (ImGui::CollapsingHeader("Connections (Springs)", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextWrapped(
            "The general version of a soft body's springs: ties any two EXISTING objects together "
            "(any shape, any BodyType) without replacing or otherwise modifying either one -- unlike "
            "Ring/Jelly/Cloth, which are made of their own auto-generated particles.");
        ImGui::BulletText("Inspector > Connections: pick another body, set Stiffness/Damping, Connect.\n"
                           "Rest length is whatever distance apart they currently are. Existing\n"
                           "connections on the selected body are listed with a Disconnect button each.");
        ImGui::BulletText("Box Select > Connect Selected with Springs: wires the whole selected group\n"
                           "together (a chain plus cross-braces, same bracing idea as a Ring).");
        ImGui::BulletText("Connections survive Play/Stop/Reset Scene along with the rest of the scene.");
    }

    if (ImGui::CollapsingHeader("Hierarchy: Create", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextWrapped(
            "One Kind dropdown + Create button makes either an ordinary physics shape or a UI "
            "Element -- which one depends entirely on which Kind is picked.");
        ImGui::BulletText("Circle/Box/Triangle/Pentagon/Hexagon: an ordinary body spawned at world\n"
                           "(0, 0) -- NOT a UI Element. Use the \"...\" button next to Create to pick\n"
                           "its Matter Kind (Rigidbody/Matter/OptiMatter), Body Type (Static/\n"
                           "Kinematic/Dynamic), and Quantity (1 up to 1000, spread in a small grid\n"
                           "around the origin rather than stacked exactly on top of each other).");
        ImGui::BulletText("Button/Text/Checkbox/Slider/Input Field/Panel: a UI Element with NO script\n"
                           "attached at all -- it renders itself directly and is immediately usable.\n"
                           "Attach a script later (Inspector > Script path) only if you need custom\n"
                           "behavior beyond the default widget. The \"...\" options don't apply to\n"
                           "these Kinds (there's nothing to batch or give a Matter/Body Type to).");
        ImGui::BulletText("Input Field: a text box bound to UI Text -- typing into the live widget\n"
                           "and editing the Inspector's field both land in the same place.");
        ImGui::BulletText("Panel: a background/decoration rectangle, sized via the Inspector's\n"
                           "\"Panel Size\" field (or an Appearance > Texture image instead of a flat\n"
                           "color fill). The only Kind that keeps the old translucent window look --\n"
                           "every other Kind is fully opaque now (see Appearance & Textures below).");
        ImGui::BulletText("FileSystem's New Script popup is a separate, script-focused path (still\n"
                           "Button/Text/Checkbox/Slider only) for when you DO want a generated\n"
                           "template to build custom behavior on top of from the start.");
    }

    if (ImGui::CollapsingHeader("FileSystem Panel", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::BulletText("New Folder creates a folder; New Script opens a popup with a name field\n"
                           "AND a Kind dropdown -- Plain Script (the default) scaffolds an ordinary\n"
                           "on_start/on_update stub with no host body. Button/Text/Checkbox/Slider\n"
                           "instead write that Kind's on_gui template AND create a live, invisible\n"
                           "host body already running it, selected immediately. (This is the\n"
                           "script-writing path -- the Hierarchy panel's Create button below makes a\n"
                           "UI Element with NO script at all, including Input Field/Panel kinds this\n"
                           "popup doesn't offer.)");
        ImGui::BulletText("A script the popup wrote for a Kind other than Plain is tagged with a\n"
                           "hidden marker comment -- shown in this panel as a '[UI: Button]'-style\n"
                           "suffix on the file. Dragging a marked script onto ANY body's Inspector\n"
                           "auto-tags that body as a UI Element of the matching Kind too, even if it\n"
                           "wasn't created through this popup.");
        ImGui::BulletText("Image files (.png/.jpg/.jpeg/.bmp/.gif/.tga) are shown in a distinct\n"
                           "color in the tree so they stand out from scripts at a glance.");
        ImGui::BulletText("Upload Texture copies an image file from elsewhere on disk into the\n"
                           "project (type/paste its full path) -- a reliable alternative to dragging\n"
                           "one in from the desktop file manager, which isn't available/consistent on\n"
                           "every desktop environment.");
        ImGui::BulletText("Click a file or folder to select it. Ctrl+C copies it, Ctrl+V pastes into\n"
                           "the selected folder (or next to the selected file), and Delete removes it.\n"
                           "Copy / Paste / Delete buttons above the tree do the same with the mouse.");
        ImGui::BulletText("Double-click a .lua file to edit it; drag it onto the Inspector's Script\n"
                           "path field to attach it to the selected body.");
        ImGui::BulletText("Drag an image file in from your desktop file manager to import it into the\n"
                           "project (if that works reliably on your desktop -- otherwise use Upload\n"
                           "Texture above); drag it back out onto the Inspector's Texture field or the\n"
                           "Settings panel's Background Texture field to use it (see Appearance below).");
    }

    if (ImGui::CollapsingHeader("Appearance & Textures", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::BulletText("Inspector > Appearance > Color: this body's flat fill, fixed once set --\n"
                           "independent of its BodyType or whether it's asleep.");
        ImGui::BulletText("Inspector > Appearance > Texture: an image drawn over the body instead of\n"
                           "Color. Type/paste a path or drag an image in from FileSystem. Mapped onto\n"
                           "the body's own local shape, so it stays fixed under rotation and camera\n"
                           "pan/zoom instead of sliding. Clear Texture removes it.");
        ImGui::BulletText("Settings > Background: a Color (always visible) plus an optional Texture,\n"
                           "same drag-and-drop as the Inspector's field. Tiled repeats the image every\n"
                           "couple world units and pans/zooms with the scene; unchecked stretches it\n"
                           "to exactly cover the current view instead.");
        ImGui::BulletText("A UI Element's host body uses its own Texture the same way -- set it and\n"
                           "the widget (native or scripted) switches to an image instead of a plain\n"
                           "one. Hide Text (Inspector, next to UI Text) then drops the label once the\n"
                           "icon speaks for itself -- not offered for Text/Input Field/Panel.");
        ImGui::BulletText("UI Element windows over the Viewport are fully opaque now, except Panel\n"
                           "(deliberately kept translucent, since it's meant as a background/\n"
                           "decoration element sitting behind other Elements you drag over it).");
        ImGui::BulletText("Both body textures and the background persist with the rest of the scene.");
    }

    if (ImGui::CollapsingHeader("Code Editor")) {
        ImGui::BulletText("Docked as a tab alongside the Viewport -- only one of the two is visible at once.");
        ImGui::BulletText("Autosaves about 2 seconds after you stop typing; Save writes immediately.");
        ImGui::BulletText("Saving re-attaches the script if it's already running on the selected body.");
    }

    if (ImGui::CollapsingHeader("Scripting API (Lua)", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextUnformatted("Every script may define:");
        ImGui::BulletText("on_start(body) -- called once when the script is attached.");
        ImGui::BulletText("on_update(body, dt) -- called every physics step while attached.");
        ImGui::BulletText("on_gui(body) -- called every rendered frame; draw ui.* widgets here.\n"
                           "Shown in the Script UI tab (one stacked list) AND as its own small,\n"
                           "draggable window over the Viewport -- grab it anywhere (no title bar) and\n"
                           "move it; where you drop it is remembered per-body and saved with the scene.\n"
                           "The drag can't leave the viewport image, and only works while Editing --\n"
                           "during Play the same window still works, just can't be repositioned.");
        ImGui::Separator();
        ImGui::TextWrapped(
            "body fields: position (Vec2), rotation, velocity (Vec2), angular_velocity, restitution, "
            "friction, density (settable, recomputes mass), mass (read), inertia (read), name, "
            "type (BodyType -- settable, recomputes mass), matter_kind (MatterKind.Rigidbody/.Matter/"
            ".OptiMatter -- solver fidelity dial, doesn't change any other field), radius (settable ONLY "
            "while already a circle -- silently ignored otherwise, use set_circle() to convert), "
            "color_r/color_g/color_b (0-255), linear_damping, angular_damping, gravity_scale, is_sensor, "
            "is_awake (read), fixed_rotation, texture_path, ui_text, ui_value, ui_min, ui_max, "
            "ui_hide_text.");
        ImGui::TextWrapped(
            "body methods: set_velocity(x, y), set_position(x, y), set_color(r, g, b), "
            "set_circle(radius)/set_box(hw, hh)/set_polygon(sides, circumradius) (rebuild this body's "
            "shape from scratch -- unlike .radius, these convert BETWEEN shape kinds too, e.g. box -> "
            "circle -- and recompute mass), apply_force(Vec2), apply_force_at_point(Vec2 force, Vec2 "
            "worldPoint), apply_impulse(Vec2), apply_torque(t), wake().");
        ImGui::Separator();
        ImGui::TextWrapped(
            "Matter (a point-mass particle, NOT a rigidbody -- no rotation/torque/inertia at all): "
            "position, velocity, restitution, friction, linear_damping, gravity_scale, name, matter_kind "
            "(MatterKind.Matter/.OptiMatter only), radius (settable, recomputes mass), texture_path, "
            "color_r/g/b, density (settable, recomputes mass), mass (read), is_awake (read), "
            "set_color(r, g, b), apply_force(v), apply_impulse(v), wake(), set_velocity(x, y), "
            "set_position(x, y).");
        ImGui::Separator();
        ImGui::TextWrapped(
            "world: gravity, create_circle(x, y, radius, BodyType), create_box(x, y, hw, hh, BodyType), "
            "create_polygon(x, y, sides, circumradius, BodyType) (any regular N-gon), find_body(name) "
            "(first match), count(), remove_body(body), bodies() (every body, 1-indexed, for ipairs -- "
            "use .radius > 0 to tell a circle from a polygon), create_matter(x, y, radius, MatterKind), "
            "find_matter(name), matter_count(), remove_matter(m), matter() (every Matter particle, "
            "1-indexed).");
        ImGui::TextWrapped(
            "world tracking helpers (names aren't required to be unique, unlike find_body()): "
            "count_by_name(name), count_by_type(BodyType), count_by_matter_kind(MatterKind) (all return "
            "an int); find_bodies_by_name(name), find_bodies_by_type(BodyType), "
            "find_bodies_by_matter_kind(MatterKind) (all return a 1-indexed table of every match); plus "
            "the Matter-particle equivalents count_matter_by_kind(MatterKind)/"
            "find_matter_by_kind(MatterKind).");
        ImGui::TextWrapped(
            "Globals: BodyType.Static / .Kinematic / .Dynamic, MatterKind.Rigidbody / .Matter / "
            ".OptiMatter, Vec2.new(x, y), attach_script(body, path), create_spring(bodyA, bodyB, "
            "stiffness, damping) (ties two EXISTING bodies together), print(...).");
        ImGui::Separator();
        ImGui::TextWrapped(
            "soft_body table: create_ring(x, y, radius, segments), create_jelly(x, y, radius, segments), "
            "create_cloth(x, y, cols, rows, spacing, pin_top) -- each returns every particle created as "
            "a 1-indexed table (particles[1] is the hub for a jelly).");
        ImGui::Separator();
        ImGui::TextWrapped(
            "ui table (only meaningful inside on_gui): ui.text(s), ui.button(label), "
            "ui.checkbox(label, value), ui.slider_float(label, value, min, max), ui.image(path, size), "
            "ui.image_button(path, id, size) (falls back to a plain button if path fails to load), "
            "ui.same_line(), ui.separator(), ui.is_item_hovered(), ui.is_item_clicked(), "
            "ui.is_item_active() (generic interaction detection on whatever widget was drawn last).");
    }

    if (ImGui::CollapsingHeader("Project Structure")) {
        ImGui::BulletText("Each project lives under ~/Physics2DProjects/<name>/ with its own scripts/\n"
                           "folder.");
        ImGui::BulletText("New projects start with a small starter scene: a floor, two walls, 5 balls\n"
                           "already dropped in, and a \"Spawn Ball\" button wired to a real script\n"
                           "(scripts/spawn_balls.lua, written into the new project the first time it's\n"
                           "created -- a real, editable file from the start, not baked into the editor).");
        ImGui::BulletText("The scene autosaves to scene.json every few seconds while editing, and loads\n"
                           "back automatically next time you open the project -- bodies, their\n"
                           "properties, attached scripts, soft bodies, and spring connections are all\n"
                           "exactly where you left them. Delete scene.json (and scripts/spawn_balls.lua,\n"
                           "if you want that regenerated too) to go back to the starter scene.");
        ImGui::BulletText("Leaving a project (Projects button or closing the app) also forces one\n"
                           "final save immediately, so an edit made right before leaving is never\n"
                           "lost waiting on the next autosave tick.");
        ImGui::BulletText("Save Project As... / Open Project File... (Project Manager toolbar) bundle a\n"
                           "whole project -- scene.json plus every file under scripts/ -- into a single\n"
                           ".p2dproj file and back, so a project can be shared or backed up without the\n"
                           "whole ~/Physics2DProjects folder. There's no native OS file picker linked\n"
                           "into this build, so both are typed-path popups rather than a real dialog.");
    }

    ImGui::EndChild();
    ImGui::End();
}

} // namespace p2d::app
