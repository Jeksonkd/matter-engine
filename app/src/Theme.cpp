#include "Theme.hpp"

#include <imgui.h>

namespace p2d::app {

void ApplyDarkTheme() {
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();

    style.WindowRounding = 8.0f;
    style.ChildRounding = 6.0f;
    style.FrameRounding = 6.0f;
    style.PopupRounding = 6.0f;
    style.ScrollbarRounding = 8.0f;
    style.GrabRounding = 6.0f;
    style.TabRounding = 6.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.PopupBorderSize = 1.0f;
    style.WindowPadding = ImVec2(10.0f, 10.0f);
    style.FramePadding = ImVec2(8.0f, 4.0f);
    style.ItemSpacing = ImVec2(8.0f, 6.0f);
    style.IndentSpacing = 18.0f;
    style.ScrollbarSize = 14.0f;
    style.GrabMinSize = 10.0f;

    ImVec4* c = style.Colors;
    const ImVec4 bg(0.114f, 0.106f, 0.145f, 1.00f);         // deep purple-charcoal
    const ImVec4 bgAlt(0.145f, 0.137f, 0.180f, 1.00f);      // slightly lighter panel bg
    const ImVec4 panel(0.180f, 0.169f, 0.220f, 1.00f);      // frame/widget bg
    const ImVec4 panelHover(0.235f, 0.220f, 0.290f, 1.00f); // hovered frame bg
    const ImVec4 border(0.290f, 0.271f, 0.349f, 0.55f);
    const ImVec4 text(0.906f, 0.898f, 0.918f, 1.00f);
    const ImVec4 textDim(0.560f, 0.545f, 0.596f, 1.00f);
    const ImVec4 accent(0.408f, 0.486f, 0.906f, 1.00f); // blue-violet, KDE-Breeze-ish
    const ImVec4 accentHov(0.478f, 0.557f, 0.965f, 1.00f);
    const ImVec4 accentAct(0.337f, 0.412f, 0.831f, 1.00f);

    c[ImGuiCol_Text] = text;
    c[ImGuiCol_TextDisabled] = textDim;
    c[ImGuiCol_WindowBg] = bg;
    c[ImGuiCol_ChildBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    c[ImGuiCol_PopupBg] = bgAlt;
    c[ImGuiCol_Border] = border;
    c[ImGuiCol_BorderShadow] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    c[ImGuiCol_FrameBg] = panel;
    c[ImGuiCol_FrameBgHovered] = panelHover;
    c[ImGuiCol_FrameBgActive] = panelHover;
    c[ImGuiCol_TitleBg] = bgAlt;
    c[ImGuiCol_TitleBgActive] = panel;
    c[ImGuiCol_TitleBgCollapsed] = bgAlt;
    c[ImGuiCol_MenuBarBg] = bgAlt;
    c[ImGuiCol_ScrollbarBg] = bg;
    c[ImGuiCol_ScrollbarGrab] = panel;
    c[ImGuiCol_ScrollbarGrabHovered] = panelHover;
    c[ImGuiCol_ScrollbarGrabActive] = accent;
    c[ImGuiCol_CheckMark] = accent;
    c[ImGuiCol_SliderGrab] = accent;
    c[ImGuiCol_SliderGrabActive] = accentHov;
    c[ImGuiCol_Button] = panel;
    c[ImGuiCol_ButtonHovered] = accentHov;
    c[ImGuiCol_ButtonActive] = accentAct;
    c[ImGuiCol_Header] = panel;
    c[ImGuiCol_HeaderHovered] = accentHov;
    c[ImGuiCol_HeaderActive] = accentAct;
    c[ImGuiCol_Separator] = border;
    c[ImGuiCol_SeparatorHovered] = accent;
    c[ImGuiCol_SeparatorActive] = accentHov;
    c[ImGuiCol_ResizeGrip] = panelHover;
    c[ImGuiCol_ResizeGripHovered] = accent;
    c[ImGuiCol_ResizeGripActive] = accentHov;
    c[ImGuiCol_Tab] = bgAlt;
    c[ImGuiCol_TabHovered] = accentHov;
    c[ImGuiCol_TabActive] = panel;
    c[ImGuiCol_TabUnfocused] = bgAlt;
    c[ImGuiCol_TabUnfocusedActive] = panel;
    c[ImGuiCol_DockingPreview] = ImVec4(accent.x, accent.y, accent.z, 0.35f);
    c[ImGuiCol_DockingEmptyBg] = bg;
    c[ImGuiCol_PlotLines] = accent;
    c[ImGuiCol_PlotLinesHovered] = accentHov;
    c[ImGuiCol_PlotHistogram] = accent;
    c[ImGuiCol_PlotHistogramHovered] = accentHov;
    c[ImGuiCol_TextSelectedBg] = ImVec4(accent.x, accent.y, accent.z, 0.35f);
    c[ImGuiCol_DragDropTarget] = ImVec4(accent.x, accent.y, accent.z, 0.90f);
    c[ImGuiCol_NavHighlight] = accent;
}

} // namespace p2d::app
