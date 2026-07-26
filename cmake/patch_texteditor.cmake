# Patches ImGuiColorTextEdit's TextEditor.cpp for compatibility with newer
# Dear ImGui versions, where PushItemFlag()/ImGuiItemFlags_NoTabStop moved
# out of the public header (imgui_internal.h only now); PushTabStop()/
# PopTabStop() are the modern public replacement with equivalent semantics
# for this call (NoTabStop=false == tab-stop allowed == PushTabStop(true)).
#
# Implemented as a `cmake -P` script (not an inline PATCH_COMMAND shell
# command) specifically to avoid CMake's command-argument-list semicolon
# splitting, which mangles any shell command whose text contains literal
# semicolons.

if(NOT DEFINED TARGET_FILE)
  message(FATAL_ERROR "patch_texteditor.cmake: TARGET_FILE not set")
endif()

file(READ "${TARGET_FILE}" _contents)

string(REPLACE
  "ImGui::PushItemFlag(ImGuiItemFlags_NoTabStop, false);"
  "ImGui::PushTabStop(true);"
  _contents "${_contents}")

string(REPLACE
  "ImGui::PopItemFlag();"
  "ImGui::PopTabStop();"
  _contents "${_contents}")

file(WRITE "${TARGET_FILE}" "${_contents}")
