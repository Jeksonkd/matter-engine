cmake_minimum_required(VERSION 3.16)
project(matter_engine LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Static MSVC runtime (/MT, not the default /MD) so a built .exe doesn't
# need the Visual C++ Redistributable installed on the machine that runs
# it -- matches the "static to keep the binary self-contained" choice
# already made for SFML below, just for the C++ runtime itself. No-op on
# every other compiler/platform.
if(MSVC)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

option(P2D_BUILD_APP "Build the SFML+ImGui visual editor" ON)
option(P2D_BUILD_TESTS "Build headless engine smoke tests" ON)

include(FetchContent)
set(FETCHCONTENT_QUIET ON)

# ---------------------------------------------------------------------------
# Lua 5.4 (plain upstream source, no CMake support -- we build it ourselves)
# ---------------------------------------------------------------------------
FetchContent_Declare(
  lua_src
  GIT_REPOSITORY https://github.com/lua/lua.git
  GIT_TAG v5.4.7
  GIT_SHALLOW ON
)
FetchContent_MakeAvailable(lua_src)

file(GLOB P2D_LUA_SOURCES ${lua_src_SOURCE_DIR}/*.c)
list(REMOVE_ITEM P2D_LUA_SOURCES
  ${lua_src_SOURCE_DIR}/lua.c
  ${lua_src_SOURCE_DIR}/luac.c
  ${lua_src_SOURCE_DIR}/onelua.c
  ${lua_src_SOURCE_DIR}/ltests.c
)
add_library(lua_static STATIC ${P2D_LUA_SOURCES})
target_include_directories(lua_static PUBLIC ${lua_src_SOURCE_DIR})
# Platform macro controls which optional POSIX/Windows bits Lua's own
# luaconf.h enables (dynamic library loading for require(), mainly) -- both
# LUA_USE_LINUX and "m" (the math library) are Linux/POSIX-specific and
# don't exist as such on Windows (math functions are already in the MSVC
# CRT, and there's no separate libm to link against).
target_link_libraries(lua_static PUBLIC ${CMAKE_DL_LIBS})
if(WIN32)
  target_compile_definitions(lua_static PRIVATE LUA_USE_WINDOWS)
elseif(UNIX)
  target_compile_definitions(lua_static PRIVATE LUA_USE_LINUX)
  target_link_libraries(lua_static PUBLIC m)
endif()

# ---------------------------------------------------------------------------
# sol2 (header-only Lua binding library) -- used purely as headers, its own
# CMake project is heavier than we need (it can build its own Lua etc.)
# ---------------------------------------------------------------------------
FetchContent_Declare(
  sol2_src
  GIT_REPOSITORY https://github.com/ThePhD/sol2.git
  GIT_TAG v3.5.0
  GIT_SHALLOW ON
)
FetchContent_Populate(sol2_src)

add_library(sol2 INTERFACE)
target_include_directories(sol2 INTERFACE ${sol2_src_SOURCE_DIR}/include)
target_link_libraries(sol2 INTERFACE lua_static)
target_compile_features(sol2 INTERFACE cxx_std_17)

# ---------------------------------------------------------------------------
# nlohmann/json (header-only) -- scene serialization (app/src/ScenePersistence
# .cpp), so a project's bodies/scripts/soft bodies/spring connections survive
# closing and reopening the app (autosave + load-on-open). Fetched
# unconditionally (not just under P2D_BUILD_APP) so the headless
# scene_persistence_test can link it too, the same way project_paths_test
# links ProjectPaths.cpp without needing the rest of the app.
# ---------------------------------------------------------------------------
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
  GIT_SHALLOW ON
)
FetchContent_MakeAvailable(json)

add_subdirectory(engine)
add_subdirectory(scripting)

if(P2D_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(P2D_BUILD_APP)
  # -------------------------------------------------------------------------
  # SFML (graphics/window/system only; static to keep the binary self-contained)
  # -------------------------------------------------------------------------
  set(SFML_BUILD_AUDIO OFF CACHE BOOL "" FORCE)
  set(SFML_BUILD_NETWORK OFF CACHE BOOL "" FORCE)
  set(SFML_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(SFML_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

  FetchContent_Declare(
    SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 2.6.1
    GIT_SHALLOW ON
  )
  FetchContent_MakeAvailable(SFML)

  # -------------------------------------------------------------------------
  # Dear ImGui (no upstream CMake support, fetch source only)
  # -------------------------------------------------------------------------
  FetchContent_Declare(
    imgui_src
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.90.5-docking
    GIT_SHALLOW ON
  )
  FetchContent_MakeAvailable(imgui_src)

  # -------------------------------------------------------------------------
  # ImGui-SFML glue layer
  # -------------------------------------------------------------------------
  set(IMGUI_DIR ${imgui_src_SOURCE_DIR} CACHE PATH "" FORCE)
  set(IMGUI_SFML_FIND_SFML OFF CACHE BOOL "" FORCE)
  set(IMGUI_SFML_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(IMGUI_SFML_BUILD_TESTING OFF CACHE BOOL "" FORCE)

  FetchContent_Declare(
    imgui-sfml
    GIT_REPOSITORY https://github.com/SFML/imgui-sfml.git
    GIT_TAG v2.6.1
    GIT_SHALLOW ON
  )
  FetchContent_MakeAvailable(imgui-sfml)

  # -------------------------------------------------------------------------
  # ImGuiColorTextEdit: the built-in script code editor (line numbers, Lua
  # syntax highlighting, scrolling). No upstream CMake support, no releases/
  # tags -- pinned to an exact commit for reproducibility. Only depends on
  # the public imgui.h API (no imgui_internal.h), so it's low-risk against
  # our ImGui version.
  # -------------------------------------------------------------------------
  FetchContent_Declare(
    imguicolortextedit
    GIT_REPOSITORY https://github.com/BalazsJako/ImGuiColorTextEdit.git
    GIT_TAG ca2f9f1462e3b60e56351bc466acda448c5ea50d
    # This ImGui version moved PushItemFlag()/ImGuiItemFlags_NoTabStop out of
    # the public header (now internal-only); the modern public replacement
    # is PushTabStop()/PopTabStop(). Patch the one call site rather than
    # vendor a modified copy of the library.
    # A cmake -P script rather than an inline sed command: PATCH_COMMAND
    # arguments go through CMake's list machinery, which mangles any shell
    # command whose text contains literal semicolons (as this one does).
    PATCH_COMMAND ${CMAKE_COMMAND} -DTARGET_FILE=TextEditor.cpp
      -P ${CMAKE_SOURCE_DIR}/cmake/patch_texteditor.cmake
  )
  FetchContent_Populate(imguicolortextedit)

  add_library(imgui_texteditor STATIC ${imguicolortextedit_SOURCE_DIR}/TextEditor.cpp)
  target_include_directories(imgui_texteditor PUBLIC ${imguicolortextedit_SOURCE_DIR})
  target_link_libraries(imgui_texteditor PUBLIC ImGui-SFML::ImGui-SFML)
  target_compile_features(imgui_texteditor PUBLIC cxx_std_17)

  # -------------------------------------------------------------------------
  # tinyfiledialogs: native OS "Open File" dialog for the FileSystem panel's
  # Upload Texture button (zenity/kdialog/... on Linux, the real Win32 file
  # picker on Windows, no GTK/Qt dependency). Upstream is SourceForge, not
  # GitHub, and has no tagged releases -- pinned to an exact commit (current
  # master as of this writing) for reproducibility, same reasoning as
  # ImGuiColorTextEdit above.
  # -------------------------------------------------------------------------
  FetchContent_Declare(
    tinyfiledialogs_src
    GIT_REPOSITORY https://git.code.sf.net/p/tinyfiledialogs/code
    GIT_TAG 58f4a241a9afb2996e2beefc938fc8d5002da601
  )
  FetchContent_Populate(tinyfiledialogs_src)

  add_library(tinyfiledialogs STATIC ${tinyfiledialogs_src_SOURCE_DIR}/tinyfiledialogs.c)
  target_include_directories(tinyfiledialogs PUBLIC ${tinyfiledialogs_src_SOURCE_DIR})
  if(WIN32)
    # Not linked by tinyfiledialogs.c itself (no #pragma comment(lib, ...)
    # in it) -- these back its native IFileOpenDialog/GetOpenFileNameW path.
    target_link_libraries(tinyfiledialogs PUBLIC comdlg32 ole32 uuid shell32)
  endif()

  add_subdirectory(app)
endif()
