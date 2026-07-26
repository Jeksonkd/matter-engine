add_executable(p2d_smoke_test smoke_test.cpp)
target_link_libraries(p2d_smoke_test PRIVATE p2d_engine)

add_executable(p2d_properties_test properties_test.cpp)
target_link_libraries(p2d_properties_test PRIVATE p2d_engine)

add_executable(p2d_broadphase_test broadphase_test.cpp)
target_link_libraries(p2d_broadphase_test PRIVATE p2d_engine)

add_executable(p2d_sleeping_test sleeping_test.cpp)
target_link_libraries(p2d_sleeping_test PRIVATE p2d_engine)

add_executable(p2d_stacking_test stacking_test.cpp)
target_link_libraries(p2d_stacking_test PRIVATE p2d_engine)

add_executable(p2d_tunneling_test tunneling_test.cpp)
target_link_libraries(p2d_tunneling_test PRIVATE p2d_engine)

add_executable(p2d_soft_body_test soft_body_test.cpp)
target_link_libraries(p2d_soft_body_test PRIVATE p2d_engine)

add_executable(p2d_spawn_overlap_test spawn_overlap_test.cpp)
target_link_libraries(p2d_spawn_overlap_test PRIVATE p2d_engine)

add_executable(p2d_spring_joint_test spring_joint_test.cpp)
target_link_libraries(p2d_spring_joint_test PRIVATE p2d_engine)

add_executable(p2d_mass_overlap_test mass_overlap_test.cpp)
target_link_libraries(p2d_mass_overlap_test PRIVATE p2d_engine)

add_executable(p2d_friction_jitter_test friction_jitter_test.cpp)
target_link_libraries(p2d_friction_jitter_test PRIVATE p2d_engine)

add_executable(p2d_matter_test matter_test.cpp)
target_link_libraries(p2d_matter_test PRIVATE p2d_engine)

add_executable(p2d_body_matter_kind_test body_matter_kind_test.cpp)
target_link_libraries(p2d_body_matter_kind_test PRIVATE p2d_engine)

# ScenePersistence.{hpp,cpp} deliberately have no SFML/ImGui/ScriptEngine
# dependency (see its header comment), so this test builds/runs independent
# of P2D_BUILD_APP and without needing a display, the same way
# project_paths_test does for ProjectPaths.{hpp,cpp}.
add_executable(p2d_scene_persistence_test
  scene_persistence_test.cpp
  ${CMAKE_SOURCE_DIR}/app/src/ScenePersistence.cpp
)
target_include_directories(p2d_scene_persistence_test PRIVATE ${CMAKE_SOURCE_DIR}/app/src)
target_link_libraries(p2d_scene_persistence_test PRIVATE p2d_engine nlohmann_json::nlohmann_json)

add_executable(p2d_script_test script_test.cpp)
target_link_libraries(p2d_script_test PRIVATE p2d_scripting)
target_compile_definitions(p2d_script_test PRIVATE TESTS_LUA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/lua")

add_executable(p2d_bacteria_test bacteria_test.cpp)
target_link_libraries(p2d_bacteria_test PRIVATE p2d_scripting)
target_compile_definitions(p2d_bacteria_test PRIVATE P2D_SCRIPTS_DIR="${CMAKE_SOURCE_DIR}/scripts")

# ProjectPaths.{hpp,cpp} deliberately have no SFML/ImGui dependency (see
# their header comment), so this test builds/runs independent of
# P2D_BUILD_APP and without needing a display.
add_executable(p2d_project_paths_test
  project_paths_test.cpp
  ${CMAKE_SOURCE_DIR}/app/src/ProjectPaths.cpp
)
target_compile_features(p2d_project_paths_test PRIVATE cxx_std_17)

add_test(NAME smoke_test COMMAND p2d_smoke_test)
add_test(NAME properties_test COMMAND p2d_properties_test)
add_test(NAME broadphase_test COMMAND p2d_broadphase_test)
add_test(NAME sleeping_test COMMAND p2d_sleeping_test)
add_test(NAME stacking_test COMMAND p2d_stacking_test)
add_test(NAME tunneling_test COMMAND p2d_tunneling_test)
add_test(NAME soft_body_test COMMAND p2d_soft_body_test)
add_test(NAME spawn_overlap_test COMMAND p2d_spawn_overlap_test)
add_test(NAME spring_joint_test COMMAND p2d_spring_joint_test)
add_test(NAME mass_overlap_test COMMAND p2d_mass_overlap_test)
add_test(NAME friction_jitter_test COMMAND p2d_friction_jitter_test)
add_test(NAME matter_test COMMAND p2d_matter_test)
add_test(NAME body_matter_kind_test COMMAND p2d_body_matter_kind_test)
add_test(NAME scene_persistence_test COMMAND p2d_scene_persistence_test)
add_test(NAME script_test COMMAND p2d_script_test)
add_test(NAME bacteria_test COMMAND p2d_bacteria_test)
add_test(NAME project_paths_test COMMAND p2d_project_paths_test)
