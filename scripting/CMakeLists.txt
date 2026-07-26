add_library(p2d_scripting STATIC
  src/ScriptEngine.cpp
)

target_include_directories(p2d_scripting PUBLIC include)
target_link_libraries(p2d_scripting PUBLIC p2d_engine sol2)
target_compile_features(p2d_scripting PUBLIC cxx_std_17)
