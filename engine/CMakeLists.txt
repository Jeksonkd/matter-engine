add_library(p2d_engine STATIC
  src/Shape.cpp
  src/Body.cpp
  src/Matter.cpp
  src/Collision.cpp
  src/World.cpp
  src/SoftBody.cpp
  src/SpringJoint.cpp
)

target_include_directories(p2d_engine PUBLIC include)
target_compile_features(p2d_engine PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(p2d_engine PRIVATE /W4)
else()
  target_compile_options(p2d_engine PRIVATE -Wall -Wextra)
endif()
