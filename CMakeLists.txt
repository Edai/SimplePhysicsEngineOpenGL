cmake_minimum_required(VERSION 3.9)
project(project1)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenGL)
find_package(GLUT)

include_directories(. include include/GL/ src/)

add_executable(project1
        include/GL/glew.h
        include/GL/glxew.h
        include/GL/wglew.h
        scenes/collision.scene
        scenes/collision_stress.scene
        scenes/damping_test.scene
        scenes/newtons_cradle.scene
        scenes/pool.scene
        scenes/rotation_test.scene
        scenes/spring_rotation.scene
        src/application/application.cpp
        src/application/application.hpp
        src/application/camera_roam.cpp
        src/application/camera_roam.hpp
        src/application/imageio.cpp
        src/application/imageio.hpp
        src/application/opengl.hpp
        src/application/scene_loader.cpp
        src/application/scene_loader.hpp
        src/math/camera.cpp
        src/math/camera.hpp
        src/math/color.cpp
        src/math/color.hpp
        src/math/math.cpp
        src/math/math.hpp
        src/math/matrix.cpp
        src/math/matrix.hpp
        src/math/quaternion.cpp
        src/math/quaternion.hpp
        src/math/vector.cpp
        src/math/vector.hpp
        src/physics/body.hpp
        src/physics/collisions.cpp
        src/physics/collisions.hpp
        src/physics/main.cpp
        src/physics/physics.cpp
        src/physics/physics.hpp
        src/physics/planebody.cpp
        src/physics/planebody.hpp
        src/physics/spherebody.cpp
        src/physics/spherebody.hpp
        src/physics/spring.cpp
        src/physics/spring.hpp
        src/physics/trianglebody.cpp
        src/physics/trianglebody.hpp
        src/scene/geometry.cpp
        src/scene/geometry.hpp
        src/scene/material.cpp
        src/scene/material.hpp
        src/scene/mesh.cpp
        src/scene/mesh.hpp
        src/scene/model.cpp
        src/scene/model.hpp
        src/scene/scene.cpp
        src/scene/scene.hpp
        src/scene/sphere.cpp
        src/scene/sphere.hpp
        src/scene/triangle.cpp
        src/scene/triangle.hpp
        src/tinyxml/tinyxml.cpp
        src/tinyxml/tinyxml.h
        src/tinyxml/tinyxmlerror.cpp
        src/tinyxml/tinyxmlparser.cpp)
