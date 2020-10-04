cmake_minimum_required(VERSION 3.16)
project(rigid-dice)

set(CMAKE_CXX_STANDARD 11)

set(BODY_SOURCES
        src/shape/shape.cpp src/shape/shape.hpp)

set(SIMULATION_SOURCES
        src/simulation/engine.cpp src/simulation/engine.hpp
        src/simulation/rigid_body.cpp src/simulation/rigid_body.hpp
        src/simulation/scene.cpp src/simulation/scene.hpp
        src/simulation/body_system.cpp src/simulation/body_system.hpp
        src/simulation/collision_detection.cpp src/simulation/collision_detection.hpp
        src/simulation/contact.cpp src/simulation/contact.hpp
        src/simulation/force/force.cpp src/simulation/force/force.hpp
        src/simulation/force/drag_force.cpp src/simulation/force/drag_force.hpp
        src/simulation/force/gravity_force.cpp src/simulation/force/gravity_force.hpp
        src/simulation/integrator.cpp src/simulation/integrator.hpp
        src/simulation/collision_handling.cpp src/simulation/collision_handling.hpp
        src/simulation/collision_state.hpp
        src/simulation/math.cpp src/simulation/math.hpp
        src/simulation/contact_derivation.cpp src/simulation/contact_derivation.hpp
        src/simulation/collision.cpp src/simulation/collision.hpp)

set(SOURCES
        src/main.cpp
        src/system/window.cpp src/system/window.hpp
        src/system/input.cpp src/system/input.hpp
        src/system/opengl.cpp src/system/opengl.hpp
        src/system/renderer.cpp src/system/renderer.hpp
        src/system/shader_manager.cpp src/system/shader_manager.hpp
        src/system/camera.cpp src/system/camera.hpp
        src/system/manager.cpp src/system/manager.hpp
        src/system/texture_manager.cpp src/system/texture_manager.hpp
        src/util/nm_log.cpp src/util/nm_log.hpp
        src/system/mesh_manager.cpp src/system/mesh_manager.hpp)

# resource files
add_subdirectory(embedder)
embed(default_frag res/shader/default.frag)
embed(default_vert res/shader/default.vert)
embed(instance_vert res/shader/instance.vert)
embed(phong_frag res/shader/phong.frag)
embed(phong_vert res/shader/phong.vert)
embed(lines_frag res/shader/lines.frag)
embed(lines_vert res/shader/lines.vert)
embed(grass_png res/tex/grass.png)
embed(test_png res/tex/test.png)
embed(dice_png res/tex/dice.png)
embed(skybox_png res/tex/skybox.png)
embed(wood_png res/tex/wood.png)


# build glfw, glad, glm, and gsl before adding compile options
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(${PROJECT_SOURCE_DIR}/external/glfw-3.3.2)
add_subdirectory(${PROJECT_SOURCE_DIR}/external/glad-0.1.33)
add_subdirectory(${PROJECT_SOURCE_DIR}/external/glm-0.9.9.8)
add_subdirectory(${PROJECT_SOURCE_DIR}/external/gsl-2.5.0)

# link MinGW libraries statically
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")

# add extra warnings
add_compile_options(-Wall -Wextra -pedantic)
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES} ${SIMULATION_SOURCES} ${EMBEDDED_RESOURCES} ${BODY_SOURCES})

# link glfw, glad, glm, gsl, stb
target_link_libraries(${CMAKE_PROJECT_NAME} glfw)
target_link_libraries(${CMAKE_PROJECT_NAME} glad)
target_link_libraries(${CMAKE_PROJECT_NAME} glm)
target_link_libraries(${CMAKE_PROJECT_NAME} gsl)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/external/stb)