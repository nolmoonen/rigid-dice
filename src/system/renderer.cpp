#include "renderer.hpp"
#include "opengl.hpp"

Renderer::Renderer(
        Camera *p_camera, ShaderManager *p_shader_manager, TextureManager *p_texture_manager,
        MeshManager *p_mesh_manager
) :
        camera(p_camera), shader_manager(p_shader_manager), texture_manager(p_texture_manager),
        mesh_manager(p_mesh_manager)
{
    Lines::create_coordinate_axes(&coordinate_mesh);
}

void Renderer::render(Engine *engine)
{
    glClear((uint32_t) GL_COLOR_BUFFER_BIT | (uint32_t) GL_DEPTH_BUFFER_BIT);

    if (draw_coordinate) {
        // todo could be a render target if polymorphism between {lines} and {mesh}
        ShaderProgram *program = shader_manager->get(SHADER_LINES);
        ShaderProgram::use_shader_program(program);
        ShaderProgram::set_mat4(program, "modelMatrix", glm::identity<glm::mat4>());
        ShaderProgram::set_mat4(program, "viewMatrix", camera->get_view_matrix());
        ShaderProgram::set_mat4(program, "projectionMatrix", camera->get_proj_matrix());
        Lines::render_lines(&coordinate_mesh);
        ShaderProgram::unuse_shader_program();
    }

    // draw skybox
    ShaderProgram *prog = shader_manager->get(SHADER_DEFAULT); // NB: no shading!
    ShaderProgram::use_shader_program(prog);
    ShaderProgram::set_mat4(prog, "modelMatrix",
                            glm::scale(glm::identity<glm::mat4>(), glm::vec3(200.f))); // scale it somewhat
    ShaderProgram::set_mat4(prog, "viewMatrix", camera->get_view_matrix());
    ShaderProgram::set_mat4(prog, "projectionMatrix", camera->get_proj_matrix());
    Texture::bind_tex(texture_manager->get(TEXTURE_SKYBOX));
    Mesh::render_mesh(mesh_manager->get(MESH_SKYBOX));
    Texture::unbind_tex();
    ShaderProgram::unuse_shader_program();

    // render all bodies
    // todo view matrix and projection matrix are calculated for every render target
    // todo right now, every rigid body is rendered as a cube
    for (auto &body : engine->body_system->bodies) {
        ShaderProgram *program = shader_manager->get(SHADER_PHONG);
        ShaderProgram::use_shader_program(program);

        glm::mat4 model_matrix =
                glm::translate(glm::identity<glm::dmat4>(), body.x) *
                glm::dmat4(body.a) *
                glm::dmat4(body.shape->get_scale());

        ShaderProgram::set_mat4(program, "modelMatrix", model_matrix);
        ShaderProgram::set_mat4(program, "viewMatrix", camera->get_view_matrix());
        ShaderProgram::set_mat4(program, "projectionMatrix", camera->get_proj_matrix());

        // todo phong-shader specific code, should find a way to deal with this generalized
        ShaderProgram::set_vec3(program, "pos_light", glm::vec3(3.f, 3.f, 3.f));
        ShaderProgram::set_vec3(program, "pos_camera", camera->get_camera_position());
        ShaderProgram::set_vec3(program, "color_light", glm::vec3(1.f, 1.f, 1.f));

        // todo small hack to give the proper texture
        Texture *tex;
        if (body.shape->get_inv_mass() == 0.f) {
            tex = texture_manager->get(TEXTURE_WOOD);
        } else {
            tex = texture_manager->get(TEXTURE_DICE);
        }
        Texture::bind_tex(tex);

        uint32_t id = body_pointer_to_id(body.shape->get_body());
        Mesh *mesh = mesh_manager->get(id);
        Mesh::render_mesh(mesh);

        Texture::unbind_tex();

        ShaderProgram::unuse_shader_program();
    }

    // todo debug render all intermediate contacts
    for (uint32_t i = 0; i < engine->prev_contacts.size(); i++) {
        ShaderProgram *contact_program = shader_manager->get(SHADER_DEFAULT);
        ShaderProgram::use_shader_program(contact_program);
        Texture::bind_tex(texture_manager->get(TEXTURE_GRASS));
        glm::mat4 model_matrix =
                glm::translate(glm::identity<glm::dmat4>(), engine->prev_contacts[i]->p) *
                glm::dmat4(engine->prev_contacts[i]->body_b->a) *
                glm::dmat4(glm::scale(glm::identity<glm::dmat4>(), glm::dvec3(.05)));
        ShaderProgram::set_mat4(contact_program, "modelMatrix", model_matrix);
        ShaderProgram::set_mat4(contact_program, "viewMatrix", camera->get_view_matrix());
        ShaderProgram::set_mat4(contact_program, "projectionMatrix", camera->get_proj_matrix());
        Mesh::render_mesh(mesh_manager->get(MESH_CUBE));
        Texture::unbind_tex();
        ShaderProgram::unuse_shader_program();

        Lines line{};
        Lines::create_line(&line, engine->prev_contacts[i]->n, glm::vec3(1.f, 0.f, 0.f));

        ShaderProgram *program = shader_manager->get(SHADER_LINES);
        ShaderProgram::use_shader_program(program);
        glm::mat4 model_matrix_line =
                glm::translate(glm::identity<glm::dmat4>(), engine->prev_contacts[i]->p) *
                glm::dmat4(glm::scale(glm::identity<glm::dmat4>(), glm::dvec3(1.)));
        ShaderProgram::set_mat4(program, "modelMatrix", model_matrix_line);
        ShaderProgram::set_mat4(program, "viewMatrix", camera->get_view_matrix());
        ShaderProgram::set_mat4(program, "projectionMatrix", camera->get_proj_matrix());
        Lines::render_lines(&line);
        if (!engine->prev_contacts[i]->vf) {
            Lines ea{};
            Lines eb{};
            Lines::create_line(&ea, engine->prev_contacts[i]->ea, glm::vec3(0.f, 1.f, 0.f));
            Lines::create_line(&eb, engine->prev_contacts[i]->eb, glm::vec3(0.f, 0.f, 1.f));
            Lines::render_lines(&ea);
            Lines::render_lines(&eb);
            Lines::delete_lines(&ea);
            Lines::delete_lines(&eb);
        }
        ShaderProgram::unuse_shader_program();

        Lines::delete_lines(&line);
    }
}

void Renderer::toggle_draw_coordinate()
{
    draw_coordinate = !draw_coordinate;
}

Renderer::~Renderer()
{
    Lines::delete_lines(&coordinate_mesh);
}
