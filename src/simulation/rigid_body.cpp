#include "rigid_body.hpp"

double ShapeWithMass::get_inv_mass() const
{
    return inv_mass;
}

glm::dmat3 ShapeWithMass::get_inv_moment_of_inertia() const
{
    return inv_moment_of_inertia;
}

Shape const *ShapeWithMass::get_body() const
{
    return body;
}

const glm::dmat3 &ShapeWithMass::get_scale() const
{
    return scale;
}

const std::vector<std::vector<std::pair<uint32_t, glm::vec2>>> &ShapeWithMass::get_faces() const
{
    return body->get_faces();
}

const std::vector<glm::dvec3> &ShapeWithMass::get_model_vertices() const
{
    return body->get_vertices();
}

const std::vector<std::pair<uint32_t, uint32_t>> &ShapeWithMass::get_edges() const
{
    return body->get_edges();
}

ShapeWithMass::ShapeWithMass(
        double inv_mass, double size_x, double size_y, double size_z
) :
        inv_mass(inv_mass)
{
    scale = glm::dmat3(glm::scale(glm::identity<glm::dmat4>(), glm::dvec3(size_x, size_y, size_z)));
}

Box::Box(
        double p_inv_mass, double size_x, double size_y, double size_z
) :
        ShapeWithMass(p_inv_mass, size_x, size_y, size_z)
{
    // NB: this will be 0 if inv_mass is 0, this is as intended
    inv_moment_of_inertia = glm::identity<glm::dmat3>();
    inv_moment_of_inertia[0][0] = (12. * inv_mass) / (size_y * size_y + size_z * size_z);
    inv_moment_of_inertia[1][1] = (12. * inv_mass) / (size_x * size_x + size_z * size_z);
    inv_moment_of_inertia[2][2] = (12. * inv_mass) / (size_x * size_x + size_y * size_y);
    body = &Shape::CUBE;
}

Icosahedron::Icosahedron(
        double p_inv_mass, double size_x, double size_y, double size_z
) :
        ShapeWithMass(p_inv_mass, size_x, size_y, size_z)
{
    // NB: this will be 0 if inv_mass is 0, this is as intended
    inv_moment_of_inertia = glm::identity<glm::dmat3>();
    double phi = (1. + sqrt(5.)) / 2.;
    inv_moment_of_inertia[0][0] = (10. * inv_mass) / (size_x * size_x * phi);
    inv_moment_of_inertia[1][1] = (10. * inv_mass) / (size_y * size_y * phi);
    inv_moment_of_inertia[2][2] = (10. * inv_mass) / (size_z * size_z * phi);
    body = &Shape::ICOSAHEDRON;
}

RigidBody::RigidBody(
        glm::dvec3 p_x, ShapeWithMass const *p_shape_with_mass
) :
        shape(p_shape_with_mass), x(p_x), p(glm::dvec3()), a(glm::identity<glm::dmat3>()), l(glm::dvec3()),
        force(glm::dvec3()), torque(glm::dvec3())
{
    // compute initial auxiliary variables
    v = p * shape->get_inv_mass();
    i_inv = a * shape->get_inv_moment_of_inertia() * glm::transpose(a);
    omega = i_inv * l;
}

RigidBody::RigidBody(
        glm::dvec3 p_x, glm::dmat3 p_a, ShapeWithMass const *p_shape_with_mass
) :
        RigidBody(p_x, p_shape_with_mass)
{
    this->a = p_a;
}

RigidBody::RigidBody(
        glm::dvec3 p_x, glm::dmat3 p_a, glm::dvec3 p_p, ShapeWithMass const *p_shape_with_mass
) :
        RigidBody(p_x, p_a, p_shape_with_mass)
{
    this->p = p_p;
}

RigidBody::RigidBody(
        glm::dvec3 p_x, glm::dmat3 p_a, glm::dvec3 p_p, glm::dvec3 p_l, ShapeWithMass const *p_shape_with_mass
) :
        RigidBody(p_x, p_a, p_p, p_shape_with_mass)
{
    this->l = p_l;
}

glm::dvec3 RigidBody::get_non_unit_normal(uint32_t face_i) const
{
    // apply rotation of the rigid body
    return a * shape->get_body()->get_non_unit_normal(face_i);
}

glm::dvec3 RigidBody::convert_to_world_space(glm::dvec3 point) const
{
    return a * (shape->get_scale() * point) + x;
}

glm::dvec3 RigidBody::get_world_space_vertex(uint32_t vertex_i) const
{
    return convert_to_world_space(shape->get_body()->get_vertices()[vertex_i]);
}

glm::dvec3 RigidBody::get_world_space_vertex(uint32_t vertex_i, double offset, glm::dvec3 dir) const
{
    glm::dvec3 point = shape->get_scale() * shape->get_body()->get_vertices()[vertex_i];
    return a * point + x + offset * glm::normalize(dir);
}

void RigidBody::clear_force_and_torque()
{
    this->force = glm::dvec3(0.);
    this->torque = glm::dvec3(0.);
}

glm::dvec3 RigidBody::point_velocity(glm::dvec3 point) const
{
    return v + glm::cross(omega, point - x);
}

glm::dvec3 RigidBody::point_acceleration(glm::dvec3 point) const
{
    glm::dvec3 r = point - x;
    glm::dvec3 l_dot = torque;
    glm::dvec3 omega_dot = i_inv * (glm::cross(l, omega) + l_dot);
    glm::dvec3 v_dot = force * shape->get_inv_mass();

    return glm::cross(omega_dot, r) + glm::cross(omega, glm::cross(omega, r)) + v_dot;
}
