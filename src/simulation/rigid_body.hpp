#ifndef SIMULATION_RIGID_BODY_HPP
#define SIMULATION_RIGID_BODY_HPP

#include <vector>
#include <cassert>
#include <cfloat>

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "../shape/shape.hpp"

class Contact;

/** Contains all constant variables of a rigid body. */
class ShapeWithMass {
protected:
    double inv_mass;                    // inverse mass
    glm::dmat3 inv_moment_of_inertia{}; // inferred from constructor
    Shape const *body{};
    glm::dmat3 scale{};
public:
    ShapeWithMass(double inv_mass, double size_x, double size_y, double size_z);

    double get_inv_mass() const;

    glm::dmat3 get_inv_moment_of_inertia() const;

    Shape const *get_body() const;

    const glm::dmat3 &get_scale() const;

    /** Functions to forward the request to {body}. */
    const std::vector<std::vector<std::pair<uint32_t, glm::vec2>>> &get_faces() const;

    const std::vector<glm::dvec3> &get_model_vertices() const;

    const std::vector<std::pair<uint32_t, uint32_t>> &get_edges() const;
};

/** Creates a box with appropriate moment of inertia. */
class Box : public ShapeWithMass {
public:
    Box(double inv_mass, double size_x, double size_y, double size_z);
};

/** Creates an icosahedron with appropriate moment of inertia. */
class Icosahedron : public ShapeWithMass {
public:
    Icosahedron(double inv_mass, double size_x, double size_y, double size_z);
};

class RigidBody {
public:
    /** Constant quantities. */
    // explicitly not destructed, should be done manually
    // as to allow multiple bodies to use the same shape
    ShapeWithMass const *shape;

    /** Quantities. */
    glm::dvec3 x;      // position
    glm::dvec3 p;      // linear momentum

    glm::dmat3 a;      // rotation matrix
    glm::dvec3 l;      // angular momentum

    /** Auxiliary quantities. */
    glm::dvec3 v;      // linear velocity
    glm::dmat3 i_inv;  // inverse world space inertia tensor
    glm::dvec3 omega;  // angular velocity

    /** Computed quantities. */
    glm::dvec3 force;  // force
    glm::dvec3 torque; // torque

    RigidBody(glm::dvec3 p_x, ShapeWithMass const *p_shape_with_mass);

    RigidBody(glm::dvec3 p_x, glm::dmat3 p_a, ShapeWithMass const *p_shape_with_mass);

    RigidBody(glm::dvec3 p_x, glm::dmat3 p_a, glm::dvec3 p_p, ShapeWithMass const *p_shape_with_mass);

    RigidBody(glm::dvec3 p_x, glm::dmat3 p_a, glm::dvec3 p_p, glm::dvec3 p_l, ShapeWithMass const *p_shape_with_mass);

    /** Get the non-unitized normal of face {face_i}. */
    glm::dvec3 get_non_unit_normal(uint32_t face_i) const;

    /** Get world space vertex from point. */
    glm::dvec3 convert_to_world_space(glm::dvec3 point) const;

    /** Get world space vertex from index. */
    glm::dvec3 get_world_space_vertex(uint32_t vertex_i) const;

    /** Get world space vertex from index, with an offset of {offset} units. */
    glm::dvec3 get_world_space_vertex(uint32_t vertex_i, double offset, glm::dvec3 dir) const;

    void clear_force_and_torque();

    /** Return the velocity of a point on a rigid body. */
    glm::dvec3 point_velocity(glm::dvec3 point) const;

    /** Return the acceleration of a point on a rigid body. */
    glm::dvec3 point_acceleration(glm::dvec3 point) const;
};

#endif //SIMULATION_RIGID_BODY_HPP
