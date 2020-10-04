#include "gravity_force.hpp"

const glm::dvec3 GravityForce::G = glm::dvec3(0., -9.81, 0.);

GravityForce::GravityForce(BodySystem *p_body_system) : Force(p_body_system)
{}

glm::dvec3 GravityForce::calculate_gravity(RigidBody *rigid_body)
{
    // no gravity is applied if mass is infinite
    if (rigid_body->shape->get_inv_mass() == 0.) {
        return glm::dvec3(0.);
    }

    return G / rigid_body->shape->get_inv_mass();
}

void GravityForce::apply_force_and_torque()
{
    for (auto &rigidBody : body_system->bodies) {
        // NB: gravity does not apply torque since it exerts force on the center of mass
        rigidBody.force += calculate_gravity(&rigidBody);
    }
}