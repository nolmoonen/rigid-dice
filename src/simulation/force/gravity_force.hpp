#ifndef SIMULATION_GRAVITY_FORCE_HPP
#define SIMULATION_GRAVITY_FORCE_HPP

#include "force.hpp"

class GravityForce : public Force {
private:
    /** Gravity constant. */
    static const glm::dvec3 G;
public:
    explicit GravityForce(BodySystem *p_body_system);

    glm::dvec3 calculate_gravity(RigidBody *rigid_body);

    void apply_force_and_torque() override;
};

#endif //SIMULATION_GRAVITY_FORCE_HPP
