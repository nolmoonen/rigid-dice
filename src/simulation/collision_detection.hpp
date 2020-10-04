#ifndef SIMULATION_COLLISION_DETECTION_HPP
#define SIMULATION_COLLISION_DETECTION_HPP

#include <cstdint>

#include <glm/gtx/orthonormalize.hpp>

#include "body_system.hpp"
#include "rigid_body.hpp"
#include "collision.hpp"
#include "math.hpp"
#include "collision_state.hpp"

class Contact;

/**
 * High-level routines that use the ones in {Collision} to determine the state of the simulation
 * as well as prevent collisions. */
namespace CollisionDetection {
    /** Returns true if at least one intersection. */
    bool intersect(BodySystem *body_system);

    /** Returns the state of the simulation. */
    CollisionState find_collision_state(BodySystem *body_system);

    /** Finds all contacts. */
    std::vector<Contact *> find_all_contacts(BodySystem *body_system);
}

#endif //SIMULATION_COLLISION_DETECTION_HPP
