#ifndef SIMULATION_BODYSYSTEM_HPP
#define SIMULATION_BODYSYSTEM_HPP

#include <cassert>
#include <vector>
#include <iostream>

#include "rigid_body.hpp"
#include "force/force.hpp"

class RigidBody;

class Force;

class BodySystem {
public:
    ~BodySystem();

    /** List of forces, pointers due to abstract class implementations. */
    std::vector<Force *> forces;

    /** List of bodies, intentionally not pointers for easy copying. */
    std::vector<RigidBody> bodies;
};

#endif //SIMULATION_BODYSYSTEM_HPP
