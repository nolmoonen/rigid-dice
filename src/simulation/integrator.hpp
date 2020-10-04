#ifndef SIMULATION_INTEGRATOR_HPP
#define SIMULATION_INTEGRATOR_HPP

#include <glm/gtx/orthonormalize.hpp>

#include "body_system.hpp"

namespace Integrator {
    /** Performs midpoint integration. */
    void midpoint(BodySystem *body_system, double dt);

    /** Performs fourth order Runge Kutta integration. */
    void runge_kutta_4(BodySystem *body_system, double dt);

    /** Performs Euler integration. */
    void integrate(BodySystem *body_system, double dt);

    void clear_forces(BodySystem *body_system);

    void apply_forces(BodySystem *body_system);

    glm::dmat3 star(glm::dvec3 a);
}

#endif //SIMULATION_INTEGRATOR_HPP
