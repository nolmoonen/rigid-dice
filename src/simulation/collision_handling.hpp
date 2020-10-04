#ifndef SIMULATION_COLLISION_HANDLING_HPP
#define SIMULATION_COLLISION_HANDLING_HPP

#include "contact.hpp"
#include "integrator.hpp"

/**
 * Routines that compute and apply the required response to handle collisions. */
namespace CollisionHandling {
    /*
     * Collision impulse computation and application.
     */

    /**
     * applies correcting impulses for a single collision
     * analogous with collision from Ref. 1 */
    void collision(Contact *contact, double epsilon);

    /**
     * finds all collisions and applies correcting impulses
     * iterates until all collisions are fixed
     * analogous with find_all_collisions from Ref. 1 */
    bool find_all_collisions(std::vector<Contact *> contacts);

    /*
     * Contact force computation and application.
     */

    /**
     * Computes the derivative of the normal vector of contact {c}.
     * Equal in function to "computeNdot" of Ref. 1. */
    glm::dvec3 compute_ndot(Contact *c);

    /**
     * Computes the contribution of the external force and inertial forces due to velocity of the contacts.
     * Equal in function to "compute_b_vector" of Ref. 1. */
    void compute_b_vector(double *bvec, std::vector<Contact *> contacts);

    /**
     * Computes the value of matrix A for pair of contacts {ci} and {cj}.
     * Equal in function to "compute_aij" of Ref. 1. */
    double compute_aij(Contact *ci, Contact *cj);

    /**
     * Computes the contribution of the inertias and contact geometry of bodies involved in the contacts.
     * Equal in function to "compute_a_matrix" of Ref. 1. */
    void compute_a_matrix(double *amat, std::vector<Contact *> contacts);

    /** finds resting contacts and prevents penetration */
    void compute_contact_forces(std::vector<Contact *> contacts);

    /*
     * Correction computation and application.
     */

    /** Corrects the state of the simulation according to Ref. 3.*/
    void correct_state(BodySystem *body_system);
}

#endif //SIMULATION_COLLISION_HANDLING_HPP
