#ifndef SIMULATION_COLLISION_HPP
#define SIMULATION_COLLISION_HPP

#include <vector>
#include <algorithm>

#include "rigid_body.hpp"
#include "contact.hpp"
#include "engine.hpp"

class Contact;

namespace Collision {
    /**
     * For every pair {a} and {b}, only one instance of this struct should exist, to prevent creating duplicate
     * contact points.
     */
    struct IntersectResult {
        /** Whether they intersect; whether a separating plane exists. */
        bool intersect;
        /** If {intersect} is false, contains {p} and {n} form a plane that separates {a} and {b}. */
        /** Point on {b} (vertex on a face or endpoint of an edge). */
        glm::dvec3 p{};
        /** Normalized, pointing outwards from {b}. If {ee}, {ea} x {eb} = n. */
        glm::dvec3 n{};

        /** True if the separating plane is found by edge-edge. */
        bool ee{};

        RigidBody *a{};
        /** Body on which the separating plane is contained. */
        RigidBody *b{};

        /** If {ee}, the direction of the edge on {a}. */
        glm::dvec3 ea{};
        /** If {ee}, the direction of the edge on {b}. */
        glm::dvec3 eb{};

        /*
         * Reference data.
         */

        /** If {ee}, the index of edge on {a}. */
        uint32_t eai{};
        /** If {ee}, the index of edge on {b}. */
        uint32_t ebi{};
        /** If not {ee}, the index of face on {b}. */
        uint32_t fbi{};

        IntersectResult();

        /** Constructor for the case the separating plane is formed by a face. */
        IntersectResult(
                glm::dvec3 p_p, glm::dvec3 p_n, RigidBody *p_a, RigidBody *p_b,
                uint32_t p_fbi);

        /** Constructor for the case the separating plane is formed by the cross product of two edges. */
        IntersectResult(
                glm::dvec3 p_p, glm::dvec3 p_n, RigidBody *p_a, RigidBody *p_b,
                glm::dvec3 p_ea, glm::dvec3 p_eb, uint32_t p_eai, uint32_t p_ebi);

        /** Returns the distance from the separating plane to {v}. */
        double dist(glm::dvec3 v) const;
    };

    /**
     * Fills an IntersectResult struct for this pair of {x} and {y},
     * by performing a check whether a separating plane can be found between the pair of bodies,
     * which either is defined by a face of either one, or a defined by the cross product of a pair of edges. */
    IntersectResult intersect(RigidBody *x, RigidBody *y, double offset);
}

#endif //SIMULATION_COLLISION_HPP