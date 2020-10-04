#ifndef SIMULATION_CONTACT_DERIVATION_HPP
#define SIMULATION_CONTACT_DERIVATION_HPP

#include "rigid_body.hpp"
#include "collision.hpp"

namespace ContactDerivation {
    /** Enum denoting the topological element of a rigid body. */
    enum TopologicalType {
        FACE,
        /**
         * A face which is in contact with a separating plane, but not at all of the
         * faces vertices. Contains three or more points (otherwise it is considered an edge). */
        SPECIAL_FACE,
        EDGE,
        VERTEX
    };

    /**
     * Given the result of an intersection, determine the type of the topological element of {result->a}
     * interacting with the separating plane adjacent to {result->b}. */
    void find_topological_element(uint32_t *index, TopologicalType *type, Collision::IntersectResult *result);

    /**
     * Given the result of an intersection, determine points of contacts if topological element
     * of {result->a} is of type {FACE}. */
    std::vector<Contact *> get_contacts_face(Collision::IntersectResult *result, uint32_t index);

    /**
     * Given the result of an intersection, determine points of contacts if topological element
     * of {result->a} is of type {SPECIAL_FACE}. */
    std::vector<Contact *> get_contacts_special_face(Collision::IntersectResult *result, uint32_t index);

    /**
     * Given the result of an intersection, determine points of contacts if topological element
     * of {result->a} is of type {EDGE}. */
    std::vector<Contact *> get_contacts_edge(Collision::IntersectResult *result, uint32_t index);

    /**
     * Given the result of an intersection, determine points of contacts if topological element
     * of {result->a} is of type {VERTEX}. */
    std::vector<Contact *> get_contacts_vertex(Collision::IntersectResult *result, uint32_t index);

    /**
     * Subroutine for {get_contacts_face} and {get_contacts_special_face}.
     * Since their behavior is very similar, but {get_contacts_special_face} requires an additional check for
     * all vertices belonging to {result->a} whether they are within {Engine::DISTANCE_THRESHOLD} of the
     * separating plane. */
    std::vector<Contact *> get_contacts_face(Collision::IntersectResult *result, uint32_t fai, bool check_distance);

    /**
     * Generates result based on an IntersectResult struct containing a separating plane.
     * This may very well result in no contacts being generated, for example when a vertex of A is close
     * to the separating plane, but no where near the face of B. */
    std::vector<Contact *> get_contacts(Collision::IntersectResult *result);

    /**
     * Edge formed by {f1} and {f2} of face with normal {fn} (endpoints are ordered as they are in the face definition).
     * Other edge formed by {e1} and {e2}.
     * Returns true if the latter intersects former, under the assumption that all points lie in
     * the plane formed by the face. */
    bool test(glm::dvec3 *p, glm::dvec3 f1, glm::dvec3 f2, glm::dvec3 fn, glm::dvec3 e1, glm::dvec3 e2);


    /**
     * Returns true if {vertex_y} of {y} is inside the infinite 'column' defined by {face_x} of {x} along the normal
     * of that face. It is used in the case where {vertex_y} and {face_x} to be assumed to be in the same plane. */
    bool inside(RigidBody *x, RigidBody *y, uint32_t face_x, uint32_t vertex_y);

    /**
     * Same as above function, but also perform the check if the vertices of {face_x} are within
     * {Engine::DISTANCE_THRESHOLD} from the plane formed by {vertex_y} which is part of a face with {normal_y}. */
    bool inside(RigidBody *x, RigidBody *y, uint32_t face_x, uint32_t vertex_y, glm::dvec3 normal_y);
}

#endif //SIMULATION_CONTACT_DERIVATION_HPP
