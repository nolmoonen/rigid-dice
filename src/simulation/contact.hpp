#ifndef SIMULATION_CONTACT_HPP
#define SIMULATION_CONTACT_HPP

#include "rigid_body.hpp"
#include "collision.hpp"

class RigidBody;

class Contact {
public:
    /** Point of contact, which always lies on {body_a}. */
    glm::dvec3 p;

    /** Unitized normal pointing outwards from {body_b}. */
    glm::dvec3 n;

    /**
     * Body {p} is attached to.
     * If {vf}, it is the body that does not contain the face. **/
    RigidBody *body_a;

    /**
     * If {vf}, it is the body that contains the face. */
    RigidBody *body_b;

    /**
     * If {vf}, point on the face of {body_b}. If not, point on the edge of {body_b}.
     * Used to compute distance between {p} and {body_b}. */
    glm::dvec3 pb;

    /** If not {vf}, the direction of the edge connected to {body_a}. */
    glm::dvec3 ea;

    /** If not {vf}, the direction of the edge connected to {body_b}. */
    glm::dvec3 eb;

    /** True if contact is formed by vertex-face interaction. */
    bool vf;

    Contact(
            glm::dvec3 p_p, glm::dvec3 p_n, RigidBody *p_body_a, RigidBody *p_body_b, glm::dvec3 p_pb);

    Contact(
            glm::dvec3 p_p, glm::dvec3 p_n, RigidBody *p_body_a, RigidBody *p_body_b, glm::dvec3 p_pb,
            glm::dvec3 p_ea, glm::dvec3 p_eb);

    /** Returns the distance from {p} to {body_b}. */
    double distance() const;
};

#endif //SIMULATION_CONTACT_HPP
