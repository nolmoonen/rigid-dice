#include "contact.hpp"

Contact::Contact(
        glm::dvec3 p_p, glm::dvec3 p_n, RigidBody *p_body_a, RigidBody *p_body_b, glm::dvec3 p_pb
) :
        p(p_p), n(p_n), body_a(p_body_a), body_b(p_body_b), pb(p_pb)
{
    vf = true;
    if (glm::dot(p_p - p_body_b->x, p_n) < 0) {
        // assert that the normal always points outwards from b
        assert(0);
    }
}

Contact::Contact(
        glm::dvec3 p_p, glm::dvec3 p_n, RigidBody *p_body_a, RigidBody *p_body_b, glm::dvec3 p_pb,
        glm::dvec3 p_ea, glm::dvec3 p_eb
) :
        Contact(p_p, p_n, p_body_a, p_body_b, p_pb)
{
    vf = false;
    ea = p_ea;
    eb = p_eb;
}

double Contact::distance() const
{
    return glm::dot(n, p - pb);
}
