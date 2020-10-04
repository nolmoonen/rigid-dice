#include "collision.hpp"

Collision::IntersectResult::IntersectResult(
) :
        intersect(true)
{}

Collision::IntersectResult::IntersectResult(
        glm::dvec3 p_p, glm::dvec3 p_n, RigidBody *p_a, RigidBody *p_b,
        uint32_t p_fbi
) :
        intersect(false), p(p_p), n(p_n), ee(false), a(p_a), b(p_b), fbi(p_fbi)
{}

Collision::IntersectResult::IntersectResult(
        glm::dvec3 p_p, glm::dvec3 p_n, RigidBody *p_a, RigidBody *p_b,
        glm::dvec3 p_ea, glm::dvec3 p_eb, uint32_t p_eai, uint32_t p_ebi
) :
        intersect(false), p(p_p), n(p_n), ee(true), a(p_a), b(p_b), ea(p_ea), eb(p_eb), eai(p_eai), ebi(p_ebi)
{}

double Collision::IntersectResult::dist(glm::dvec3 v) const
{
    return glm::dot(n, v - p);
}

/**
 * Tests on which side the vertices of {e} lie
 * compared to a plane formed by {p} and {n} which lies on {e}.
 * todo this method misses the case where all vertices of a lie on the plane formed by p and n
 */
int32_t which_side(RigidBody *e, glm::dvec3 p, glm::dvec3 n)
{
    uint32_t positive = 0;
    uint32_t negative = 0;
    // iterate vertices of e
    for (uint32_t i = 0; i < e->shape->get_model_vertices().size(); i++) {
        glm::dvec3 v = e->get_world_space_vertex(i);
        double t = glm::dot(n, v - p);
        if (t > 0) positive++; else if (t < 0) negative++;
        if (positive && negative) return 0;
    }

    if (positive) {
        return +1;
    } else {
        return -1;
    }
}

/**
 * Tests on which side the (offset) vertices of {c} lie
 * compared to a plane formed by {p} and {n} which lies on {d}.
 * todo this method misses the case where all vertices of a lie on the plane formed by p and n
 */
int32_t which_side(RigidBody *c, RigidBody *d, glm::dvec3 p, glm::dvec3 n, double offset)
{
    uint32_t positive = 0;
    uint32_t negative = 0;
    // iterate vertices of c
    for (uint32_t i = 0; i < c->shape->get_model_vertices().size(); i++) {
        glm::dvec3 v = c->get_world_space_vertex(i, offset, d->x - c->x);
        double t = glm::dot(n, v - p);
        if (t > 0) positive++; else if (t < 0) negative++;
        if (positive && negative) return 0;
    }

    if (positive) {
        return +1;
    } else {
        return -1;
    }
}

Collision::IntersectResult Collision::intersect(RigidBody *x, RigidBody *y, double offset)
{
    int32_t side_x, side_y;

    // take x as b and test planes formed by faces of x against (offset) vertices of y
    // (we know that the vertices of x all lie on the negative side of this plane
    for (uint32_t i = 0; i < x->shape->get_faces().size(); i++) {
        glm::dvec3 p = x->get_world_space_vertex(x->shape->get_faces()[i][0].first);
        glm::dvec3 n = glm::normalize(x->get_non_unit_normal(i));
        if (which_side(y, x, p, n, offset) > 0) {
            return {p, n, y, x, i};
        }
    }

    // take y as b and test planes formed by faces of y against (offset) vertices of x
    // (we know that the vertices of y all lie on the negative side of this plane)
    for (uint32_t i = 0; i < y->shape->get_faces().size(); i++) {
        glm::dvec3 p = y->get_world_space_vertex(y->shape->get_faces()[i][0].first);
        glm::dvec3 n = glm::normalize(y->get_non_unit_normal(i));
        if (which_side(x, y, p, n, offset) > 0) {
            return {p, n, x, y, i};
        }
    }

    for (uint32_t i = 0; i < x->shape->get_edges().size(); i++) {
        glm::dvec3 ex0 = x->get_world_space_vertex(x->shape->get_edges()[i].first);
        glm::dvec3 ex1 = x->get_world_space_vertex(x->shape->get_edges()[i].second);
        glm::dvec3 ex = glm::normalize(ex0 - ex1);
        for (uint32_t j = 0; j < y->shape->get_edges().size(); j++) {
            glm::dvec3 ey0 = y->get_world_space_vertex(y->shape->get_edges()[j].first);
            glm::dvec3 ey1 = y->get_world_space_vertex(y->shape->get_edges()[j].second);
            glm::dvec3 ey = glm::normalize(ey0 - ey1);

            glm::dvec3 n = glm::normalize(glm::cross(ex, ey));

            // take x as b
            // test the plane formed by edge of x against (offset) vertices of y
            side_y = which_side(y, x, ex0, n, offset);
            if (side_y == 0) goto other_way;
            // test the plane formed by edge of x against vertices of x
            side_x = which_side(x, ex0, n);
            if (side_x == 0) goto other_way;

            if (side_x * side_y < 0) {
                // if the vertices of x lie on the positive side of the plane, the normal does not point outwards
                // from b, so correct it
                if (side_x == 1) {
                    ex *= -1.;
                    n = glm::normalize(glm::cross(ex, ey));
                }
                return {ex0, n, y, x, ey, ex, j, i};
            }

            other_way:
            // take y as b
            // test the plane formed by edge of y against (offset) vertices of x
            side_x = which_side(x, y, ey0, n, offset);
            if (side_x == 0) continue;
            // test the plane formed by edge of y against vertices of y
            side_y = which_side(y, ey0, n);
            if (side_y == 0) continue;

            if (side_x * side_y < 0) {
                // if the vertices of y lie on the positive side of the plane, the normal does not point outwards
                // from b, so correct it
                if (side_y == 1) {
                    ex *= -1.;
                    n = glm::normalize(glm::cross(ex, ey));
                }
                return {ey0, n, x, y, ex, ey, i, j};
            }
        }
    }

    return {}; // default constructor sets intersect to true
}