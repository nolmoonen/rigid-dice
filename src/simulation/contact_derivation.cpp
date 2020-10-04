#include "contact_derivation.hpp"

bool ContactDerivation::test(
        glm::dvec3 *p, glm::dvec3 f1, glm::dvec3 f2, glm::dvec3 fn, glm::dvec3 e1, glm::dvec3 e2)
{
    /** determine inside or outside */
    // points 'outwards' since normal of a points outwards from a and
    // edges of a are counter-clockwise ordered from the outside
    glm::dvec3 fm = glm::normalize(glm::cross(f2 - f1, fn));
    double dist_e1 = glm::dot(fm, e1 - f1);
    double dist_e2 = glm::dot(fm, e2 - f1);

    // does not necessarily point inwards or outwards
    glm::dvec3 em = glm::normalize(glm::cross(e2 - e1, fn));
    double dist_f1 = glm::dot(em, f1 - e1);
    double dist_f2 = glm::dot(em, f2 - e1);

    /** determine intersection points */
    if (dist_e1 * dist_e2 <= 0 && dist_f1 * dist_f2 <= 0) {
        // intersect if endpoint lie on opposite side of line segment, for both segments
        glm::dvec3 x = glm::normalize(e2 - e1);
        *p = e2 - x * (dist_e2 / glm::dot(x, fm));
        // if v lies between the endpoints
        return glm::dot(e2 - e1, *p - e1) >= 0 && glm::dot(e1 - e2, *p - e2) >= 0;
    }

    return false;
}

void ContactDerivation::find_topological_element(
        uint32_t *index, TopologicalType *type, Collision::IntersectResult *result)
{
    // 1. find a face
    int64_t face_i = -1;
    for (uint32_t i = 0; i < result->a->shape->get_faces().size(); i++) {
        bool contained = true;
        for (uint32_t j = 0; j < result->a->shape->get_faces()[i].size(); j++) {
            glm::dvec3 v = result->a->get_world_space_vertex(result->a->shape->get_faces()[i][j].first);
            if (fabs(result->dist(v)) > Engine::DISTANCE_THRESHOLD) {
                contained = false;
                break;
            }
        }
        if (contained) {
            if (face_i != -1) {
                // two faces are contained. not necessarily a problem, but should be dealt with
                assert(0); // todo deal with this problem
            }
            face_i = i;
        }
    }

    if (face_i != -1) {
        *index = face_i;
        *type = FACE;
        return;
    }

    // 2. find a face with more than 1 edge and fewer than all edges
    int64_t special_face_i = -1;
    for (uint32_t i = 0; i < result->a->shape->get_faces().size(); i++) {
        uint32_t edges_contained = 0;
        glm::dvec3 e1 = result->a->get_world_space_vertex(result->a->shape->get_faces()[i].back().first);
        for (uint32_t j = 0; j < result->a->shape->get_faces()[i].size(); j++) {
            glm::dvec3 e2 = result->a->get_world_space_vertex(result->a->shape->get_faces()[i][j].first);
            if (fabs(result->dist(e1)) <= Engine::DISTANCE_THRESHOLD &&
                fabs(result->dist(e2)) <= Engine::DISTANCE_THRESHOLD) {
                edges_contained++;
            }
            e1 = e2;
        }
        if (edges_contained > 1) {
            if (special_face_i != -1) {
                // two faces are contained. not necessarily a problem, but should be dealt with
                assert(0); // todo deal with this problem
            }
            special_face_i = i;
        }
    }

    if (special_face_i != -1) {
        *index = special_face_i;
        *type = SPECIAL_FACE;
        return;
    }

    // 3. find an edge
    int64_t edge_i = -1;
    for (uint32_t i = 0; i < result->a->shape->get_edges().size(); i++) {
        glm::dvec3 e1 = result->a->get_world_space_vertex(result->a->shape->get_edges()[i].first);
        glm::dvec3 e2 = result->a->get_world_space_vertex(result->a->shape->get_edges()[i].second);
        if (fabs(result->dist(e1)) <= Engine::DISTANCE_THRESHOLD &&
            fabs(result->dist(e2)) <= Engine::DISTANCE_THRESHOLD) {
            if (edge_i != -1) {
                // two edges are contained. not necessarily a problem, but should be dealt with
                assert(0); // todo deal with this problem
            }
            edge_i = i;
        }
    }

    if (edge_i != -1) {
        *index = edge_i;
        *type = EDGE;
        return;
    }

    // 4. find a vertex
    int64_t vertex_i = -1;
    for (uint32_t i = 0; i < result->a->shape->get_model_vertices().size(); i++) {
        glm::dvec3 v = result->a->get_world_space_vertex(i);
        if (fabs(result->dist(v)) <= Engine::DISTANCE_THRESHOLD) {
            if (vertex_i != -1) {
                // two vertices are contained. not necessarily a problem, but should be dealt with
                assert(0); // todo deal with this problem
            }
            vertex_i = i;
        }
    }

    if (vertex_i != -1) {
        *index = vertex_i;
        *type = VERTEX;
        return;
    }

    // it should not be the case that no topological element is found
    assert(0);
}

std::vector<Contact *> ContactDerivation::get_contacts_face(Collision::IntersectResult *result, uint32_t index)
{
    // make use of the subroutine, without added check for distance
    return get_contacts_face(result, index, false);
}

std::vector<Contact *> ContactDerivation::get_contacts_special_face(Collision::IntersectResult *result, uint32_t index)
{
    // make use of the subroutine, with added check for distance
    return get_contacts_face(result, index, true);
}

std::vector<Contact *> ContactDerivation::get_contacts_edge(Collision::IntersectResult *result, uint32_t index)
{
    std::vector<Contact *> contacts;

    if (result->ee) {
        // todo technically could test if more than just the edge of B is involved in the separating plane however,
        //  if a face of B was involved it is highly likely that a separating plane was found defined by that face
        // plane formed by edge x edge, intersecting with edge
        // this creates point v lying on body A
        glm::dvec3 ea1 = result->a->get_world_space_vertex(result->a->shape->get_edges()[index].first);
        glm::dvec3 ea2 = result->a->get_world_space_vertex(result->a->shape->get_edges()[index].second);

        glm::dvec3 eb1 = result->b->get_world_space_vertex(result->b->shape->get_edges()[result->ebi].first);
        glm::dvec3 eb2 = result->b->get_world_space_vertex(result->b->shape->get_edges()[result->ebi].second);

        // direction of plane coming out of edge eb
        glm::dvec3 m = glm::normalize(glm::cross(result->n, result->eb));
        double dist_ea1 = glm::dot(m, ea1 - eb1);
        double dist_ea2 = glm::dot(m, ea2 - eb1);

        // direction of plane coming out of edge ea
        glm::dvec3 k = glm::normalize(glm::cross(result->n, result->ea));
        double dist_eb1 = glm::dot(k, eb1 - ea1);
        double dist_eb2 = glm::dot(k, eb2 - ea1);

        // test if edge of B which is part of the separating plane intersects the found edge of A
        if (dist_ea1 * dist_ea2 <= 0 && dist_eb1 * dist_eb2 <= 0) {
            glm::dvec3 x = glm::normalize(ea2 - ea1);
            glm::dvec3 v = ea2 - x * (dist_ea2 / glm::dot(x, m));
            glm::dvec3 pb = result->b->get_world_space_vertex(result->b->shape->get_edges()[result->ebi].first);
            contacts.emplace_back(new Contact(v, result->n, result->a, result->b, pb, result->ea, result->eb));
        }
    } else {
        // plane formed by face of b, against edge of A
        // find intersection points, and find out if the endpoints of edge of A are inside or outside face of B
        glm::dvec3 ea1 = result->a->get_world_space_vertex(result->a->shape->get_edges()[index].first);
        glm::dvec3 ea2 = result->a->get_world_space_vertex(result->a->shape->get_edges()[index].second);
        glm::dvec3 ea = glm::normalize(ea1 - ea2);

        bool ea1_inside = inside(result->b, result->a, result->fbi, result->a->shape->get_edges()[index].first);
        bool ea2_inside = inside(result->b, result->a, result->fbi, result->a->shape->get_edges()[index].second);
        glm::dvec3 p1;         // first intersection point
        bool p1_found = false; // whether the first intersection has been found
        glm::dvec3 eb_one;     // unitized direction of the first edge that is intersected
        glm::dvec3 p2;         // second intersection point
        bool p2_found = false; // whether the second intersection has been found
        glm::dvec3 eb_two;     // unitized direction of the second edge that is intersected

        glm::dvec3 eb1 = result->b->get_world_space_vertex(result->b->shape->get_faces()[result->fbi].back().first);
        for (uint32_t i = 0; i < result->b->shape->get_faces()[result->fbi].size(); i++) {
            glm::dvec3 eb2 = result->b->get_world_space_vertex(result->b->shape->get_faces()[result->fbi][i].first);

            glm::dvec3 p;
            if (test(&p, eb1, eb2, result->n, ea1, ea2)) {
                // intersection found between this current edge of b and the edge on a
                if (!p1_found) {
                    p1 = p;
                    p1_found = true;
                    eb_one = glm::normalize(eb2 - eb1);
                } else {
                    if (p2_found) {
                        assert(0); // an edge and a convex face cannot cause more than two intersections
                    }
                    p2 = p;
                    p2_found = true;
                    eb_two = glm::normalize(eb2 - eb1);
                }
            }

            eb1 = eb2;
        }

        /** four cases */
        if (ea1_inside && ea2_inside) {
            // edge is fully contained by face, create two vertex-face contacts
            assert(!p1_found && !p2_found);
            contacts.emplace_back(new Contact(ea1, result->n, result->a, result->b, eb1));
            contacts.emplace_back(new Contact(ea2, result->n, result->a, result->b, eb1));
        } else if (ea1_inside != ea2_inside) {
            // one point of edge is contained by face, the other is not, create one vertex-face and one edge-edge contact
            assert(p1_found && !p2_found);

            // find a normal formed by the cross project of the edges in question, pointing outwards from b
            glm::dvec3 n1 = glm::normalize(glm::cross(ea, eb_one));
            if (glm::dot(p1 - result->b->x, n1) < 0) {
                eb_one *= -1;
                n1 = glm::normalize(glm::cross(ea, eb_one));
            }

            // create the contacts
            if (ea1_inside) {
                contacts.emplace_back(new Contact(ea1, result->n, result->a, result->b, eb1));
                contacts.emplace_back(new Contact(p1, n1, result->a, result->b, eb1, ea, eb_one));
            } else { // if (eb2_inside)
                contacts.emplace_back(new Contact(ea2, result->n, result->a, result->b, eb1));
                contacts.emplace_back(new Contact(p1, n1, result->a, result->b, eb1, ea, eb_one));
            }
        } else if (!ea1_inside && !ea2_inside && p1_found && p2_found) {
            // endpoints of edge are outside face, but intersect at two points
            glm::dvec3 n1 = glm::normalize(glm::cross(ea, eb_one));
            if (glm::dot(p1 - result->b->x, n1) < 0) {
                eb_one *= -1;
                n1 = glm::normalize(glm::cross(ea, eb_two));
            }

            glm::dvec3 n2 = glm::normalize(glm::cross(ea, eb_one));
            if (glm::dot(p2 - result->b->x, n2) < 0) {
                eb_two *= -1;
                n2 = glm::normalize(glm::cross(ea, eb_two));
            }

            contacts.emplace_back(new Contact(p1, n1, result->a, result->b, eb1, ea, eb_one));
            contacts.emplace_back(new Contact(p2, n2, result->a, result->b, eb1, ea, eb_two));
        } else if (!ea1_inside && !ea2_inside && !p1_found && !p2_found) {
            // endpoints of edge are outside face, but intersect at no points
            // if this happens, no contact points should be generated
        } else {
            // no other cases possible
            assert(0);
        }
    }

    return contacts;
}

std::vector<Contact *> ContactDerivation::get_contacts_vertex(Collision::IntersectResult *result, uint32_t index)
{
    std::vector<Contact *> contacts;

    if (result->ee) {
        // NB: this case should not occur, if a separating plane is formed by two edges,
        // an edge must be the largest topological element of A lying in this plane
        assert(0);
    } else {
        // check if vertex is actually contained in face
        if (inside(result->b, result->a, result->fbi, index)) {
            glm::dvec3 pb = result->b->get_world_space_vertex(result->b->shape->get_edges()[result->ebi].first);
            glm::dvec3 p = result->a->get_world_space_vertex(index);
            contacts.emplace_back(new Contact(p, result->n, result->a, result->b, pb));
        }
    }

    return contacts;
}


std::vector<Contact *> ContactDerivation::get_contacts(Collision::IntersectResult *result)
{
    assert(!result->intersect);

    uint32_t index;
    TopologicalType type;
    find_topological_element(&index, &type, result);
    switch (type) {
        case FACE:
            return get_contacts_face(result, index);
        case SPECIAL_FACE:
            return get_contacts_special_face(result, index);
        case EDGE:
            return get_contacts_edge(result, index);
        case VERTEX:
            return get_contacts_vertex(result, index);
    }

    assert(0); // {find_topological_element} must return one of the specified types
    return std::vector<Contact *>();
}

std::vector<Contact *> ContactDerivation::get_contacts_face(
        Collision::IntersectResult *result, uint32_t fai, bool check_distance)
{
    std::vector<Contact *> contacts;

    if (result->ee) {
        // todo technically could test if more than just the edge of B is involved in the separating plane however,
        //  if a face of B was involved it is highly likely that a separating plane was found defined by that face
        // START DEBUG: assert that the face found contains the edge of the separating plane
        uint32_t contained = 0;
        for (uint32_t i = 0; i < result->a->shape->get_faces()[fai].size(); i++) {
            uint32_t vertex = result->a->shape->get_faces()[fai][i].first;
            std::pair<uint32_t, uint32_t> edge = result->a->shape->get_edges()[result->eai];
            if (vertex == edge.first || vertex == edge.second) contained++;
        }
        if (contained != 2) {
            assert(0); // the edge of the separating plane is not contained in the plane
        }
        // END DEBUG

        // we have a face-edge contact with a face of A and an edge of B
        glm::dvec3 eb1 = result->b->get_world_space_vertex(result->b->shape->get_edges()[result->ebi].first);
        glm::dvec3 eb2 = result->b->get_world_space_vertex(result->b->shape->get_edges()[result->ebi].second);
        glm::dvec3 eb = glm::normalize(eb1 - eb2);

        bool eb1_inside = inside(result->a, result->b, fai, result->a->shape->get_edges()[result->ebi].first);
        bool eb2_inside = inside(result->a, result->b, fai, result->a->shape->get_edges()[result->ebi].second);
        glm::dvec3 p1;          // first intersection point
        bool p1_found = false;  // whether the first intersection has been found
        glm::dvec3 ea_one;      // unitized direction of the first edge that is intersected
        glm::dvec3 p2;          // second intersection point
        bool p2_found = false;  // whether the second intersection has been found
        glm::dvec3 ea_two;      // unitized direction of the first edge that is intersected

        // if we first need to check whether a point is within distance from the separating plane,
        // ea1 is the last point which is within distance (instead of simply the last point)
        // this construction works since we know that at least three points are available (else we would be in the edge case)
        glm::dvec3 ea1;
        if (check_distance) {
            ea1 = glm::dvec3(0.);
            uint32_t n = result->a->shape->get_faces()[fai].size();
            for (uint32_t i = 0; i < n; i++) {
                glm::dvec3 v = result->a->get_world_space_vertex(result->a->shape->get_faces()[fai][n - 1 - i].first);
                if (fabs(result->dist(v)) <= Engine::DISTANCE_THRESHOLD) {
                    ea1 = v;
                    break;
                }
            }
        } else {
            ea1 = result->a->get_world_space_vertex(result->a->shape->get_faces()[fai].back().first);
        }
        for (uint32_t i = 0; i < result->a->shape->get_faces()[fai].size(); i++) {
            glm::dvec3 ea2 = result->a->get_world_space_vertex(result->a->shape->get_faces()[fai][i].first);
            // if ea2 is not within distance from the separating plane continue and do *not* update ea1
            if (check_distance && fabs(result->dist(ea2)) > Engine::DISTANCE_THRESHOLD) continue;

            glm::dvec3 p;
            if (test(&p, ea1, ea2, result->a->get_non_unit_normal(fai), eb1, eb2)) {
                if (!p1_found) {
                    p1 = p;
                    p1_found = true;
                    ea_one = glm::normalize(ea2 - ea1);
                } else {
                    if (p2_found) {
                        assert(0); // an edge and a convex face cannot cause more than two intersections
                    }
                    p2 = p;
                    p2_found = true;
                    ea_two = glm::normalize(ea2 - ea1);
                }
            }

            ea1 = ea2;
        }

        /** four cases */
        if (eb1_inside && eb2_inside) {
            assert(!p1_found && !p2_found);
            contacts.emplace_back(new Contact(eb1, -result->n, result->b, result->a, ea1));
            contacts.emplace_back(new Contact(eb2, -result->n, result->b, result->a, ea1));
        } else if (eb1_inside != eb2_inside) {
            // one point of edge is contained by face, the other is not, create one vertex-face and one edge-edge contact
            assert(p1_found && !p2_found);

            // find a normal formed by the cross project of the edges in question, pointing outwards from b
            glm::dvec3 n1 = glm::normalize(glm::cross(ea_one, eb));
            if (glm::dot(p1 - result->b->x, n1) < 0) {
                ea_one *= -1;
                n1 = glm::normalize(glm::cross(ea_one, eb));
            }

            if (eb1_inside) {
                contacts.emplace_back(new Contact(eb1, -result->n, result->b, result->a, ea1));
                contacts.emplace_back(new Contact(p1, n1, result->a, result->b, ea1, ea_one, eb));
            } else { // if (eb2_inside)
                contacts.emplace_back(new Contact(eb2, -result->n, result->b, result->a, ea1));
                contacts.emplace_back(new Contact(p1, n1, result->a, result->b, ea1, ea_one, eb));
            }
        } else if (!eb1_inside && !eb2_inside && p1_found && p2_found) {
            // endpoints of edge are outside face, but intersect at two points
            glm::dvec3 n1 = glm::normalize(glm::cross(ea_one, eb));
            if (glm::dot(p1 - result->b->x, n1) < 0) {
                ea_one *= -1;
                n1 = glm::normalize(glm::cross(ea_one, eb));
            }

            glm::dvec3 n2 = glm::normalize(glm::cross(ea_two, eb));
            if (glm::dot(p2 - result->b->x, n2) < 0) {
                ea_two *= -1;
                n2 = glm::normalize(glm::cross(ea_two, eb));
            }

            contacts.emplace_back(new Contact(p1, n1, result->a, result->b, ea1, ea_one, eb));
            contacts.emplace_back(new Contact(p2, n2, result->a, result->b, ea1, ea_two, eb));
        } else if (!eb1_inside && !eb2_inside && !p1_found && !p2_found) {
            // endpoints of edge are outside face, but intersect at no points
            // if this happens, no contact points should be generated
        } else {
            // no other cases possible
            assert(0);
        }
    } else {
        // face-face contact between A and B
        // iterate over all edges and make a decision on the endpoint of the edge whether to add a contact for
        // (or intersection with face border) this way no dupes are added
        uint32_t prev_va;
        // if we first need to check whether a point is within distance from the separating plane,
        // prev_v is the last point which is within distance (instead of simply the last point)
        // this construction works since we know that at least three points are available (else we would be in the edge case)
        if (check_distance) {
            prev_va = UINT32_MAX;
            uint32_t n = result->a->shape->get_faces()[fai].size();
            for (uint32_t i = 0; i < n; i++) {
                glm::dvec3 v = result->a->get_world_space_vertex(result->a->shape->get_faces()[fai][n - 1 - i].first);
                if (fabs(result->dist(v)) <= Engine::DISTANCE_THRESHOLD) {
                    prev_va = n - 1 - i;
                    break;
                }
            }
        } else {
            prev_va = result->a->shape->get_faces()[fai].back().first;
        }
        bool prev_va_inside = inside(result->b, result->a, result->fbi, prev_va);
        for (uint32_t i = 0; i < result->a->shape->get_faces()[fai].size(); i++) {
            uint32_t this_va = result->a->shape->get_faces()[fai][i].first;
            bool this_va_inside = inside(result->b, result->a, result->fbi, this_va);

            glm::dvec3 ea1 = result->a->get_world_space_vertex(prev_va);
            glm::dvec3 ea2 = result->a->get_world_space_vertex(this_va);

            // if ea2 is not within distance from the separating plane continue and do *not* update ea1
            if (check_distance && fabs(result->dist(ea2)) > Engine::DISTANCE_THRESHOLD) continue;

            /** do intersect test */
            glm::dvec3 p1;              // first point of intersection
            glm::dvec3 ea_one;          // (unitized) edge direction of A of first intersection
            glm::dvec3 eb_one;          // (unitized) edge direction of B of first intersection
            glm::dvec3 n_one;           // normal outwards from B of first intersection
            glm::dvec3 p2;              // second point of intersection
            glm::dvec3 ea_two;          // (unitized) edge direction of A of second intersection
            glm::dvec3 eb_two;          // (unitized) edge direction of B of second intersection
            glm::dvec3 n_two;           // normal outwards from B of first intersection
            uint32_t intersections = 0; // number of intersections
            glm::dvec3 eb1 = result->b->get_world_space_vertex(result->b->shape->get_faces()[result->fbi].back().first);
            for (uint32_t j = 0; j < result->b->shape->get_faces()[result->fbi].size(); j++) {
                glm::dvec3 eb2 = result->b->get_world_space_vertex(result->b->shape->get_faces()[result->fbi][j].first);
                glm::dvec3 *p = intersections == 0 ? &p1 : &p2;
                // NB: order of eb1 and eb2 matters
                if (test(p, eb1, eb2, result->b->get_non_unit_normal(result->fbi), ea1, ea2)) {
                    if (intersections == 0) {
                        ea_one = glm::normalize(ea1 - ea2);
                        eb_one = glm::normalize(eb1 - eb2);
                        n_one = glm::normalize(glm::cross(ea_one, eb_one));
                        if (glm::dot(p1 - result->b->x, n_one) < 0.) {
                            ea_one *= -1;
                            n_one = glm::normalize(glm::cross(ea_one, eb_one));
                        }
                    } else {
                        ea_two = glm::normalize(ea1 - ea2);
                        eb_two = glm::normalize(eb1 - eb2);
                        n_two = glm::normalize(glm::cross(ea_two, eb_two));
                        if (glm::dot(p2 - result->b->x, n_two) < 0.) {
                            ea_two *= -1;
                            n_two = glm::normalize(glm::cross(ea_two, eb_two));
                        }
                    }
                    intersections++;
                }
                eb1 = eb2;
            }

            assert(intersections < 3);

            /** interpret intersect results */
            if (!prev_va_inside && !this_va_inside) {
                // add no endpoints, add the two intersection points if they exist
                assert(intersections == 2 || intersections == 0);
                if (intersections == 2) {
                    contacts.emplace_back(new Contact(p1, n_one, result->a, result->b, eb1, ea_one, eb_one));
                    contacts.emplace_back(new Contact(p2, n_two, result->a, result->b, eb1, ea_two, eb_two));
                }
            } else if (!prev_va_inside && this_va_inside) {
                // add current endpoint, and intersection
//                assert(intersections == 1); // todo edges can be collinear: investigate if this can cause troubles (also in other places)
                contacts.emplace_back(new Contact(p1, n_one, result->a, result->b, eb1, ea_one, eb_one));
                contacts.emplace_back(
                        new Contact(result->a->get_world_space_vertex(this_va), result->n, result->a, result->b, eb1));
            } else if (prev_va_inside && !this_va_inside) {
                // only add intersection
                assert(intersections == 1);
                contacts.emplace_back(new Contact(p1, n_one, result->a, result->b, eb1, ea_one, eb_one));
            } else { // if (prev_inside && this_inside)
                // only add current endpoint
//                assert(intersections == 0); // todo collinearity issue: see todo above
                contacts.emplace_back(
                        new Contact(result->a->get_world_space_vertex(this_va), result->n, result->a, result->b, eb1));
            }

            prev_va_inside = this_va_inside;
            prev_va = this_va;
        }

        /** now do the same from Bs POV, and do not add intersections */

        glm::dvec3 fbn = glm::normalize(result->b->get_non_unit_normal(result->fbi));
        uint32_t prev_vb = result->b->shape->get_faces()[result->fbi].back().first;
        bool prev_vb_inside;
        if (check_distance) {
            prev_vb_inside = inside(result->a, result->b, fai, prev_vb, fbn);
        } else {
            prev_vb_inside = inside(result->a, result->b, fai, prev_vb);
        }
        for (uint32_t i = 0; i < result->b->shape->get_faces()[result->fbi].size(); i++) {
            uint32_t this_vb = result->b->shape->get_faces()[result->fbi][i].first;
            bool this_vb_inside;
            if (check_distance) {
                this_vb_inside = inside(result->a, result->b, fai, this_vb, fbn);
            } else {
                this_vb_inside = inside(result->a, result->b, fai, this_vb);
            }

            /** interpret intersect results */
            if (!prev_vb_inside && !this_vb_inside) {
                // add no endpoints, add the two intersection points if they exist
            } else if (!prev_vb_inside && this_vb_inside) {
                // add current endpoint, and intersection
                contacts.emplace_back(new Contact(
                        result->b->get_world_space_vertex(this_vb),
                        glm::normalize(result->a->get_non_unit_normal(fai)),
                        result->b, result->a,
                        result->a->get_world_space_vertex(result->a->shape->get_faces()[fai][0].first)));
            } else if (prev_vb_inside && !this_vb_inside) {
                // only add intersection
            } else { // if (prev_vb_inside && this_vb_inside)
                // only add current endpoint
                contacts.emplace_back(new Contact(
                        result->b->get_world_space_vertex(this_vb),
                        glm::normalize(result->a->get_non_unit_normal(fai)),
                        result->b, result->a,
                        result->a->get_world_space_vertex(result->a->shape->get_faces()[fai][0].first)));
            }

            prev_vb_inside = this_vb_inside;
            prev_vb = this_vb;
        }
    }

    return contacts;
}

bool ContactDerivation::inside(RigidBody *x, RigidBody *y, uint32_t face_x, uint32_t vertex_y)
{
    // point of body Y
    glm::dvec3 vy = y->get_world_space_vertex(vertex_y);

    bool all_inside = true;
    glm::dvec3 ex1 = x->get_world_space_vertex(x->shape->get_faces()[face_x].back().first);
    for (uint32_t i = 0; i < x->shape->get_faces()[face_x].size(); i++) {
        glm::dvec3 ex2 = x->get_world_space_vertex(x->shape->get_faces()[face_x][i].first);
        // points 'outwards' of edges of X since normal of face of X points outwards from X and
        // edges of X are counter-clockwise ordered from the outside
        glm::dvec3 m = glm::normalize(glm::cross(ex2 - ex1, x->get_non_unit_normal(face_x)));
        if (glm::dot(vy - ex1, m) > 0) {
            all_inside = false;
            break;
        }

        ex1 = ex2;
    }

    return all_inside;
}

bool ContactDerivation::inside(RigidBody *x, RigidBody *y, uint32_t face_x, uint32_t vertex_y, glm::dvec3 normal_y)
{
    // point of body Y
    glm::dvec3 vy = y->get_world_space_vertex(vertex_y);

    bool all_inside = true;
    // find the last point which is within threshold
    glm::dvec3 ex1 = glm::dvec3(0.);
    uint32_t n = x->shape->get_faces()[face_x].size();
    for (uint32_t i = 0; i < n; i++) {
        glm::dvec3 vx = x->get_world_space_vertex(x->shape->get_faces()[face_x][n - 1 - i].first);
        if (fabs(glm::dot(normal_y, vx - vy)) <= Engine::DISTANCE_THRESHOLD) {
            ex1 = vx;
            break;
        }
    }
    for (uint32_t i = 0; i < x->shape->get_faces()[face_x].size(); i++) {
        glm::dvec3 ex2 = x->get_world_space_vertex(x->shape->get_faces()[face_x][i].first);
        // if ex2 is not within distance from the separating plane continue and do *not* update ex1
        if (fabs(glm::dot(normal_y, ex2 - vy)) > Engine::DISTANCE_THRESHOLD) continue;
        // points 'outwards' of edges of X since normal of face of X points outwards from X and
        // edges of X are counter-clockwise ordered from the outside
        glm::dvec3 m = glm::normalize(glm::cross(ex2 - ex1, x->get_non_unit_normal(face_x)));
        if (glm::dot(vy - ex1, m) > 0) {
            all_inside = false;
            break;
        }

        ex1 = ex2;
    }

    return all_inside;
}