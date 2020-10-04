#include "collision_handling.hpp"

void CollisionHandling::collision(Contact *contact, double epsilon)
{
    glm::dvec3 padot = contact->body_a->point_velocity(contact->p); // P^{dot}a^{line}(t_0)
    glm::dvec3 pbdot = contact->body_b->point_velocity(contact->p); // P^{dot}b^{line}(t_0)
    glm::dvec3 n = contact->n;                                            // n^{hat}(t_0)
    glm::dvec3 ra = contact->p - contact->body_a->x;
    glm::dvec3 rb = contact->p - contact->body_b->x;

    double vrel = glm::dot(n, padot - pbdot); // v^{line}_{rel}
    double numerator = -(1. + epsilon) * vrel;

    double term1 = contact->body_a->shape->get_inv_mass();
    double term2 = contact->body_b->shape->get_inv_mass();
    double term3 = glm::dot(n, glm::cross(contact->body_a->i_inv * glm::cross(ra, n), ra));
    double term4 = glm::dot(n, glm::cross(contact->body_b->i_inv * glm::cross(rb, n), rb));

    double impulse_magnitude = numerator / (term1 + term2 + term3 + term4);
    glm::dvec3 impulse = impulse_magnitude * n;

    // add the impulse to contact objects
    contact->body_a->p += impulse;
    contact->body_b->p -= impulse;
    contact->body_a->l += glm::cross(ra, impulse);
    contact->body_b->l -= glm::cross(rb, impulse);

    // recompute auxiliary quantities
    contact->body_a->v = contact->body_a->p * contact->body_a->shape->get_inv_mass();
    contact->body_b->v = contact->body_b->p * contact->body_b->shape->get_inv_mass();
    contact->body_a->omega = contact->body_a->i_inv * contact->body_a->l;
    contact->body_b->omega = contact->body_b->i_inv * contact->body_b->l;
}

bool CollisionHandling::find_all_collisions(std::vector<Contact *> contacts)
{
    const double EPSILON = .6; // coefficient of restitution

    for (auto &contact : contacts) {
        glm::dvec3 padot = contact->body_a->point_velocity(contact->p); // P^{dot}a^{line}(t_0)
        glm::dvec3 pbdot = contact->body_b->point_velocity(contact->p); // P^{dot}b^{line}(t_0)
        double vrel = glm::dot(contact->n, padot - pbdot);                    // v^{line}_{rel}

        if (vrel > Engine::COLLISION_THRESHOLD) {
            // moving away: do nothing
        } else if (vrel < -Engine::COLLISION_THRESHOLD) {
            // interpenetration: find impulse
            collision(contact, EPSILON);
            return true;
        } else { // if (vrel >= -Engine::COLLISION_THRESHOLD && vrel <= Engine::COLLISION_THRESHOLD)
            // resting contact: do nothing for now
        }
    }

    return false;
}

glm::dvec3 CollisionHandling::compute_ndot(Contact *c)
{
    if (c->vf) {
        return glm::cross(c->body_b->omega, c->n);
    } else {
        // find derivative of (ea x eb) / |ea x eb|
        glm::dvec3 eadot = glm::cross(c->body_a->omega, c->ea); // derivative of ea
        glm::dvec3 ebdot = glm::cross(c->body_b->omega, c->eb); // derivative of eb
        glm::dvec3 n1 = glm::cross(c->ea, c->eb); // ea x eb
        glm::dvec3 z = glm::cross(eadot, c->eb) + glm::cross(c->ea, ebdot); // derivative of ea x eb
        double l = glm::length(n1); // |ea x eb|
        n1 = n1 / l; // ea x eb normalized
        return (z - (glm::dot(z, n1) * n1)) / l;
    }
}

void CollisionHandling::compute_b_vector(double *bvec, std::vector<Contact *> contacts)
{
    for (uint32_t i = 0; i < contacts.size(); i++) {
        Contact *c = contacts[i];
        RigidBody *a = c->body_a;
        RigidBody *b = c->body_b;
        glm::dvec3 n = c->n;
        glm::dvec3 ra = c->p - a->x;
        glm::dvec3 rb = c->p - b->x;

        // get the external forces and torques
        glm::dvec3 f_ext_a = a->force;
        glm::dvec3 f_ext_b = b->force;
        glm::dvec3 t_ext_a = a->torque;
        glm::dvec3 t_ext_b = b->torque;

        // compute the part due to the external force and torque
        glm::dvec3 a_ext_part = f_ext_a * a->shape->get_inv_mass() + glm::cross(a->i_inv * t_ext_a, ra);
        glm::dvec3 b_ext_part = f_ext_b * b->shape->get_inv_mass() + glm::cross(b->i_inv * t_ext_b, rb);

        // compute the part due to velocity
        glm::dvec3 a_vel_part = glm::cross(a->omega, glm::cross(a->omega, ra)) +
                                glm::cross(a->i_inv * glm::cross(a->l, a->omega), ra);
        glm::dvec3 b_vel_part = glm::cross(b->omega, glm::cross(b->omega, rb)) +
                                glm::cross(b->i_inv * glm::cross(b->l, b->omega), rb);

        // combine the above results
        double k1 = glm::dot(n, (a_ext_part + a_vel_part) - (b_ext_part + b_vel_part));

        glm::dvec3 ndot = compute_ndot(c);
        double k2 = 2. * glm::dot(ndot, a->point_velocity(c->p) - b->point_velocity(c->p));
        bvec[i] = k1 + k2;
    }
}

double CollisionHandling::compute_aij(Contact *ci, Contact *cj)
{
    // if the bodies involved in the ith and jth contact are distinct, then aij is zero
    if ((ci->body_a != cj->body_a) && (ci->body_b != cj->body_b) &&
        (ci->body_a != cj->body_b) && (ci->body_b != cj->body_a)) {
        return 0.;
    }

    RigidBody *a = ci->body_a;
    RigidBody *b = ci->body_b;
    glm::dvec3 ni = ci->n;
    glm::dvec3 nj = cj->n;
    glm::dvec3 pi = ci->p;
    glm::dvec3 pj = cj->p;
    glm::dvec3 ra = pi - a->x;
    glm::dvec3 rb = pi - b->x;

    // what force and torque does contact j exert on body a?
    glm::dvec3 force_on_a = glm::dvec3(0.);  // force direction of jth contact force on a
    glm::dvec3 torque_on_a = glm::dvec3(0.); // torque direction
    if (cj->body_a == ci->body_a) {
        force_on_a = nj;
        torque_on_a = glm::cross(pj - a->x, force_on_a);
    } else if (cj->body_b == ci->body_a) {
        force_on_a = -nj;
        torque_on_a = glm::cross(pj - a->x, force_on_a);
    }

    // what force and torque does contact j exert on body b?
    glm::dvec3 force_on_b = glm::dvec3(0.);  // force direction of jth contact force on b
    glm::dvec3 torque_on_b = glm::dvec3(0.); // torque direction
    if (cj->body_a == ci->body_b) {
        force_on_b = nj;
        torque_on_b = glm::cross(pj - b->x, force_on_b);
    } else if (cj->body_b == ci->body_b) {
        force_on_b = -nj;
        torque_on_b = glm::cross(pj - b->x, force_on_b);
    }

    // compute how the jth contact force affects the linear and angular acceleration of the contact point on body a
    glm::dvec3 a_linear = force_on_a * a->shape->get_inv_mass();
    glm::dvec3 a_angular = glm::cross(a->i_inv * torque_on_a, ra);

    glm::dvec3 b_linear = force_on_b * b->shape->get_inv_mass();
    glm::dvec3 b_angular = glm::cross(b->i_inv * torque_on_b, rb);

    return glm::dot(ni, (a_linear + a_angular) - (b_linear + b_angular));
}

void CollisionHandling::compute_a_matrix(double *amat, std::vector<Contact *> contacts)
{
    for (uint32_t i = 0; i < contacts.size(); i++) {
        // fill in for every pair, since matrix is symmetric
        for (uint32_t j = i + 1; j < contacts.size(); j++) {
            double val = compute_aij(contacts[i], contacts[j]);
            amat[i * contacts.size() + j] = val;
            amat[j * contacts.size() + i] = val;
        }
        // fill in the diagonal
        amat[i * contacts.size() + i] = compute_aij(contacts[i], contacts[i]);
    }
}

void CollisionHandling::compute_contact_forces(std::vector<Contact *> contacts)
{
    /** identify all resting contacts */
    std::vector<Contact *> resting_contacts;
    for (uint32_t i = 0; i < contacts.size(); i++) {
        glm::dvec3 padot = contacts[i]->body_a->point_velocity(contacts[i]->p); // P^{dot}a^{line}(t_0)
        glm::dvec3 pbdot = contacts[i]->body_b->point_velocity(contacts[i]->p); // P^{dot}b^{line}(t_0)
        double vrel = glm::dot(contacts[i]->n, padot - pbdot);                        // v^{line}_{rel}

        if (vrel > Engine::COLLISION_THRESHOLD) {
            // moving away: do nothing
        } else if (vrel < -Engine::COLLISION_THRESHOLD) {
            // interpenetration: already been handled
            assert(0);
        } else { // if (vrel >= -COLLISION_THRESHOLD && vrel <= COLLISION_THRESHOLD)
            resting_contacts.emplace_back(contacts[i]);
        }
    }

    if (resting_contacts.empty()) return;

    /** solve all resting contacts */
    auto bvec = (double *) malloc(resting_contacts.size() * sizeof(double));
    compute_b_vector(bvec, resting_contacts);

    auto amat = (double *) malloc(resting_contacts.size() * resting_contacts.size() * sizeof(double));
    compute_a_matrix(amat, resting_contacts);

    auto fvec = (double *) malloc(resting_contacts.size() * sizeof(double));
    math::qp_solve(amat, bvec, fvec, resting_contacts.size());

    /** scatter the forces */
    for (uint32_t i = 0; i < resting_contacts.size(); i++) {
        // force may be very slightly smaller than zero due to the threshold used in qp_solve
        if (fvec[i] < 0.) {
            fvec[i] = 0.;
        }
        glm::dvec3 force = fvec[i] * resting_contacts[i]->n;

        resting_contacts[i]->body_a->force += force;
        resting_contacts[i]->body_b->force -= force;

        resting_contacts[i]->body_a->torque += glm::cross(
                resting_contacts[i]->p - resting_contacts[i]->body_a->x, force);
        resting_contacts[i]->body_b->torque -= glm::cross(
                resting_contacts[i]->p - resting_contacts[i]->body_b->x, force);
    }

    free(fvec);
    free(amat);
    free(bvec);
}

void CollisionHandling::correct_state(BodySystem *body_system)
{
    bool needs_correction = false;
    std::vector<double> deltas;
    std::vector<Contact *> contacts = CollisionDetection::find_all_contacts(body_system);

    for (auto &contact : contacts) {
        double delta = contact->distance();
        assert(delta >= -Engine::DISTANCE_THRESHOLD);
        needs_correction |= delta <= -Engine::WARNING_DISTANCE_THRESHOLD;
        deltas.emplace_back(delta);
    }

    if (!needs_correction) return;

    auto b = (double *) malloc(contacts.size() * sizeof(double)); // 1 rhs vector
    for (uint32_t i = 0; i < contacts.size(); i++) {
        b[i] = -deltas[i]; // NB: notice the minus sign
    }

    auto amat = (double *) malloc(contacts.size() * contacts.size() * sizeof(double));
    CollisionHandling::compute_a_matrix(amat, contacts);

    auto fvec = (double *) calloc(contacts.size(), sizeof(double));
    math::lp_solve(amat, fvec, b, contacts.size());

    Integrator::clear_forces(body_system);

    for (uint32_t i = 0; i < contacts.size(); i++) {
        glm::dvec3 force = fvec[i] * contacts[i]->n;

        contacts[i]->body_a->force += force;
        contacts[i]->body_b->force -= force;

        contacts[i]->body_a->torque += glm::cross(contacts[i]->p - contacts[i]->body_a->x, force);
        contacts[i]->body_b->torque -= glm::cross(contacts[i]->p - contacts[i]->body_b->x, force);
    }

    for (auto &body : body_system->bodies) {
        body.x += body.force * body.shape->get_inv_mass();
        body.a += Integrator::star(body.i_inv * body.torque) * body.a;

        // todo use a quaternion
        body.a = glm::orthonormalize(body.a);

//        // compute auxiliary quantities
//        body.v = body.p * body.shape->get_inv_mass();
        body.i_inv = body.a * body.shape->get_inv_moment_of_inertia() * glm::transpose(body.a);
        body.omega = body.i_inv * body.l;
    }
}