#include "integrator.hpp"

void Integrator::clear_forces(BodySystem *body_system)
{
    for (auto &body : body_system->bodies) {
        body.clear_force_and_torque();
    }
}

void Integrator::apply_forces(BodySystem *body_system)
{
    for (auto &force : body_system->forces) {
        force->apply_force_and_torque();
    }
}

glm::dmat3 Integrator::star(glm::dvec3 a)
{
    glm::dmat3 r_val = glm::dmat3(
            0, a.z, -a.y,
            -a.z, 0, a.x,
            a.y, -a.x, 0
    );

    return r_val;
}

void Integrator::runge_kutta_4(BodySystem *body_system, double dt)
{
    std::vector<RigidBody> initial_state = body_system->bodies; // save the state at t0

    /** k1 */

    // calculate hf(x0, t0)
    for (auto &body : body_system->bodies) {
        // compute the change in variables
        body.x = dt * body.v;
        body.p = dt * body.force;
        body.a = dt * star(body.omega) * body.a;
        body.l = dt * body.torque;

//        body.a = glm::orthonormalize(body.a);

        // compute change in auxiliary variables
        body.v = body.p * body.shape->get_inv_mass();
        body.i_inv = body.a * body.shape->get_inv_moment_of_inertia() * glm::transpose(body.a);
        body.omega = body.i_inv * body.l;
    }

    std::vector<RigidBody> k1 = body_system->bodies;

    /** k2 */

    // set state to x0 + k1/2
    for (uint32_t i = 0; i < body_system->bodies.size(); i++) {
        body_system->bodies[i].x = initial_state[i].x + .5 * k1[i].x;
        body_system->bodies[i].p = initial_state[i].p + .5 * k1[i].p;
        body_system->bodies[i].a = initial_state[i].a + .5 * k1[i].a;
        body_system->bodies[i].l = initial_state[i].l + .5 * k1[i].l;

        body_system->bodies[i].a = glm::orthonormalize(body_system->bodies[i].a);
        body_system->bodies[i].v = body_system->bodies[i].p * body_system->bodies[i].shape->get_inv_mass();
        body_system->bodies[i].i_inv =
                body_system->bodies[i].a * body_system->bodies[i].shape->get_inv_moment_of_inertia() *
                glm::transpose(body_system->bodies[i].a);
        body_system->bodies[i].omega = body_system->bodies[i].i_inv * body_system->bodies[i].l;
    }

    // calculate hf(x0 + k1/2, t0 + dt/2)
    for (auto &body : body_system->bodies) {
        // compute the change in variables
        body.x = dt * body.v;
        body.p = dt * body.force;
        body.a = dt * star(body.omega) * body.a;
        body.l = dt * body.torque;

//        body.a = glm::orthonormalize(body.a);

        // compute change in auxiliary variables
        body.v = body.p * body.shape->get_inv_mass();
        body.i_inv = body.a * body.shape->get_inv_moment_of_inertia() * glm::transpose(body.a);
        body.omega = body.i_inv * body.l;
    }

    std::vector<RigidBody> k2 = body_system->bodies;

    /** k3 */

    // set state to x0 + k2/2
    for (uint32_t i = 0; i < body_system->bodies.size(); i++) {
        body_system->bodies[i].x = initial_state[i].x + .5 * k2[i].x;
        body_system->bodies[i].p = initial_state[i].p + .5 * k2[i].p;
        body_system->bodies[i].a = initial_state[i].a + .5 * k2[i].a;
        body_system->bodies[i].l = initial_state[i].l + .5 * k2[i].l;

        body_system->bodies[i].a = glm::orthonormalize(body_system->bodies[i].a);
        body_system->bodies[i].v = body_system->bodies[i].p * body_system->bodies[i].shape->get_inv_mass();
        body_system->bodies[i].i_inv =
                body_system->bodies[i].a * body_system->bodies[i].shape->get_inv_moment_of_inertia() *
                glm::transpose(body_system->bodies[i].a);
        body_system->bodies[i].omega = body_system->bodies[i].i_inv * body_system->bodies[i].l;
    }


    // calculate hf(x0 + k2/2, t0 + dt/2)
    for (auto &body : body_system->bodies) {
        // compute the change in variables
        body.x = dt * body.v;
        body.p = dt * body.force;
        body.a = dt * star(body.omega) * body.a;
        body.l = dt * body.torque;

//        body.a = glm::orthonormalize(body.a);

        // compute change in auxiliary variables
        body.v = body.p * body.shape->get_inv_mass();
        body.i_inv = body.a * body.shape->get_inv_moment_of_inertia() * glm::transpose(body.a);
        body.omega = body.i_inv * body.l;
    }

    std::vector<RigidBody> k3 = body_system->bodies;

    /** k4 */

    // set state to x0 + k3
    for (uint32_t i = 0; i < body_system->bodies.size(); i++) {
        body_system->bodies[i].x = initial_state[i].x + k3[i].x;
        body_system->bodies[i].p = initial_state[i].p + k3[i].p;
        body_system->bodies[i].a = initial_state[i].a + k3[i].a;
        body_system->bodies[i].l = initial_state[i].l + k3[i].l;

        body_system->bodies[i].a = glm::orthonormalize(body_system->bodies[i].a);
        body_system->bodies[i].v = body_system->bodies[i].p * body_system->bodies[i].shape->get_inv_mass();
        body_system->bodies[i].i_inv =
                body_system->bodies[i].a * body_system->bodies[i].shape->get_inv_moment_of_inertia() *
                glm::transpose(body_system->bodies[i].a);
        body_system->bodies[i].omega = body_system->bodies[i].i_inv * body_system->bodies[i].l;
    }

    // calculate hf(x0 + k3, t0 + h)
    for (auto &body : body_system->bodies) {
        // compute the change in variables
        body.x = dt * body.v;
        body.p = dt * body.force;
        body.a = dt * star(body.omega) * body.a;
        body.l = dt * body.torque;

//        body.a = glm::orthonormalize(body.a);

        // compute change in auxiliary variables
        body.v = body.p * body.shape->get_inv_mass();
        body.i_inv = body.a * body.shape->get_inv_moment_of_inertia() * glm::transpose(body.a);
        body.omega = body.i_inv * body.l;
    }

    std::vector<RigidBody> k4 = body_system->bodies;

    /** perform update */

    // update the bodies to time t0 + h
    const double F16 = 1. / 6.;
    const double F13 = 1. / 3.;
    for (uint32_t i = 0; i < body_system->bodies.size(); i++) {
        body_system->bodies[i].x = initial_state[i].x + F16 * k1[i].x + F13 * k2[i].x + F13 * k3[i].x + F16 * k4[i].x;
        body_system->bodies[i].p = initial_state[i].p + F16 * k1[i].p + F13 * k2[i].p + F13 * k3[i].p + F16 * k4[i].p;

        body_system->bodies[i].a = initial_state[i].a + F16 * k1[i].a + F13 * k2[i].a + F13 * k3[i].a + F16 * k4[i].a;
        body_system->bodies[i].l = initial_state[i].l + F16 * k1[i].l + F13 * k2[i].l + F13 * k3[i].l + F16 * k4[i].l;

        body_system->bodies[i].a = glm::orthonormalize(body_system->bodies[i].a);
        body_system->bodies[i].v = body_system->bodies[i].p * body_system->bodies[i].shape->get_inv_mass();
        body_system->bodies[i].i_inv =
                body_system->bodies[i].a * body_system->bodies[i].shape->get_inv_moment_of_inertia() *
                glm::transpose(body_system->bodies[i].a);
        body_system->bodies[i].omega = body_system->bodies[i].i_inv * body_system->bodies[i].l;
    }
}

void Integrator::midpoint(BodySystem *body_system, double dt)
{
    // save the state at t0
    std::vector<RigidBody> initial_state = body_system->bodies;

    // update to time t0 + dt/2
    integrate(body_system, .5 * dt);

    // update to time t0 + h
    for (uint32_t i = 0; i < body_system->bodies.size(); i++) {
        body_system->bodies[i].x = initial_state[i].x + dt * body_system->bodies[i].v;
        body_system->bodies[i].p = initial_state[i].p + dt * body_system->bodies[i].force;

        body_system->bodies[i].a =
                initial_state[i].a + dt * star(body_system->bodies[i].omega) * body_system->bodies[i].a;
        body_system->bodies[i].l = initial_state[i].l + dt * body_system->bodies[i].torque;

        // todo use a quaternion
        body_system->bodies[i].a = glm::orthonormalize(body_system->bodies[i].a);

        // compute auxiliary quantities
        body_system->bodies[i].v =
                body_system->bodies[i].p * body_system->bodies[i].shape->get_inv_mass();
        body_system->bodies[i].i_inv =
                body_system->bodies[i].a * body_system->bodies[i].shape->get_inv_moment_of_inertia() *
                glm::transpose(body_system->bodies[i].a);
        body_system->bodies[i].omega =
                body_system->bodies[i].i_inv * body_system->bodies[i].l;
    }
}

void Integrator::integrate(BodySystem *body_system, double dt)
{
    // perform an Euler step
    for (auto &body : body_system->bodies) {
        // integrate quantities
        body.x += dt * body.v;
        body.p += dt * body.force;

        body.a += dt * star(body.omega) * body.a;
        body.l += dt * body.torque;

        // todo use a quaternion
        body.a = glm::orthonormalize(body.a);

        // compute auxiliary quantities
        body.v = body.p * body.shape->get_inv_mass();
        body.i_inv = body.a * body.shape->get_inv_moment_of_inertia() * glm::transpose(body.a);
        body.omega = body.i_inv * body.l;
    }
}