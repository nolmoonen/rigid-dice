#include "engine.hpp"

Engine::~Engine()
{
    cleanup();
}

void Engine::init()
{
    body_system = scene->initialize();
}

void Engine::update()
{
    if (!run && !step_once) return;

    // perform an integration step
    step();

    step_once = false;
}

void Engine::toggle_run()
{
    run = !run;
}

void Engine::change_scene(Scene *p_scene)
{
    delete scene;
    this->scene = p_scene;
    this->reset();
}

void Engine::reset()
{
    cleanup();
    init();
}

void Engine::cleanup() const
{
    delete body_system;
}

void Engine::ask_to_step_once()
{
    step_once = true;
}

bool equal(RigidBody *x, RigidBody *y)
{
    if (x->x != y->x) return false;
    if (x->p != y->p) return false;

    if (x->a != y->a) return false;
    if (x->l != y->l) return false;

    if (x->v != y->v) return false;
    if (x->i_inv != y->i_inv) return false;
    if (x->omega != y->omega) return false;

    if (x->force != y->force) return false;
    if (x->torque != y->torque) return false;

    return true;
}

void Engine::step()
{
    for (auto &prev_contact : prev_contacts) {
        delete prev_contact;
    }
    prev_contacts.clear();

    double t_current = 0.;
    while (t_current < dt) {
        double t_target = dt - t_current;
//        CollisionHandling::correct_state(body_system); // todo debug
        std::vector<Contact *> contacts = CollisionDetection::find_all_contacts(body_system);

        bool had_collision;
        do {
            had_collision = CollisionHandling::find_all_collisions(contacts);
        } while (had_collision);

        Integrator::clear_forces(body_system);
        Integrator::apply_forces(body_system);
        CollisionHandling::compute_contact_forces(contacts);

        std::vector<RigidBody> bodies_t0 = body_system->bodies;
        Integrator::runge_kutta_4(body_system, t_target);
        if (!CollisionDetection::intersect(body_system)) {
            // free the contact list
            for (auto &contact : contacts) {
                prev_contacts.emplace_back(contact);
            }
            return;
        }

        double t = t_target * .5;
        double t_step = t_target * .5;
        bool searching = true;
        while (searching) {
            // restore state
            body_system->bodies = bodies_t0;

            // apply step
            Integrator::runge_kutta_4(body_system, t);

            // calculate whether the current time is correct
            CollisionState state = CollisionDetection::find_collision_state(body_system);
            switch (state) {
                case PENETRATING:
                    // we are too deep, step back (we are interpenetrating)
                    t_step *= .5;
                    t = t - t_step;
                    break;
                case CONTACT_RESTING_OR_COLLIDING:
                    // t_c = t
                    searching = false;
                    break;
                case CONTACT_SEPARATING:
                case NOT_PENETRATING:
                    // we are too far out, step forward (we are not even in contact anymore)
                    t_step *= .5;
                    t = t + t_step;
                    break;
            }
            if (t_step == 0.) {
                printf("cannot find time of collision\n"); // todo debug
            }
        }

        t_current += t;

        // free the contact list
        for (auto &contact : contacts) {
            prev_contacts.emplace_back(contact);
        }

        bool change = false;
        for (uint32_t i = 0; i < bodies_t0.size(); i++) {
            if (!equal(&bodies_t0[i], &body_system->bodies[i])) {
                change = true;
            }
        }

        if (!change) {
            printf("nonprogress\n"); // todo debug
        }
    }
}