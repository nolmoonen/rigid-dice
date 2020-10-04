#include "collision_detection.hpp"
#include "contact_derivation.hpp"

bool CollisionDetection::intersect(BodySystem *body_system)
{
    for (uint32_t i = 0; i < body_system->bodies.size(); i++) {
        for (uint32_t j = i + 1; j < body_system->bodies.size(); j++) {
            Collision::IntersectResult inner = Collision::intersect(
                    &body_system->bodies[i], &body_system->bodies[j], -Engine::DISTANCE_THRESHOLD);

            if (inner.intersect) {
                // if a pair of bodies which are translated towards each other with distance Engine::DISTANCE_THRESHOLD
                // intersect, interpenetration has occurred.
                return true;
            }
        }
    }

    return false;
}

CollisionState CollisionDetection::find_collision_state(BodySystem *body_system)
{
    CollisionState return_state = NOT_PENETRATING;

    for (uint32_t i = 0; i < body_system->bodies.size(); i++) {
        for (uint32_t j = i + 1; j < body_system->bodies.size(); j++) {
            Collision::IntersectResult inner = Collision::intersect(
                    &body_system->bodies[i], &body_system->bodies[j], -Engine::DISTANCE_THRESHOLD);

            if (inner.intersect) {
                // if the pair has been translated closer and intersect, they interpenetrate
                return PENETRATING;
            }

            Collision::IntersectResult outer = Collision::intersect(
                    &body_system->bodies[i], &body_system->bodies[j], +Engine::DISTANCE_THRESHOLD);
            if (!outer.intersect) {
                // if the pair has been translated away from each other and do not intersect,
                // they do not have any contact
                continue;
            }

            std::vector<Contact *> this_contacts = ContactDerivation::get_contacts(&inner);
            for (auto &this_contact : this_contacts) {
                // find velocity if it is between small and large
                glm::dvec3 padot = this_contact->body_a->point_velocity(this_contact->p);
                glm::dvec3 pbdot = this_contact->body_b->point_velocity(this_contact->p);

                // find relative velocity
                double vrel = glm::dot(this_contact->n, padot - pbdot);
                if (vrel < Engine::COLLISION_THRESHOLD) {
                    return_state = CONTACT_RESTING_OR_COLLIDING;
                } else {
                    if (return_state != CONTACT_RESTING_OR_COLLIDING) {
                        return_state = CONTACT_SEPARATING;
                    }
                }
            }
        }
    }

    return return_state;
}

std::vector<Contact *> CollisionDetection::find_all_contacts(BodySystem *body_system)
{
    std::vector<Contact *> all_contacts;
    for (uint32_t i = 0; i < body_system->bodies.size(); i++) {
        for (uint32_t j = i + 1; j < body_system->bodies.size(); j++) {
            Collision::IntersectResult inner = Collision::intersect(
                    &body_system->bodies[i], &body_system->bodies[j], -Engine::DISTANCE_THRESHOLD);

            if (inner.intersect) {
                // penetration should not happen
                assert(0);
            }

            Collision::IntersectResult outer = Collision::intersect(
                    &body_system->bodies[i], &body_system->bodies[j], +Engine::DISTANCE_THRESHOLD);
            if (!outer.intersect) {
                // bodies are not in contact
                continue;
            }

            std::vector<Contact *> contacts = ContactDerivation::get_contacts(&inner);
            for (auto &this_contact : contacts) {
                all_contacts.emplace_back(this_contact);
            }
        }
    }

    return all_contacts;
}