#include "drag_force.hpp"

DragForce::DragForce(BodySystem *p_body_system) : Force(p_body_system)
{}

void DragForce::apply_force_and_torque()
{
    for (auto &body : body_system->bodies) {
        body.force -= linear_drag_constant * body.p;
        body.torque -= angular_drag_constant * body.l;
    }
}