#ifndef SIMULATION_DRAG_FORCE_HPP
#define SIMULATION_DRAG_FORCE_HPP

#include "force.hpp"

class DragForce : public Force {
private:
    constexpr static const double linear_drag_constant = .6f;
    constexpr static const double angular_drag_constant = .6f;
public:
    DragForce(BodySystem *p_body_system);

    void apply_force_and_torque() override;
};


#endif //SIMULATION_DRAG_FORCE_HPP
