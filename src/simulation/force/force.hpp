#ifndef SIMULATION_FORCE_HPP
#define SIMULATION_FORCE_HPP

#include "../body_system.hpp"

class BodySystem;

class Force {
protected:
    /** Body system on which this force operates. */
    BodySystem *body_system;
public:
    Force(BodySystem *p_body_system);

    /** Apply the force to all rigid bodies in {body_system}. */
    virtual void apply_force_and_torque() = 0;

    virtual ~Force();
};

#endif //SIMULATION_FORCE_HPP
