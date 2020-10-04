#ifndef SIMULATION_COLLISION_STATE_HPP
#define SIMULATION_COLLISION_STATE_HPP

enum CollisionState {
    /** There is at least one pair of bodies penetrating with a distance more than -DISTANCE_THRESHOLD. */
    PENETRATING,
    /**
     * There is no pair of bodies penetrating with a distance more than -DISTANCE_THRESHOLD.
     *
     * There is a pair of bodies in contact (so within -DISTANCE_THRESHOLD and DISTANCE_THRESHOLD),
     * and relative velocity is smaller than +COLLISION_THRESHOLD (therefore going to collide or resting). */
    CONTACT_RESTING_OR_COLLIDING,
    /**
     * There is no pair of bodies penetrating with a distance more than -DISTANCE_THRESHOLD.
     *
     * There is a pair of bodies in contact (so within -DISTANCE_THRESHOLD and DISTANCE_THRESHOLD),
     * and relative velocity is larger than +COLLISION_THRESHOLD (therefore moving apart). */
    CONTACT_SEPARATING,
    /** All bodies are more than DISTANCE_THRESHOLD apart. */
    NOT_PENETRATING
};

#endif //SIMULATION_COLLISION_STATE_HPP
