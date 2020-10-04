#ifndef SIMULATION_ENGINE_HPP
#define SIMULATION_ENGINE_HPP

#include <vector>
#include <cassert>
#include <algorithm>
#include <cfloat>

#include "body_system.hpp"
#include "collision_detection.hpp"
#include "scene.hpp"
#include "integrator.hpp"
#include "collision_handling.hpp"

class Engine {
public:
    /** Tolerance in velocity units to decide whether bodies are:
     *  - Resting, their relative velocity is within COLLISION_THRESHOLD and -COLLISION_THRESHOLD.
     *  - Colliding, their relative velocity is less than -COLLISION_THRESHOLD.
     *  - Moving away, their relative velocity is greater than COLLISION_THRESHOLD. */
    static constexpr double const COLLISION_THRESHOLD = .001;

    /**
     * NB: notice the difference with {COLLISION_THRESHOLD}
     * Tolerance in distance units to decide whether bodies are:
     *  - In contact, their distance is within DISTANCE_THRESHOLD and -DISTANCE_THRESHOLD.
     *  - Penetrating, their distance is less than -DISTANCE_THRESHOLD.
     *  - Separate, their distance is greater than DISTANCE_THRESHOLD. */
    static constexpr double const DISTANCE_THRESHOLD = .02;

    /** Warning threshold indicating when error tolerance should be applied. */
    static constexpr double const WARNING_DISTANCE_THRESHOLD = .75 * DISTANCE_THRESHOLD;

    /** Time delta for {update}. */
    double dt = 1. / 60.;

    /** If true, run the simulation at every invocation of Engine::step. */
    bool run = false;

    /** See {ask_to_step_once}. */
    bool step_once = false;

    Scene *scene = new RandomScene();

    BodySystem *body_system = nullptr;

    /** For debugging purposes, maintain a list of intermediate contacts for every step. */
    std::vector<Contact*> prev_contacts;

    ~Engine();

    void init();

    /** Process one step in the simulation. */
    void update();

    /** Pause the simulation. */
    void toggle_run();

    void change_scene(Scene *p_scene);

    /** Reset the simulation. */
    void reset();

    void cleanup() const;

    /**
     * If called when {run} is false, the next call to {step} will complete one step
     * as if {run} is true. Afterwards, it continues normally. */
    void ask_to_step_once();

private:
    /** Progress the body system with a single time step. */
    void step();
};

#endif //SIMULATION_ENGINE_HPP
