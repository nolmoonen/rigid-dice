#ifndef SIMULATION_SCENE_HPP
#define SIMULATION_SCENE_HPP

#include "body_system.hpp"
#include "force/gravity_force.hpp"
#include "force/drag_force.hpp"

class Scene {
protected:
    /**
     * Private list of shapes which gets cleared upon destruction.
     * All implementations of {initialize} should register the shape
     * pointers to this list to ensure destruction. */
    std::vector<const ShapeWithMass*> shapes;
public:
    /** NB: Initial state has to be collision-free. */
    // todo could throw warning if this is the case
    virtual BodySystem *initialize() = 0;

    virtual ~Scene();
};

class DebugScene: public Scene {
public:
    BodySystem *initialize() override;
};

class DefaultScene : public Scene {
public:
    BodySystem *initialize() override;
};

class ThrowingScene : public Scene {
public:
    BodySystem *initialize() override;
};

class RandomScene : public Scene {
public:
    BodySystem *initialize() override;
};

/** Scene to test whether collisions work that are not vertex-face based. */
class SideWaysCollisionScene : public Scene {
public:
    BodySystem *initialize() override;
};

/** Scene to test whether collisions work that directly parallel to each other. */
class ParallelCollisionScene : public Scene {
public:
    BodySystem *initialize() override;
};

/** {ParallelCollisionScene} but angled in x,z-plane so that multiple edge-edge contacts are created. */
class AngledParallelCollisionScene : public Scene {
public:
    BodySystem *initialize() override;
};

class StableScene : public Scene {
public:
    BodySystem *initialize() override;
};

class StackingScene : public Scene {
public:
    BodySystem *initialize() override;
};

class ContactScene : public Scene {
public:
    BodySystem *initialize() override;
};

#endif //SIMULATION_SCENE_HPP
