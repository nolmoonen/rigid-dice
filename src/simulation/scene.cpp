#include "scene.hpp"

Scene::~Scene()
{
    for (auto &shape : shapes) {
        delete shape;
    }
}

BodySystem *DebugScene::initialize()
{
    auto bs = new BodySystem();

    const double SIZE = 1.;
    const ShapeWithMass *static_cube = new Box(0., SIZE, SIZE, SIZE);
    shapes.emplace_back(static_cube);
    bs->bodies.emplace_back(glm::dvec3(2., 1., 0.), static_cube);

    const ShapeWithMass *static_icosahedron = new Icosahedron(0., SIZE, SIZE, SIZE);
    shapes.emplace_back(static_icosahedron);
    bs->bodies.emplace_back(glm::dvec3(0., 1., 0.), static_icosahedron);

    return bs;
}

BodySystem *DefaultScene::initialize()
{
    auto bs = new BodySystem();

    // create an immovable surface
    const double HEIGHT = .4;
    const ShapeWithMass *surface = new Box(0., 16., HEIGHT, 10.);
    shapes.emplace_back(surface);
    bs->bodies.emplace_back(glm::dvec3(0., -HEIGHT / 2., 0.), surface);

    const double MASS = 3.;
    const double SIZE = 1.;
    const ShapeWithMass *cube = new Icosahedron(1.f / MASS, SIZE, SIZE, SIZE);
    shapes.emplace_back(cube);
    bs->bodies.emplace_back(glm::dvec3(0., 1 + SIZE / 2., 0.), cube);

    // apply gravity
    bs->forces.emplace_back(new GravityForce(bs));

    // apply air resistance
//    bs->forces.emplace_back(new DragForce(bs));

    return bs;
}

BodySystem *ThrowingScene::initialize()
{
    auto bs = new BodySystem();

    // create an immovable surface
    const double HEIGHT = .4;
    const ShapeWithMass *surface = new Box(0., 16., HEIGHT, 10.);
    shapes.emplace_back(surface);
    bs->bodies.emplace_back(glm::dvec3(0., -HEIGHT / 2., 0.), surface);

    const double MASS = 3.;
    const double SIZE = 1.;
    const ShapeWithMass *cube = new Box(1. / MASS, SIZE, SIZE, SIZE);
    shapes.emplace_back(cube);
    // move from negative x to positive x
    bs->bodies.emplace_back(
            glm::dvec3(-10., 6., 0.),
            glm::dmat3(glm::rotate(
                    glm::identity<glm::dmat4>(),
                    (double) M_PI_4,
                    glm::normalize(glm::dvec3(1., 0.f, 1.)))),
            glm::dvec3(6., 6., 0.),
            glm::dvec3(1., 1., 0.),
            cube);

    // apply gravity
    bs->forces.emplace_back(new GravityForce(bs));

    // apply air resistance
//    bs->forces.emplace_back(new DragForce(bs));

    return bs;
}

BodySystem *RandomScene::initialize()
{
    auto bs = new BodySystem();

    // create an immovable surface
    const double HEIGHT = .4;
    const ShapeWithMass *surface = new Box(0., 20., HEIGHT, 35.);
    shapes.emplace_back(surface);
    bs->bodies.emplace_back(glm::dvec3(0., -HEIGHT / 2., 0.), surface);

    const double MASS = 3.;
    const double SIZE = 1.;
    const ShapeWithMass *cube = new Box(1. / MASS, SIZE, SIZE, SIZE);
    shapes.emplace_back(cube);
    bs->bodies.emplace_back(
            glm::dvec3(.0, 1., 0.),
            glm::dmat3(glm::rotate(
                    glm::identity<glm::dmat4>(),
                    (double) std::rand() * (double) M_PI_2 * 2. / (double) RAND_MAX,
                    glm::normalize(glm::dvec3(0., 0., 1.)))),
            cube
    );
    bs->bodies.emplace_back(
            glm::dvec3(.0, 6., 0.),
            glm::dmat3(glm::rotate(
                    glm::identity<glm::dmat4>(),
                    (double) std::rand() * (double) M_PI_2 * 2. / (double) RAND_MAX,
                    glm::normalize(glm::dvec3(0., 1., 0.)))),
            cube
    );
    bs->bodies.emplace_back(
            glm::dvec3(.0, 12., 0.),
            glm::dmat3(glm::rotate(
                    glm::identity<glm::dmat4>(),
                    (double) std::rand() * (double) M_PI_2 * 2. / (double) RAND_MAX,
                    glm::normalize(glm::dvec3(1., 0., 0.)))),
            cube
    );

    // apply gravity
    bs->forces.emplace_back(new GravityForce(bs));

    // apply air resistance
//    bs->forces.emplace_back(new DragForce(bs));

    return bs;
}

BodySystem *SideWaysCollisionScene::initialize()
{
    auto bs = new BodySystem();

    // create two cubes with their edges perpendicular in the x,z-plane
    const double MASS = 3.;
    const double SIZE = 1.;
    const ShapeWithMass *static_cube = new Box(0., SIZE, SIZE, SIZE);
    shapes.emplace_back(static_cube);
    bs->bodies.emplace_back(
            glm::dvec3(0., SIZE / 2., 0.),
            glm::dmat3(glm::rotate(
                    glm::identity<glm::dmat4>(),
                    (double) M_PI_4,
                    glm::normalize(glm::dvec3(0., 0., 1.)))),
            static_cube
    );
    const ShapeWithMass *falling_cube = new Box(1. / MASS, SIZE, SIZE, SIZE);
    shapes.emplace_back(falling_cube);
    bs->bodies.emplace_back(
            glm::dvec3(0., SIZE / 2. + 2., 0.),
            glm::dmat3(glm::rotate(
                    glm::identity<glm::dmat4>(),
                    (double) M_PI_4,
                    glm::normalize(glm::dvec3(1., 0., 0.)))),
            falling_cube
    );

    // apply gravity
    bs->forces.emplace_back(new GravityForce(bs));

    // apply air resistance
//    bs->forces.emplace_back(new DragForce(bs));

    return bs;
}

BodySystem *ParallelCollisionScene::initialize()
{
    auto bs = new BodySystem();

    // create two cube directly above each other
    const double MASS = 3.;
    const double SIZE = 1.;
    const ShapeWithMass *static_cube = new Box(0., SIZE, SIZE, SIZE);
    shapes.emplace_back(static_cube);
    bs->bodies.emplace_back(glm::dvec3(0., SIZE / 2, 0.), static_cube);
    const ShapeWithMass *falling_cube = new Box(1.f / MASS, SIZE, SIZE, SIZE);
    shapes.emplace_back(falling_cube);
    bs->bodies.emplace_back(glm::dvec3(0., SIZE / 2 + 5., 0.), falling_cube);
    // apply gravity
    bs->forces.emplace_back(new GravityForce(bs));

    // apply air resistance
//    bs->forces.emplace_back(new DragForce(bs));

    return bs;
}

BodySystem *AngledParallelCollisionScene::initialize()
{
    auto bs = new BodySystem();

    const double MASS = 3.;
    const double SIZE = 1.;
    const ShapeWithMass *falling_cube = new Box(1. / MASS, SIZE, SIZE, SIZE);
    shapes.emplace_back(falling_cube);
    bs->bodies.emplace_back(
            glm::dvec3(.0, SIZE / 2 + 6., 0.),
            glm::dmat3(glm::rotate(
                    glm::identity<glm::dmat4>(),
                    (double) M_PI_4,
                    glm::normalize(glm::dvec3(0., 1., 0.)))),
            falling_cube
    );
    const ShapeWithMass *static_cube = new Box(0., SIZE, SIZE, SIZE);
    shapes.emplace_back(static_cube);
    bs->bodies.emplace_back(
            glm::dvec3(.0, SIZE / 2 + 1., 0.),
            static_cube
    );

    // apply gravity
    bs->forces.emplace_back(new GravityForce(bs));

    // apply air resistance
//    bs->forces.emplace_back(new DragForce(bs));

    return bs;
}

BodySystem *StableScene::initialize()
{
    auto bs = new BodySystem();

    // create an immovable surface
    const double HEIGHT = .4;
    const ShapeWithMass *surface = new Box(0., 20., HEIGHT, 35.);
    shapes.emplace_back(surface);
    bs->bodies.emplace_back(glm::dvec3(0., -HEIGHT / 2., 0.), surface);

    const double MASS = .1;
    const double SIZE = 1.;
    const ShapeWithMass *falling_cube = new Box(1. / MASS, SIZE, SIZE, SIZE);
    shapes.emplace_back(falling_cube);
    bs->bodies.emplace_back(glm::dvec3(.0, SIZE / 2, 0.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(SIZE * 2., SIZE / 2, 0.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(.0, SIZE / 2, SIZE * 2.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(SIZE * 2., SIZE / 2, SIZE * 2.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(-SIZE * 2., SIZE / 2, 0.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(.0, SIZE / 2, -SIZE * 2.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(-SIZE * 2., SIZE / 2, SIZE * 2.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(-SIZE * 2., SIZE / 2, -SIZE * 2.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(SIZE * 2., SIZE / 2, -SIZE * 2.), falling_cube);

    // apply gravity
    bs->forces.emplace_back(new GravityForce(bs));

    return bs;
}

// todo crashes
BodySystem *StackingScene::initialize()
{
    auto bs = new BodySystem();

    // create an immovable surface
    const double HEIGHT = .4;
    const ShapeWithMass *surface = new Box(0., 20., HEIGHT, 35.);
    shapes.emplace_back(surface);
    bs->bodies.emplace_back(glm::dvec3(0., -HEIGHT / 2., 0.), surface);

    const double MASS = .1;
    const double SIZE = 1.;
    const ShapeWithMass *falling_cube = new Box(1. / MASS, SIZE, SIZE, SIZE);
    shapes.emplace_back(falling_cube);
    bs->bodies.emplace_back(glm::dvec3(.0, 0.5 * SIZE, 0.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(.0, 1.5 * SIZE, 0.), falling_cube);

    // apply gravity
    bs->forces.emplace_back(new GravityForce(bs));

    return bs;
}

BodySystem *ContactScene::initialize()
{
    auto bs = new BodySystem();

    const double MASS = .1;
    const double SIZE = 1.;
    const ShapeWithMass *static_cube = new Box(0., SIZE, SIZE, SIZE);
    shapes.emplace_back(static_cube);
    const ShapeWithMass *falling_cube = new Box(1. / MASS, SIZE, SIZE, SIZE);
    shapes.emplace_back(falling_cube);

    /** first row */

    // parallel on each other
    bs->bodies.emplace_back(glm::dvec3(0., 1.5 * SIZE, 0.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(0., .5 * SIZE, 0.), static_cube);

    // rotated on y-axis
    bs->bodies.emplace_back(glm::dvec3(3., 1.5 * SIZE, 0.),
                            glm::dmat3(glm::rotate(
                                    glm::identity<glm::dmat4>(),
                                    (double) M_PI_4,
                                    glm::normalize(glm::dvec3(0., 1., 0.)))),
                            falling_cube);
    bs->bodies.emplace_back(glm::dvec3(3., .5 * SIZE, 0.), static_cube);

    // rotated on y-axis, first one smaller than the second
    const ShapeWithMass *falling_cube_small = new Box(1. / MASS, .5 * SIZE, .5 * SIZE, .5 * SIZE);
    shapes.emplace_back(falling_cube_small);
    bs->bodies.emplace_back(glm::dvec3(6., 1.25 * SIZE, 0.),
                            glm::dmat3(glm::rotate(
                                    glm::identity<glm::dmat4>(),
                                    (double) M_PI_4,
                                    glm::normalize(glm::dvec3(0., 1., 0.)))),
                            falling_cube_small);
    bs->bodies.emplace_back(glm::dvec3(6., .5 * SIZE, 0.), static_cube);

    // rotated on y-axis, second one smaller than the first
    const ShapeWithMass *static_cube_small = new Box(0., .5 * SIZE, .5 * SIZE, .5 * SIZE);
    shapes.emplace_back(static_cube_small);
    bs->bodies.emplace_back(glm::dvec3(9., 1. * SIZE, 0.),
                            glm::dmat3(glm::rotate(
                                    glm::identity<glm::dmat4>(),
                                    (double) M_PI_4,
                                    glm::normalize(glm::dvec3(0., 1., 0.)))),
                            falling_cube);
    bs->bodies.emplace_back(glm::dvec3(9., .25 * SIZE, 0.), static_cube_small);

    /** second row */

    // rotated on y-axis, small shift in -x and -z
    bs->bodies.emplace_back(glm::dvec3(0. - .3, 1.5 * SIZE, 3. - .3),
                            glm::dmat3(glm::rotate(
                                    glm::identity<glm::dmat4>(),
                                    (double) M_PI_4,
                                    glm::normalize(glm::dvec3(0., 1., 0.)))),
                            falling_cube);
    bs->bodies.emplace_back(glm::dvec3(0., .5 * SIZE, 3.), static_cube);

    // rotated on y-axis, small shift in -x and +z
    bs->bodies.emplace_back(glm::dvec3(3. - .3, 1.5 * SIZE, 3. + .3),
                            glm::dmat3(glm::rotate(
                                    glm::identity<glm::dmat4>(),
                                    (double) M_PI_4,
                                    glm::normalize(glm::dvec3(0., 1., 0.)))),
                            falling_cube);
    bs->bodies.emplace_back(glm::dvec3(3., .5 * SIZE, 3.), static_cube);

    // rotated on y-axis, small shift in +x and -z
    bs->bodies.emplace_back(glm::dvec3(6. + .3, 1.5 * SIZE, 3. - .3),
                            glm::dmat3(glm::rotate(
                                    glm::identity<glm::dmat4>(),
                                    (double) M_PI_4,
                                    glm::normalize(glm::dvec3(0., 1., 0.)))),
                            falling_cube);
    bs->bodies.emplace_back(glm::dvec3(6., .5 * SIZE, 3.), static_cube);

    // rotated on y-axis, small shift in +x and +z
    bs->bodies.emplace_back(glm::dvec3(9. + .3, 1.5 * SIZE, 3. + .3),
                            glm::dmat3(glm::rotate(
                                    glm::identity<glm::dmat4>(),
                                    (double) M_PI_4,
                                    glm::normalize(glm::dvec3(0., 1., 0.)))),
                            falling_cube);
    bs->bodies.emplace_back(glm::dvec3(9., .5 * SIZE, 3.), static_cube);

    /** third row */

    // rotated on y-axis, small shift in -x and -z
    bs->bodies.emplace_back(glm::dvec3(0., .5 * SIZE, 6.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(0. - .3, 1.5 * SIZE, 6. - .3),
                            glm::dmat3(glm::rotate(
                                    glm::identity<glm::dmat4>(),
                                    (double) M_PI_4,
                                    glm::normalize(glm::dvec3(0., 1., 0.)))),
                            static_cube);

    // rotated on y-axis, small shift in -x and +z
    bs->bodies.emplace_back(glm::dvec3(3., .5 * SIZE, 6.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(3. - .3, 1.5 * SIZE, 6. + .3),
                            glm::dmat3(glm::rotate(
                                    glm::identity<glm::dmat4>(),
                                    (double) M_PI_4,
                                    glm::normalize(glm::dvec3(0., 1., 0.)))),
                            static_cube);

    // rotated on y-axis, small shift in +x and -z
    bs->bodies.emplace_back(glm::dvec3(6., .5 * SIZE, 6.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(6. + .3, 1.5 * SIZE, 6. - .3),
                            glm::dmat3(glm::rotate(
                                    glm::identity<glm::dmat4>(),
                                    (double) M_PI_4,
                                    glm::normalize(glm::dvec3(0., 1., 0.)))),
                            static_cube);

    // rotated on y-axis, small shift in +x and +z
    bs->bodies.emplace_back(glm::dvec3(9., .5 * SIZE, 6.), falling_cube);
    bs->bodies.emplace_back(glm::dvec3(9. + .3, 1.5 * SIZE, 6. + .3),
                            glm::dmat3(glm::rotate(
                                    glm::identity<glm::dmat4>(),
                                    (double) M_PI_4,
                                    glm::normalize(glm::dvec3(0., 1., 0.)))),
                            static_cube);

    // apply gravity
    bs->forces.emplace_back(new GravityForce(bs));

    return bs;
}
