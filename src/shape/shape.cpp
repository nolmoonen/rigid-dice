#include "shape.hpp"

Shape::Shape(
        std::vector<glm::dvec3> p_vertices,
        std::vector<std::pair<uint32_t, uint32_t>> p_edges,
        std::vector<std::vector<std::pair<uint32_t, glm::vec2>>> p_faces
) :
        vertices(std::move(p_vertices)), edges(std::move(p_edges)), faces(std::move(p_faces))
{}

const std::vector<glm::dvec3> &Shape::get_vertices() const
{
    return vertices;
}

const std::vector<std::pair<uint32_t, uint32_t>> &Shape::get_edges() const
{
    return edges;
}

const std::vector<std::vector<std::pair<uint32_t, glm::vec2>>> &Shape::get_faces() const
{
    return faces;
}

glm::dvec3 Shape::get_non_unit_normal(uint32_t face_i) const
{
    glm::dvec3 v1 = vertices[faces[face_i][0].first]; // first  point on face
    glm::dvec3 v2 = vertices[faces[face_i][1].first]; // second point on face
    glm::dvec3 v3 = vertices[faces[face_i][2].first]; // third  point on face
    //    v1
    //     `v2-v3
    return glm::cross(v3 - v2, v1 - v2); // normal pointing outwards from the shape
}

const Shape Shape::TETRAHEDRON(
        std::vector<glm::dvec3>{},
        std::vector<std::pair<uint32_t, uint32_t>>{},
        std::vector<std::vector<std::pair<uint32_t, glm::vec2>>>{}
);

const Shape Shape::CUBE(
        // +         (4) top            -
        // y       3-------5           z
        // ^      /|  (2) /|          ^
        // |     2-------6 | (1)     /
        // o (3) | 0-----|-4        o
        // |     |/ (0)  |/        /
        // v     1-------7        v
        // y        (5) bottom   z
        // - - x < - o - > x +  +

        std::vector<glm::dvec3>{
                glm::dvec3(-.5f, -.5f, -.5f), // 0
                glm::dvec3(-.5f, -.5f, +.5f), // 1
                glm::dvec3(-.5f, +.5f, +.5f), // 2
                glm::dvec3(-.5f, +.5f, -.5f), // 3
                glm::dvec3(+.5f, -.5f, -.5f), // 4
                glm::dvec3(+.5f, +.5f, -.5f), // 5
                glm::dvec3(+.5f, +.5f, +.5f), // 6
                glm::dvec3(+.5f, -.5f, +.5f)  // 7
        },
        std::vector<std::pair<uint32_t, uint32_t>>{
                {0, 1},
                {1, 2},
                {2, 3},
                {3, 0},
                {4, 5},
                {5, 6},
                {6, 7},
                {7, 4},
                {0, 4},
                {1, 7},
                {2, 6},
                {3, 5}
        },
        std::vector<std::vector<std::pair<uint32_t, glm::vec2>>>{
                { // -z (2)
                        {0, glm::vec2(.2f * 2, .25f * 3)}, {3, glm::vec2(.2f * 2, .25f * 4)},
                        {5, glm::vec2(.2f * 1, .25f * 4)}, {4, glm::vec2(.2f * 1, .25f * 3)}},
                { // +x (1)
                        {4, glm::vec2(.2f * 4, .25f * 3)}, {5, glm::vec2(.2f * 4, .25f * 4)},
                        {6, glm::vec2(.2f * 3, .25f * 4)}, {7, glm::vec2(.2f * 3, .25f * 3)}},
                { // +z (0)
                        {7, glm::vec2(.2f * 5, .25f * 3)}, {6, glm::vec2(.2f * 5, .25f * 4)},
                        {2, glm::vec2(.2f * 4, .25f * 4)}, {1, glm::vec2(.2f * 4, .25f * 3)}},
                { // -x (3)
                        {1, glm::vec2(.2f * 3, .25f * 3)}, {2, glm::vec2(.2f * 3, .25f * 4)},
                        {3, glm::vec2(.2f * 2, .25f * 4)}, {0, glm::vec2(.2f * 2, .25f * 3)}},
                { // +y (4)
                        {3, glm::vec2(.2f * 1, .25f * 3)}, {2, glm::vec2(.2f * 1, .25f * 4)},
                        {6, glm::vec2(.2f * 0, .25f * 4)}, {5, glm::vec2(.2f * 0, .25f * 3)}},
                { // -y (5)
                        {1, glm::vec2(.2f * 1, .25f * 2)}, {0, glm::vec2(.2f * 1, .25f * 3)},
                        {4, glm::vec2(.2f * 0, .25f * 3)}, {7, glm::vec2(.2f * 0, .25f * 2)}}
        }
);

const Shape Shape::OCTAHEDRON(
        std::vector<glm::dvec3>{},
        std::vector<std::pair<uint32_t, uint32_t>>{},
        std::vector<std::vector<std::pair<uint32_t, glm::vec2>>>{}
);

const Shape Shape::DODECAHEDRON(
        std::vector<glm::dvec3>{},
        std::vector<std::pair<uint32_t, uint32_t>>{},
        std::vector<std::vector<std::pair<uint32_t, glm::vec2>>>{}
);

const double A = (1.f / ((1.f + sqrtf(5.f)) / 2.f)) / 2.f; // (1.f)
const double B = .5f; // (PHI)
const double D = (1.f - sqrtf(3.f) / 2.f) / 4.f;
const Shape Shape::ICOSAHEDRON(
        // +            2                -
        // y       5 ----- 7            z
        // ^     8 (19) |  (1) 10      ^
        // |    /   3   0     /       /
        // o   9    |       11       o
        // |     4 -|--- 6          /
        // v        1              v
        // y                     z
        // - - x < - o - > x +  +
        std::vector<glm::dvec3>{
                glm::dvec3(0.f, -A, -B), // 0
                glm::dvec3(0.f, -A, +B), // 1
                glm::dvec3(0.f, +A, -B), // 2
                glm::dvec3(0.f, +A, +B), // 3
                glm::dvec3(-A, -B, 0.f), // 4
                glm::dvec3(-A, +B, 0.f), // 5
                glm::dvec3(+A, -B, 0.f), // 6
                glm::dvec3(+A, +B, 0.f), // 7
                glm::dvec3(-B, 0.f, -A), // 8
                glm::dvec3(-B, 0.f, +A), // 9
                glm::dvec3(+B, 0.f, -A), // 10
                glm::dvec3(+B, 0.f, +A)  // 11
        },
        std::vector<std::pair<uint32_t, uint32_t>>{
                {1,  3},
                {4,  6},
                {11, 10},
                {8,  9},
                {2,  0},
                {5,  7},
                {9,  3},
                {9,  1},
                {11, 3},
                {11, 1},
                {10, 2},
                {10, 0},
                {8,  2},
                {8,  0},
                {5,  8},
                {5,  9},
                {4,  8},
                {4,  9},
                {7,  10},
                {7,  11},
                {6,  10},
                {6,  11},
                {3,  5},
                {3,  7},
                {2,  5},
                {2,  7},
                {1,  4},
                {1,  6},
                {0,  4},
                {0,  6},
        },
        //      2
        //    number
        //  1       0
        std::vector<std::vector<std::pair<uint32_t, glm::vec2>>>{
                { // (1)
                        {10, glm::vec2(.2f * 0, .25f * 3 + D)},
                        {0,  glm::vec2(.2f * 1, .25f * 3 + D)},
                        {2,  glm::vec2(.1f + .2f * 0, .25f * 4)},
                },
                { // (2)
                        {3,  glm::vec2(.2f * 1, .25f * 3 + D)},
                        {1,  glm::vec2(.2f * 2, .25f * 3 + D)},
                        {11, glm::vec2(.1f + .2f * 1, .25f * 4)},
                },
                { // (3)
                        {2,  glm::vec2(.2f * 2, .25f * 3 + D)},
                        {8,  glm::vec2(.2f * 3, .25f * 3 + D)},
                        {5,  glm::vec2(.1f + .2f * 2, .25f * 4)},
                },
                { // (4)
                        {1,  glm::vec2(.2f * 3, .25f * 3 + D)},
                        {4,  glm::vec2(.2f * 4, .25f * 3 + D)},
                        {6,  glm::vec2(.1f + .2f * 3, .25f * 4)},
                },
                { // (5)
                        {6,  glm::vec2(.2f * 4, .25f * 3 + D)},
                        {10, glm::vec2(.2f * 5, .25f * 3 + D)},
                        {11, glm::vec2(.1f + .2f * 4, .25f * 4)},
                },
                { // (6)
                        {4,  glm::vec2(.2f * 0, .25f * 2 + D)},
                        {9,  glm::vec2(.2f * 1, .25f * 2 + D)},
                        {8,  glm::vec2(.1f + .2f * 0, .25f * 3)},
                },
                { // (7)
                        {10, glm::vec2(.2f * 1, .25f * 2 + D)},
                        {2,  glm::vec2(.2f * 2, .25f * 2 + D)},
                        {7,  glm::vec2(.1f + .2f * 1, .25f * 3)},
                },
                { // (8)
                        {9,  glm::vec2(.2f * 2, .25f * 2 + D)},
                        {3,  glm::vec2(.2f * 3, .25f * 2 + D)},
                        {5,  glm::vec2(.1f + .2f * 2, .25f * 3)},
                },
                { // (9)
                        {8,  glm::vec2(.2f * 3, .25f * 2 + D)},
                        {0,  glm::vec2(.2f * 4, .25f * 2 + D)},
                        {4,  glm::vec2(.1f + .2f * 3, .25f * 3)},
                },
                { // (10)
                        {5,  glm::vec2(.2f * 4, .25f * 2 + D)},
                        {3,  glm::vec2(.2f * 5, .25f * 2 + D)},
                        {7,  glm::vec2(.1f + .2f * 4, .25f * 3)},
                },
                { // (11)
                        {0,  glm::vec2(.2f * 0, .25f * 1 + D)},
                        {6,  glm::vec2(.2f * 1, .25f * 1 + D)},
                        {4,  glm::vec2(.1f + .2f * 0, .25f * 2)},
                },
                { // (12)
                        {3,  glm::vec2(.2f * 1, .25f * 1 + D)},
                        {11, glm::vec2(.2f * 2, .25f * 1 + D)},
                        {7,  glm::vec2(.1f + .2f * 1, .25f * 2)},
                },
                { // (13)
                        {0,  glm::vec2(.2f * 2, .25f * 1 + D)},
                        {10, glm::vec2(.2f * 3, .25f * 1 + D)},
                        {6,  glm::vec2(.1f + .2f * 2, .25f * 2)},
                },
                { // (14)
                        {1,  glm::vec2(.2f * 3, .25f * 1 + D)},
                        {9,  glm::vec2(.2f * 4, .25f * 1 + D)},
                        {4,  glm::vec2(.1f + .2f * 3, .25f * 2)},
                },
                { // (15)
                        {10, glm::vec2(.2f * 4, .25f * 1 + D)},
                        {7,  glm::vec2(.2f * 5, .25f * 1 + D)},
                        {11, glm::vec2(.1f + .2f * 4, .25f * 2)},
                },
                { // (16)
                        {9,  glm::vec2(.2f * 0, .25f * 0 + D)},
                        {5,  glm::vec2(.2f * 1, .25f * 0 + D)},
                        {8,  glm::vec2(.1f + .2f * 0, .25f * 1)},
                },
                { // (17)
                        {7,  glm::vec2(.2f * 1, .25f * 0 + D)},
                        {2,  glm::vec2(.2f * 2, .25f * 0 + D)},
                        {5,  glm::vec2(.1f + .2f * 1, .25f * 1)},
                },
                { // (18)
                        {11, glm::vec2(.2f * 2, .25f * 0 + D)},
                        {1,  glm::vec2(.2f * 3, .25f * 0 + D)},
                        {6,  glm::vec2(.1f + .2f * 2, .25f * 1)},
                },
                { // (19)
                        {2,  glm::vec2(.2f * 3, .25f * 0 + D)},
                        {0,  glm::vec2(.2f * 4, .25f * 0 + D)},
                        {8,  glm::vec2(.1f + .2f * 3, .25f * 1)},
                },
                { // (20)
                        {3,  glm::vec2(.2f * 4, .25f * 0 + D)},
                        {9,  glm::vec2(.2f * 5, .25f * 0 + D)},
                        {1,  glm::vec2(.1f + .2f * 4, .25f * 1)},
                },
        }
);
