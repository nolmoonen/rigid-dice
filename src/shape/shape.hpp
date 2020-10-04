#ifndef SHAPE_SHAPE_HPP
#define SHAPE_SHAPE_HPP

#include <vector>
#include <utility>

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>
#include <glm/geometric.hpp>

/**
 * Class which defines shapes in 3d, from which {ShapeWithMass} and {Mesh} instances may be created.
 * NB: instances of this class should define a shape which fits in a 1x1x1 cube (or is one).
 * Of which the center is at (0,0,0). */
class Shape {
private:
    /** Vertices, in no particular order. */
    std::vector<glm::dvec3> vertices;

    /** Edges, as indices to {vertices}, in no particular order. */
    std::vector<std::pair<uint32_t, uint32_t>> edges;

    /**
     * Faces, as lists of indices to {vertices}.
     * The faces are in no particular order, but a singular face must be defined in counter-clockwise orientation from
     * outside of the shape (this is to calculate normal and facilitate OpenGL culling). */
    std::vector<std::vector<std::pair<uint32_t, glm::vec2>>> faces;
public:
    Shape(
            std::vector<glm::dvec3> p_vertices,
            std::vector<std::pair<uint32_t, uint32_t>> p_edges,
            std::vector<std::vector<std::pair<uint32_t, glm::vec2>>> p_faces
    );

    const std::vector<glm::dvec3> &get_vertices() const;

    const std::vector<std::pair<uint32_t, uint32_t>> &get_edges() const;

    const std::vector<std::vector<std::pair<uint32_t, glm::vec2>>> &get_faces() const;

    /** Normal pointing outwards. */
    glm::dvec3 get_non_unit_normal(uint32_t face_i) const;

    /** Regular convex polygon with four faces. */
    static const Shape TETRAHEDRON;
    /** Regular convex polygon with six faces. */
    static const Shape CUBE;
    /** Regular convex polygon with eight faces. */
    static const Shape OCTAHEDRON;
    /** Regular convex polygon with twelve faces. */
    static const Shape DODECAHEDRON;
    /** Regular convex polygon with twenty faces. */
    static const Shape ICOSAHEDRON;
};

#endif //SHAPE_SHAPE_HPP
