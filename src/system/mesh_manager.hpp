#ifndef SYSTEM_MESH_MANAGER_HPP
#define SYSTEM_MESH_MANAGER_HPP

#include "opengl.hpp"
#include "manager.hpp"
#include "../shape/shape.hpp"

#define MESH_TETRAHEDRON 0
#define MESH_CUBE 1
#define MESH_OCTAHEDRON 2
#define MESH_DODECAHEDRON 3
#define MESH_ICOSAHEDRON 4

#define MESH_SKYBOX 5

/** Hack to enable the use of the manager class. */
uint32_t body_pointer_to_id(Shape const *body);

class MeshManager : public Manager<Mesh> {
private:
    // todo find better solution and combine with ShaderResource?
    typedef struct {
        uint32_t id;
        /** Pointer to creation data (not in embedded data). */
        const Shape *body;
        void (*create_function)(Mesh *);
    } MeshResource;

    /** Array to obtain the desired data using an id. */
    static const MeshResource MESH_RESOURCES[];

    int32_t create_item(Mesh *item, uint32_t id) override;

    void delete_item(Mesh *item, uint32_t id) override;

public:
    ~MeshManager() override;
};

#endif //SYSTEM_MESH_MANAGER_HPP
