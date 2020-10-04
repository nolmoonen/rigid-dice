#include "mesh_manager.hpp"

uint32_t body_pointer_to_id(Shape const *body)
{
    if (body == &Shape::TETRAHEDRON) {
        return MESH_TETRAHEDRON;
    } else if (body == &Shape::CUBE) {
        return MESH_CUBE;
    } else if (body == &Shape::OCTAHEDRON) {
        return MESH_OCTAHEDRON;
    } else if (body == &Shape::DODECAHEDRON) {
        return MESH_DODECAHEDRON;
    } else if (body == &Shape::ICOSAHEDRON) {
        return MESH_ICOSAHEDRON;
    }

    // todo properly handle?
    nm_log::log(LOG_ERROR, "unknown body pointer\n");
    assert(0);
    return -1;
}

// provide a default destructor for the base class
template<>
inline Manager<Mesh>::~Manager<Mesh>() = default;

const MeshManager::MeshResource MeshManager::MESH_RESOURCES[] = {
        {.id=MESH_TETRAHEDRON, .body=&Shape::TETRAHEDRON},
        {.id=MESH_CUBE, .body=&Shape::CUBE},
        {.id=MESH_OCTAHEDRON, .body=&Shape::OCTAHEDRON},
        {.id=MESH_DODECAHEDRON, .body=&Shape::DODECAHEDRON},
        {.id=MESH_ICOSAHEDRON, .body=&Shape::ICOSAHEDRON},
        {.id=MESH_SKYBOX, .body=nullptr, .create_function=&Mesh::create_skybox},
};

int32_t MeshManager::create_item(Mesh *item, uint32_t id)
{
    // find index of program shader
    int64_t prog_index = -1;
    for (uint32_t i = 0; i < sizeof(MESH_RESOURCES) / sizeof(MeshResource); i++) {
        if (MESH_RESOURCES[i].id == id) {
            prog_index = i;
            break;
        }
    }
    if (prog_index == -1) {
        nm_log::log(LOG_ERROR, "\"%d\" is not a registered texture id\n", id);

        return EXIT_FAILURE;
    }

    if (MESH_RESOURCES[prog_index].body != nullptr) {
        // if the body pointer exists, create from body
        Mesh::create_mesh_from_body(item, MESH_RESOURCES[prog_index].body);
    } else {
        // else, create from function pointer
        MESH_RESOURCES[prog_index].create_function(item);
    }

    nm_log::log(LOG_INFO, "created mesh with id \"%d\"\n", id);

    return EXIT_SUCCESS;
}

void MeshManager::delete_item(Mesh *item, uint32_t id)
{
    Mesh::delete_mesh(item);

    nm_log::log(LOG_INFO, "deleted mesh with id \"%d\"\n", id);
}

MeshManager::~MeshManager()
{
    for (auto &item : map) {
        delete_item(&item.second.item, item.first);
    }
}
