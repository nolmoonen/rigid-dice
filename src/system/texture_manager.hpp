#ifndef SYSTEM_TEXTURE_MANAGER_HPP
#define SYSTEM_TEXTURE_MANAGER_HPP

#include <cstdint>

#include "opengl.hpp"
#include "manager.hpp"

#define TEXTURE_GRASS 0
#define TEXTURE_TEST 1
#define TEXTURE_DICE 2
#define TEXTURE_SKYBOX 3
#define TEXTURE_WOOD 4

/** Specified manually based on identifiers in CMakeLists. */
extern const char grass_png[];
extern const size_t grass_png_len;

extern const char test_png[];
extern const size_t test_png_len;

extern const char dice_png[];
extern const size_t dice_png_len;

extern const char skybox_png[];
extern const size_t skybox_png_len;

extern const char wood_png[];
extern const size_t wood_png_len;

class TextureManager : public Manager<Texture> {
private:
    // todo find better solution and combine with ShaderResource?
    typedef struct {
        uint32_t id;
        uint32_t channels;
        /** Pointers to extern embedded data. */
        const char *text;
        const size_t *len;
    } TextureResource;

    /** Array to obtain the desired data using an id. */
    static const TextureResource TEXTURE_RESOURCES[];

    int32_t create_item(Texture *item, uint32_t id) override;

    void delete_item(Texture *item, uint32_t id) override;

public:
    ~TextureManager() override;
};

#endif //SYSTEM_TEXTURE_MANAGER_HPP
