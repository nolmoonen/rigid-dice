#include "body_system.hpp"

BodySystem::~BodySystem()
{
    for (auto &f : forces) {
        delete f;
    }
}