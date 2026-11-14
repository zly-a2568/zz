#ifndef BODY_H
#define BODY_H
#include "shape.h"
#include "vec.h"
#include "aabb.h"
#include <stddef.h>
typedef  struct body{
    vec2 position;
    vec2 linear_velocity;
    vec3 rotation;
    vec3 angular_velocity;
    float mass;
    float inertia;
    size_t id;
    aabb box;
    shape shape;
}body;

static const body default_body={
    .position=VEC2_ZERO,
    .linear_velocity=VEC2_ZERO,
    .rotation=VEC3_ZERO,
    .angular_velocity=VEC3_ZERO,
    .mass=1.0f,
    .inertia=1.0f,
    .id=0,
    .box={0,1,0,1},
    .shape=default_shape
};
#endif //BODY_H