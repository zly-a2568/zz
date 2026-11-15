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
    /* Split-impulse pseudo velocities. They are only used to recover overlap
     * and are never fed back into linear/angular velocity, so position recovery
     * cannot inject energy into the simulation. */
    vec2 position_bias;
    float rotation_bias;
    float mass;
    float inertia;
    size_t id;
    size_t proxy;   /* broad-phase proxy id in the world's AABB tree */
    aabb box;
    shape shape;
}body;

/* Tight world-space AABB of the body's shape. Only circles are supported so far. */
static inline aabb body_compute_aabb(const body* b){
    if (b->shape.id & TYPE_CIRCLE){
        float r = b->shape.c.r;
        return (aabb){b->position.x - r, b->position.x + r,
                      b->position.y - r, b->position.y + r};
    }
    else if (b->shape.id & TYPE_SEGMENT) {
        float min_x=fmin(b->shape.s.p1.x,b->shape.s.p2.x);
        float min_y=fmin(b->shape.s.p1.y,b->shape.s.p2.y);
        float max_x=fmax(b->shape.s.p1.x,b->shape.s.p2.x);
        float max_y=fmax(b->shape.s.p1.y,b->shape.s.p2.y);
        return (aabb){min_x-10.0f,max_x+10.0f,min_y-10.0f,max_y+10.0f};
    }
    return (aabb){b->position.x, b->position.x, b->position.y, b->position.y};
}

static const body default_body={
    .position=VEC2_ZERO,
    .linear_velocity=VEC2_ZERO,
    .rotation=VEC3_ZERO,
    .angular_velocity=VEC3_ZERO,
    .position_bias=VEC2_ZERO,
    .rotation_bias=0.0f,
    .mass=1.0f,
    .inertia=1.0f,
    .id=0,
    .proxy=(size_t)-1,
    .box={0,1,0,1},
    .shape=default_shape
};
#endif //BODY_H