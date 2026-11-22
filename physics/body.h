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
    bool dynamic;
    /* Sleep system: a dynamic body that stays nearly motionless for SLEEP_TIME
     * is put to sleep and skipped by integration and contact solving. It still
     * collides, so it acts as a static support until something wakes it. */
    bool sleeping;
    float sleep_timer;
    /* Position where the current rest window began. The sleep system measures
     * the body's net displacement from here rather than its instantaneous
     * velocity. A resting body in a pile can carry a small residual velocity
     * (the solver never fully cancels the gravity added every step) while
     * merely oscillating in place, so the per-step displacement is a much more
     * reliable "at rest" signal than the velocity. */
    vec2 sleep_anchor_position;
    float sleep_anchor_rotation;
}body;

/* A body can move this step only if it is dynamic and not sleeping. Static and
 * sleeping bodies are treated as immovable by the solver. */
static inline bool body_is_awake(const body* b){
    return b->dynamic && !b->sleeping;
}

/* Inverse mass/inertia as the solver sees them: zero for static and sleeping
 * bodies, so an impulse applied to them produces no motion. */
static inline float body_inv_mass(const body* b){
    return body_is_awake(b) ? 1.0f/b->mass : 0.0f;
}
static inline float body_inv_inertia(const body* b){
    return body_is_awake(b) ? 1.0f/b->inertia : 0.0f;
}

/* Rotate a body-local point into world space (2D, about rotation.z).
 * Segment endpoints and polygon vertices are stored relative to the body
 * centre, so this is how you get their true world position. */
static inline vec2 body_to_world(const body* b, vec2 local){
    float a=b->rotation.z;
    float c=cosf(a), s=sinf(a);
    return (vec2){b->position.x + c*local.x - s*local.y,
                  b->position.y + s*local.x + c*local.y};
}

/* Tight world-space AABB of the body's shape. */
static inline aabb body_compute_aabb(const body* b){
    if (b->shape.id & TYPE_CIRCLE){
        float r = b->shape.c.r;
        return (aabb){b->position.x - r, b->position.x + r,
                      b->position.y - r, b->position.y + r};
    }
    else if (b->shape.id & TYPE_SEGMENT) {
        vec2 p1=body_to_world(b, b->shape.s.p1);
        vec2 p2=body_to_world(b, b->shape.s.p2);
        float min_x=fmin(p1.x,p2.x);
        float min_y=fmin(p1.y,p2.y);
        float max_x=fmax(p1.x,p2.x);
        float max_y=fmax(p1.y,p2.y);
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
    .shape=default_shape,
    .dynamic=true,
    .sleeping=false,
    .sleep_timer=0.0f,
    .sleep_anchor_position=VEC2_ZERO,
    .sleep_anchor_rotation=0.0f
};
#endif //BODY_H