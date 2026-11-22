#ifndef WORLD_H
#define WORLD_H
#include "array.h"
#include "shape.h"
#include "aabb_tree.h"
#include <stdbool.h>

/* ---- sleep tuning --------------------------------------------------------
 * A dynamic body whose average speed over a SLEEP_TIME window stays under
 * these thresholds is put to sleep. The test uses net displacement over the
 * window rather than the instantaneous velocity, because a resting body in a
 * stack or pile keeps a small residual velocity that the solver never fully
 * cancels (the body only jitters in place). Sleeping bodies are skipped by
 * integration and contact solving but still collide, so they keep supporting
 * whatever rests on them. Anything touching a sleeping body wakes it (and its
 * whole contact island). */
#ifndef SLEEP_LINEAR_THRESHOLD
#define SLEEP_LINEAR_THRESHOLD  2.0f    /* px/s   */
#endif
#ifndef SLEEP_ANGULAR_THRESHOLD
#define SLEEP_ANGULAR_THRESHOLD 0.5f    /* rad/s  */
#endif
#ifndef SLEEP_TIME
#define SLEEP_TIME              0.5f    /* seconds at rest before sleeping */
#endif

typedef struct{ 
  size_t body_count;
  array bodies;
  array contacts;
  aabb_tree tree;
  /* Scratch for the island sleep/wake pass, kept here so world_update does not
   * allocate every step. sleep_cap is the allocated body count. */
  size_t* sleep_parent;
  unsigned char* sleep_ready;
  size_t sleep_cap;
  /* Contacts from the previous step, as {id_a,id_b} pairs. A contact that is
   * still present is treated as resting (no restitution), which is what lets a
   * stack settle instead of micro-bouncing on the gravity added every step. */
  array prev_contacts;
}world;

world* create_world();
void destroy_world(world* w);
void world_update(world* w,float delta);
void world_set_body_velocity(world* w,size_t id,float x,float y);
void world_set_body_position(world* w,size_t id,float x,float y);
vec2 world_get_body_position(world* w,size_t id);
void world_body_apply_impulse(world* w,size_t id,float x,float y,float rx,float ry);
void world_set_body_shape(world* w,size_t id,shape s);
float world_get_body_rotation(world*w,size_t id);
size_t world_create_body(world* w);
void world_destroy_body(world* w,size_t id);
float world_get_body_mass(world*w,size_t id);
void world_set_body_mass(world* w,size_t id,float m);
void world_set_body_dynamic(world*w,size_t id,bool dynamic);
void world_wake_body(world*w,size_t id);
bool world_is_body_sleeping(world*w,size_t id);
#endif //WORLD_H
