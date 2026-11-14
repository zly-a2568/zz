#ifndef WORLD_H
#define WORLD_H
#include "array.h"
#include "shape.h"

typedef struct{ 
  size_t body_count;
  array bodies;
  array contacts;
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
#endif //WORLD_H
