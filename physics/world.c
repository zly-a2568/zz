#include "world.h"
#include "shape.h"
#include "vec.h"
#include "body.h"
#include "collision.h"
#include <stddef.h>

world* create_world(){
  world* w=malloc(sizeof(world));
  w->body_count=0;
  array_init(sizeof(body),&w->bodies);
  array_init(sizeof(contact_info),&w->contacts);
  aabb_tree_init(&w->tree);
  return w;
}

void destroy_world(world* w){
  array_free(&w->bodies);
  array_free(&w->contacts);
  aabb_tree_destroy(&w->tree);
  w->body_count=0;
  free(w);
}

size_t world_create_body(world* w){
  body b=default_body;
  b.id=w->body_count;
  b.proxy=AABB_TREE_NULL;
  array_push(&b, &w->bodies);

  size_t index=w->body_count;
  w->body_count++;

  body* stored=(body*)array_get(index, &w->bodies);
  stored->box=body_compute_aabb(stored);
  stored->proxy=aabb_tree_create_proxy(&w->tree, stored->box, index);
  return index;
}

void world_destroy_body(world* w,size_t id){
  if (id>=w->body_count)return;
  body* b=(body*)array_get(id, &w->bodies);
  aabb_tree_destroy_proxy(&w->tree, b->proxy);
  array_remove(id, &w->bodies);
  w->body_count--;
  /* bodies after `id` shifted down one slot: refresh their tree user_data */
  for(size_t i=id;i<w->body_count;i++){
    body* bb=(body*)array_get(i, &w->bodies);
    aabb_tree_set_user_data(&w->tree, bb->proxy, i);
  }
}

typedef struct{
  world* w;
  size_t self;
}broad_phase_ctx;

/* called by the AABB tree for every leaf whose box overlaps the queried box */
static void broad_phase_callback(size_t other, void* ctx){
  broad_phase_ctx* c=(broad_phase_ctx*)ctx;
  if(other<=c->self)return;              /* visit each pair only once */
  body* body_a=(body*)array_get(c->self, &c->w->bodies);
  body* body_b=(body*)array_get(other, &c->w->bodies);
  if(!body_a||!body_b)return;
  contact_info info;
  if(generate_contact_info(body_a, body_b, &info)!=-1){
    array_push(&info, &c->w->contacts);
  }
}

void world_update(world* w,float delta){
  size_t i;
  for(i=0;i<w->bodies.size;i++){
    body* b=(body*)array_get(i, &w->bodies);
    b->position=VEC2_ADD(b->position, VEC2_MUL(b->linear_velocity, delta));
    b->rotation=VEC3_ADD(b->rotation, VEC3_MUL(b->angular_velocity, delta));
  }

  /* broad phase: refresh every proxy, then query overlapping pairs from the tree */
  for(i=0;i<w->bodies.size;i++){
    body* b=(body*)array_get(i, &w->bodies);
    b->box=body_compute_aabb(b);
    aabb_tree_move_proxy(&w->tree, b->proxy, b->box);
  }

  array_clear(&w->contacts);
  for(i=0;i<w->bodies.size;i++){
    body* b=(body*)array_get(i, &w->bodies);
    broad_phase_ctx ctx={w,i};
    aabb_tree_query(&w->tree, b->box, broad_phase_callback, &ctx);
  }

  /* velocity pass: restitution + friction (no position bias) */
  for(int iter=0; iter<CONTACT_ITERATIONS; iter++){
    for(i=0;i<w->contacts.size;i++){
      solve_contact(array_get(i, &w->contacts), delta);
    }
  }

  /* position pass: split impulse. Accumulate pseudo velocities from the
   * overlap, then integrate them into the positions and discard them. */
  for(i=0;i<w->bodies.size;i++){
    body* b=(body*)array_get(i, &w->bodies);
    b->position_bias=VEC2_ZERO;
    b->rotation_bias=0.0f;
  }
  for(int iter=0; iter<CONTACT_POSITION_ITERATIONS; iter++){
    for(i=0;i<w->contacts.size;i++){
      solve_position(array_get(i, &w->contacts), delta);
    }
  }
  for(i=0;i<w->bodies.size;i++){
    body* b=(body*)array_get(i, &w->bodies);
    b->position=VEC2_ADD(b->position, VEC2_MUL(b->position_bias, delta));
    b->rotation.z+=b->rotation_bias*delta;
    b->position_bias=VEC2_ZERO;
    b->rotation_bias=0.0f;
  }
}

void world_set_body_velocity(world* w,size_t id,float x,float y){
  if (id>=w->body_count)return;
  body* b=(body*)array_get(id, &w->bodies);
  b->linear_velocity.x=x;
  b->linear_velocity.y=y;
}

void world_set_body_position(world* w,size_t id,float x,float y){
  if (id>=w->body_count)return;
  body* b=(body*)array_get(id, &w->bodies);
  b->position.x=x;
  b->position.y=y;
}
void world_set_body_mass(world* w,size_t id,float m){
  if (id>=w->body_count)return;
  body* b=(body*)array_get(id, &w->bodies);
  b->mass=m;
  if(b->shape.id&TYPE_CIRCLE){
    b->inertia=0.5f*b->mass*b->shape.c.r*b->shape.c.r;
  }
}

void world_body_apply_impulse(world* w,size_t id,float x,float y,float rx,float ry){
  if (id>=w->body_count)return;
  body* b=(body*)array_get(id, &w->bodies);
  b->linear_velocity=VEC2_ADD(b->linear_velocity,VEC2_MUL((vec2){x,y}, 1/b->mass));
  b->angular_velocity.z+=1/b->inertia*(rx*y-ry*x);
}

vec2 world_get_body_position(world* w,size_t id){
  if (id>=w->body_count)return VEC2_ZERO;
  body* b=(body*)array_get(id, &w->bodies);
  return b->position;
}

float world_get_body_rotation(world*w,size_t id){
  if (id>=w->body_count)return 0.0;
  body* b=(body*)array_get(id, &w->bodies);
  return b->rotation.z;
}

float world_get_body_mass(world*w,size_t id){
  if (id>=w->body_count)return 0.0;
  body* b=(body*)array_get(id, &w->bodies);
  return b->mass;
}

void world_set_body_shape(world* w,size_t id,shape s){
  if (id>=w->body_count)return;
  body* b=(body*)array_get(id, &w->bodies);
  b->shape=s;
  if(s.id&TYPE_CIRCLE){
    b->inertia=0.5f*b->mass*s.c.r*s.c.r;
  }
}