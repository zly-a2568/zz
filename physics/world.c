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
  return w;
}

void destroy_world(world* w){
  array_free(&w->bodies);
  array_free(&w->contacts);
  w->body_count=0;
  free(w);
}

size_t world_create_body(world* w){
  body b=default_body;
  b.id=w->body_count;
  array_push(&b, &w->bodies);
  w->body_count++;
  return w->body_count-1;
}

void world_destroy_body(world* w,size_t id){
  if (id>=w->body_count)return;
  array_remove(id, &w->bodies);
  w->body_count--;
}

void world_update(world* w,float delta){
  for(int i=0;i<w->bodies.size;i++){
    body* b=array_get(i, &w->bodies);
    b->position=VEC2_ADD(b->position, VEC2_MUL(b->linear_velocity, delta));
    b->rotation=VEC3_ADD(b->rotation, VEC3_MUL(b->angular_velocity, delta));
  }

  array_clear(&w->contacts);
  for(int a=0;a<w->body_count;a++){
    body* body_a=array_get(a, &w->bodies);
    for(int b=a+1;b<w->body_count;b++){
      body* body_b=array_get(b, &w->bodies);
      contact_info info;
      if(generate_contact_info(body_a, body_b, &info)!=-1){
        array_push(&info, &w->contacts);
      }
    }
  }

  for(int k=0;k<w->contacts.size;k++){
    solve_contact(array_get(k, &w->contacts));
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
  if(s.id==0){
    b->inertia=0.5f*b->mass*s.c.r*s.c.r;
    b->shape.c.p=&(b->position);
  }
}