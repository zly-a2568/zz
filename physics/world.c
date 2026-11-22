#include "world.h"
#include "shape.h"
#include "vec.h"
#include "body.h"
#include "collision.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

/* Stable key for a contact: the two body ids in ascending order. */
typedef struct{
  size_t a;
  size_t b;
}contact_pair;

/* Moment of inertia about the body origin, derived from the current shape and
 * mass. Segment endpoints are stored body-local, so integrating |p1+t(p2-p1)|^2
 * over t in [0,1] automatically includes the parallel-axis term when the
 * endpoints are not centred on the origin. For a centred rod this reduces to
 * the usual m*L^2/12. Keeping this in one place stops the mass/shape/dynamic
 * setters from disagreeing about the inertia. */
static float body_compute_inertia(float mass, shape s){
  if(!isfinite(mass) || mass<=0.0f) return INFINITY;
  if(s.id & TYPE_CIRCLE){
    return 0.5f*mass*s.c.r*s.c.r;
  }
  if(s.id & TYPE_SEGMENT){
    vec2 p1=s.s.p1, p2=s.s.p2;
    return mass*(VEC2_DOT(p1,p1)+VEC2_DOT(p1,p2)+VEC2_DOT(p2,p2))/3.0f;
  }
  return 1.0f;
}

world* create_world(){
  world* w=malloc(sizeof(world));
  w->body_count=0;
  array_init(sizeof(body),&w->bodies);
  array_init(sizeof(contact_info),&w->contacts);
  array_init(sizeof(contact_pair),&w->prev_contacts);
  aabb_tree_init(&w->tree);
  w->sleep_parent=NULL;
  w->sleep_ready=NULL;
  w->sleep_cap=0;
  return w;
}

void destroy_world(world* w){
  array_free(&w->bodies);
  array_free(&w->contacts);
  array_free(&w->prev_contacts);
  aabb_tree_destroy(&w->tree);
  free(w->sleep_parent);
  free(w->sleep_ready);
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

/* Wake every sleeping body. Used when a body is removed, because that can take
 * away whatever a sleeping island was resting on. */
static void wake_all(world* w){
  for(size_t i=0;i<w->body_count;i++){
    body* b=(body*)array_get(i, &w->bodies);
    b->sleeping=false;
    b->sleep_timer=0.0f;
  }
}

/* Wake only the sleeping bodies whose AABB overlaps `box`. A body that is moved
 * can drop the support out from under a stack, but it should not disturb
 * islands it never touched. */
typedef struct{ world* w; }wake_ctx;
static void wake_callback(size_t user_data, void* ctx){
  wake_ctx* c=(wake_ctx*)ctx;
  body* b=(body*)array_get(user_data, &c->w->bodies);
  b->sleeping=false;
  b->sleep_timer=0.0f;
}
static void wake_overlapping(world* w, aabb box){
  wake_ctx ctx={w};
  aabb_tree_query(&w->tree, box, wake_callback, &ctx);
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
  wake_all(w);   /* a removed body may have been supporting a sleeping stack */
  array_clear(&w->prev_contacts);   /* ids after `id` shifted down one slot */
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

/* ---- contact persistence ------------------------------------------------
 * A contact that is still present one step later is almost certainly resting,
 * not a fresh impact. Marking it as resting removes its restitution, which
 * stops the micro-bounce a stack would otherwise get from the velocity that
 * gravity adds every step (that bounce never decays and blocks sleeping). */

static void mark_resting_contacts(world* w){
  for(size_t i=0;i<w->contacts.size;i++){
    contact_info* c=(contact_info*)array_get(i,&w->contacts);
    size_t a=c->body_a->id, b=c->body_b->id;
    if(a>b){ size_t t=a; a=b; b=t; }
    c->resting=false;
    for(size_t k=0;k<w->prev_contacts.size;k++){
      contact_pair* p=(contact_pair*)array_get(k,&w->prev_contacts);
      if(p->a==a && p->b==b){ c->resting=true; break; }
    }
  }
}

static void store_prev_contacts(world* w){
  array_clear(&w->prev_contacts);
  for(size_t i=0;i<w->contacts.size;i++){
    contact_info* c=(contact_info*)array_get(i,&w->contacts);
    size_t a=c->body_a->id, b=c->body_b->id;
    if(a>b){ size_t t=a; a=b; b=t; }
    contact_pair p={a,b};
    array_push(&p,&w->prev_contacts);
  }
}

/* ---- island sleep system -------------------------------------------------
 * Bodies joined by contacts form an island. An island is asleep only when
 * every dynamic body in it has been at rest for SLEEP_TIME; any island with a
 * moving body is fully awake. Union-find over the current contact list gives
 * the islands without any per-step allocation. */

static size_t uf_root(size_t* parent, size_t i){
  while(parent[i]!=i){ parent[i]=parent[parent[i]]; i=parent[i]; }
  return i;
}

/* grow the scratch buffers if the body count has outgrown them */
static void sleep_ensure(world* w){
  if(w->sleep_cap<w->body_count){
    w->sleep_parent =(size_t*)realloc(w->sleep_parent, w->body_count*sizeof(size_t));
    w->sleep_ready  =(unsigned char*)realloc(w->sleep_ready, w->body_count*sizeof(unsigned char));
    w->sleep_cap=w->body_count;
  }
}

/* union every contacting pair into one island. Static bodies do not merge
 * islands: otherwise every stack resting on the same floor would become a
 * single island, and waking one stack would wake the whole world. */
static void sleep_build_islands(world* w){
  sleep_ensure(w);
  for(size_t i=0;i<w->body_count;i++) w->sleep_parent[i]=i;
  for(size_t i=0;i<w->contacts.size;i++){
    contact_info* c=(contact_info*)array_get(i,&w->contacts);
    if(!c->body_a->dynamic || !c->body_b->dynamic)continue;
    size_t a=uf_root(w->sleep_parent, c->body_a->id);
    size_t b=uf_root(w->sleep_parent, c->body_b->id);
    if(a!=b) w->sleep_parent[a]=b;
  }
}

/* mark every island that still contains a body which is not at rest */
static void sleep_mark_ready(world* w){
  for(size_t i=0;i<w->body_count;i++) w->sleep_ready[i]=1;
  for(size_t i=0;i<w->body_count;i++){
    body* b=(body*)array_get(i,&w->bodies);
    if(!b->dynamic||b->sleeping)continue;      /* sleeping bodies count as at rest */
    if(b->sleep_timer<SLEEP_TIME) w->sleep_ready[uf_root(w->sleep_parent,i)]=0;
  }
}

/* wake any sleeping body whose island has a mover in it */
static void sleep_wake_islands(world* w){
  sleep_mark_ready(w);
  for(size_t i=0;i<w->body_count;i++){
    body* b=(body*)array_get(i,&w->bodies);
    if(!b->dynamic||!b->sleeping)continue;
    if(!w->sleep_ready[uf_root(w->sleep_parent,i)]){
      b->sleeping=false;
      b->sleep_timer=0.0f;
    }
  }
}

/* put fully-at-rest islands to sleep */
static void sleep_sleep_islands(world* w){
  sleep_mark_ready(w);
  for(size_t i=0;i<w->body_count;i++){
    body* b=(body*)array_get(i,&w->bodies);
    if(!b->dynamic||b->sleeping)continue;
    if(w->sleep_ready[uf_root(w->sleep_parent,i)]){
      b->sleeping=true;
      b->linear_velocity=VEC2_ZERO;
      b->angular_velocity=VEC3_ZERO;
      b->position_bias=VEC2_ZERO;
      b->rotation_bias=0.0f;
    }
  }
}

/* accumulate time at rest, or reset the timer the moment a body starts moving.
 *
 * "At rest" is measured from the body's net displacement since the current
 * rest window began, not from its velocity. The sequential-impulse solver
 * leaves a small residual velocity (it grows with the number of stacked
 * bodies) that the split-impulse position pass largely cancels, so a resting
 * body can carry a non-zero velocity while merely oscillating in place. Over
 * the window we compare the displacement against SLEEP_LINEAR_THRESHOLD *
 * SLEEP_TIME, i.e. the average speed over the window. A body that is really
 * creeping or sliding keeps resetting the anchor; one that only jitters in
 * place does not. */
static void sleep_accumulate(world* w,float delta){
  for(size_t i=0;i<w->body_count;i++){
    body* b=(body*)array_get(i,&w->bodies);
    if(!body_is_awake(b))continue;
    vec2 moved=VEC2_SUB(b->position, b->sleep_anchor_position);
    float turned=b->rotation.z - b->sleep_anchor_rotation;
    if(VEC2_LENGTH(moved)<SLEEP_LINEAR_THRESHOLD*SLEEP_TIME &&
       fabsf(turned)<SLEEP_ANGULAR_THRESHOLD*SLEEP_TIME){
      b->sleep_timer+=delta;
    }else{
      b->sleep_timer=0.0f;
      b->sleep_anchor_position=b->position;
      b->sleep_anchor_rotation=b->rotation.z;
    }
  }
}

void world_update(world* w,float delta){
  size_t i;
  /* integrate only awake dynamic bodies */
  for(i=0;i<w->bodies.size;i++){
    body* b=(body*)array_get(i, &w->bodies);
    if(!body_is_awake(b)){
      /* Static and sleeping bodies are immovable. A sleeping body is kept at
       * exactly zero velocity so callers that add gravity unconditionally
       * (as the demos do) cannot make it accumulate speed while it is asleep. */
      if(b->sleeping){
        b->linear_velocity=VEC2_ZERO;
        b->angular_velocity=VEC3_ZERO;
      }
      continue;
    }
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

  /* Mark each contact as resting if the same pair was already touching last
   * step, then remember this step's pairs for the next one. */
  mark_resting_contacts(w);
  store_prev_contacts(w);

  /* wake whole islands that contain a body which is still moving */
  sleep_build_islands(w);
  sleep_wake_islands(w);

  /* velocity pass: restitution + friction (no position bias). Contacts with no
   * awake body (static-static or sleeping-sleeping) cannot move anything. */
  for(int iter=0; iter<CONTACT_ITERATIONS; iter++){
    for(i=0;i<w->contacts.size;i++){
      contact_info* c=(contact_info*)array_get(i, &w->contacts);
      if(!body_is_awake(c->body_a)&&!body_is_awake(c->body_b))continue;
      solve_contact(c, delta);
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
      contact_info* c=(contact_info*)array_get(i, &w->contacts);
      if(!body_is_awake(c->body_a)&&!body_is_awake(c->body_b))continue;
      solve_position(c, delta);
    }
  }
  for(i=0;i<w->bodies.size;i++){
    body* b=(body*)array_get(i, &w->bodies);
    if(!body_is_awake(b))continue;
    b->position=VEC2_ADD(b->position, VEC2_MUL(b->position_bias, delta));
    b->rotation.z+=b->rotation_bias*delta;
    b->position_bias=VEC2_ZERO;
    b->rotation_bias=0.0f;
  }

  /* update rest timers from how far bodies actually moved, then sleep them */
  sleep_accumulate(w, delta);
  sleep_sleep_islands(w);
}

void world_set_body_velocity(world* w,size_t id,float x,float y){
  if (id>=w->body_count)return;
  body* b=(body*)array_get(id, &w->bodies);
  b->linear_velocity.x=x;
  b->linear_velocity.y=y;
  b->sleeping=false;      /* giving a body motion wakes it */
  b->sleep_timer=0.0f;
}

void world_set_body_position(world* w,size_t id,float x,float y){
  if (id>=w->body_count)return;
  body* b=(body*)array_get(id, &w->bodies);
  aabb old_box=b->box;
  b->position.x=x;
  b->position.y=y;
  b->sleeping=false;      /* the moved body itself is now awake */
  b->sleep_timer=0.0f;
  /* wake anything that was resting where the body used to be (it may have been
   * its support) and anything already at the destination */
  wake_overlapping(w, old_box);
  wake_overlapping(w, body_compute_aabb(b));
}
void world_set_body_mass(world* w,size_t id,float m){
  if (id>=w->body_count)return;
  body* b=(body*)array_get(id, &w->bodies);
  if(!b->dynamic)return;
  b->mass=m;
  b->inertia=body_compute_inertia(b->mass, b->shape);
  b->sleeping=false;
  b->sleep_timer=0.0f;
}

void world_body_apply_impulse(world* w,size_t id,float x,float y,float rx,float ry){
  if (id>=w->body_count)return;
  body* b=(body*)array_get(id, &w->bodies);
  b->linear_velocity=VEC2_ADD(b->linear_velocity,VEC2_MUL((vec2){x,y}, 1/b->mass));
  b->angular_velocity.z+=1/b->inertia*(rx*y-ry*x);
  b->sleeping=false;
  b->sleep_timer=0.0f;
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
  b->inertia=body_compute_inertia(b->mass, s);
  b->sleeping=false;
  b->sleep_timer=0.0f;
}

void world_set_body_dynamic(world*w,size_t id,bool dynamic){
  if (id>=w->body_count)return;
  body* b=(body*)array_get(id, &w->bodies);
  b->dynamic=dynamic;
  b->sleeping=false;
  b->sleep_timer=0.0f;
  if(!dynamic){
    b->mass=INFINITY;
    b->inertia=INFINITY;
    b->angular_velocity=VEC3_ZERO;
    b->linear_velocity=VEC2_ZERO;
  }
  else if(!isfinite(b->mass)||b->mass<=0.0f){
    /* turning a static body back into a dynamic one: restore finite mass */
    b->mass=1.0f;
    b->inertia=body_compute_inertia(b->mass, b->shape);
  }
}

void world_wake_body(world*w,size_t id){
  if (id>=w->body_count)return;
  body* b=(body*)array_get(id, &w->bodies);
  b->sleeping=false;
  b->sleep_timer=0.0f;
}

bool world_is_body_sleeping(world*w,size_t id){
  if (id>=w->body_count)return false;
  body* b=(body*)array_get(id, &w->bodies);
  return b->sleeping;
}