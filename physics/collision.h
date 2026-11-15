#ifndef COLLISION_H
#define COLLISION_H
#include "body.h"
#include "vec.h"

typedef struct contact_info{
    body *body_a;
    body *body_b;
    vec2 normal;
    vec2 tangent;
    vec2 collision_point;
    float depth;
}contact_info ;

int generate_contact_info(body*body_a,body*body_b,contact_info* info);
void solve_contact(contact_info* info, float dt);
void solve_position(contact_info* info, float dt);

/* How many times per step the contact set is re-solved (sequential impulses).
 * More iterations propagate support through a stack, so it stops sinking and
 * stops jittering. */
#ifndef CONTACT_ITERATIONS
#define CONTACT_ITERATIONS 16
#endif
#ifndef CONTACT_POSITION_ITERATIONS
#define CONTACT_POSITION_ITERATIONS 4
#endif
#endif //COLLISION_H