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
void solve_contact(contact_info* info);
#endif //COLLISION_H