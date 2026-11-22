#include <stdlib.h>
#include "vec.h"

static inline bool circle_contact_circle(vec2 c1,vec2 c2,float r1,float r2){
    return VEC2_LENGTH(VEC2_SUB(c1, c2))<=(r1+r2);
}

static inline float point_segment_distance(vec2 p1,vec2 p2,vec2 p){
    /* closest point on segment p1p2 to p, then its distance */
    vec2 e=VEC2_SUB(p2,p1);          /* edge */
    vec2 w=VEC2_SUB(p,p1);
    float ee=VEC2_DOT(e,e);
    float t=(ee>0.0f)? VEC2_DOT(w,e)/ee : 0.0f;
    t=clamp(0.0f,1.0f,t);
    vec2 closest=VEC2_ADD(p1, VEC2_MUL(e,t));
    return VEC2_LENGTH(VEC2_SUB(p,closest));
}
static inline vec2 point_segment_vector(vec2 p1,vec2 p2,vec2 p){
    /* vector from the closest point on segment p1p2 to p */
    vec2 e=VEC2_SUB(p2,p1);
    vec2 w=VEC2_SUB(p,p1);
    float ee=VEC2_DOT(e,e);
    float t=(ee>0.0f)? VEC2_DOT(w,e)/ee : 0.0f;
    t=clamp(0.0f,1.0f,t);
    vec2 closest=VEC2_ADD(p1, VEC2_MUL(e,t));
    return VEC2_SUB(p,closest);
}