#include <stdlib.h>
#include "vec.h"

static inline bool circle_contact_circle(vec2 c1,vec2 c2,float r1,float r2){
    return VEC2_LENGTH(VEC2_SUB(c1, c2))<=(r1+r2);
}

static inline float point_segment_distance(vec2 p1,vec2 p2,vec2 p){
    vec2 p1_p2=VEC2_SUB(p2, p1);
    vec2 p1_p=VEC2_SUB(p, p1);
    float dot = VEC2_DOT(p1_p2, p1_p);
    if (dot<0){
        return VEC2_LENGTH(p1_p);
    }
    else if(dot<=VEC2_LENGTH(p1_p2)){
        return abs(VEC2_CROSS(p1_p, p1_p2))/VEC2_LENGTH(p1_p2);
    }
    else{
        return VEC2_LENGTH(VEC2_SUB(p, p2));
    }

}
static inline vec2 point_segment_vector(vec2 p1,vec2 p2,vec2 p){
    vec2 p1_p2=VEC2_SUB(p2, p1);
    vec2 p1_p=VEC2_SUB(p, p1);
    float dot = VEC2_DOT(p1_p2, p1_p);
    if (dot<0){
        return p1_p;
    }
    else if(dot<=VEC2_LENGTH(p1_p2)){
        float d = VEC2_CROSS(p1_p, p1_p2);
        if(d<0)return VEC2_MUL(VEC2_NORMALIZE((vec2){-p1_p2.y,p1_p2.x}),d);
        else if(d>0)return VEC2_MUL(VEC2_NORMALIZE((vec2){p1_p2.y,-p1_p2.x}),d);
        else return VEC2_ZERO;
    }
    else{
        return VEC2_SUB(p, p2);
    }

}