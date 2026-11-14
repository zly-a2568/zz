#ifndef VEC_H
#define VEC_H
#include <math.h>

typedef struct{
  float x;
  float y;
}vec2;

typedef struct{
  float x;
  float y;
  float z;
}vec3;


static inline vec2 VEC2_ADD(vec2 a, vec2 b) {
    return (vec2){a.x + b.x, a.y + b.y};
}

static inline vec3 VEC3_ADD(vec3 a, vec3 b) {
    return (vec3){a.x + b.x, a.y + b.y,a.z+b.z};
}

static inline vec2 VEC2_SUB(vec2 a, vec2 b) {
    return (vec2){a.x - b.x, a.y - b.y};
}

static inline vec3 VEC3_SUB(vec3 a, vec3 b) {
    return (vec3){a.x - b.x, a.y - b.y,a.z-b.z};
}

static inline vec2 VEC2_MUL(vec2 a, float s) {
    return (vec2){a.x * s, a.y * s};
}

static inline vec3 VEC3_MUL(vec3 a, float s) {
    return (vec3){a.x*s,a.y*s,a.z*s};
}

static inline float VEC2_LENGTH(vec2 a) {
    return sqrtf(a.x * a.x + a.y * a.y);
}

static inline float VEC2_DOT(vec2 a,vec2 b){
    return a.x*b.x+a.y*b.y;
}

static inline float VEC2_CROSS(vec2 a,vec2 b){
    return a.x*b.y-a.y*b.x;
}
/*

PERP:FOLLOW THE CLOCK
 ^
 |  =>
 |      ---->

*/
static inline vec2 VEC2_PERP(vec2 a){
    return (vec2){a.y,-a.x};
}

static inline vec2 VEC2_NORMALIZE(vec2 a) {
    float len = VEC2_LENGTH(a);
    if (len == 0.0f) return (vec2){0.0f, 0.0f}; 
    return (vec2){a.x / len, a.y / len};
}

static inline vec3 VEC3_CROSS(vec3 a,vec3 b){
    return (vec3){a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}

static inline float clamp(float l,float r,float x){
    if(x<l){
        x=l;
    }else if (x>r) {
        x=r;
    }
    return x;
}

static const vec2 VEC2_ZERO=(vec2){0.0,0.0};
static const vec3 VEC3_ZERO=(vec3){0.0,0.0,0.0};

#endif // VEC_H
