#ifndef AABB_H
#define AABB_H
#include <stdbool.h>
typedef struct aabb{
    float l,r,b,t;
}aabb;

static inline bool aabb_intersect(aabb a,aabb b){
    return (a.l<=b.r && a.r>=b.l)&&(a.b<=b.t&&a.t>=b.b);
}

#endif //AABB_H