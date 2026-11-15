#ifndef AABB_H
#define AABB_H
#include <stdbool.h>
typedef struct aabb{
    float l,r,b,t;
}aabb;

static inline bool aabb_intersect(aabb a,aabb b){
    return (a.l<=b.r && a.r>=b.l)&&(a.b<=b.t&&a.t>=b.b);
}

static inline aabb aabb_combine(aabb a,aabb b){
  return (aabb){a.l<b.l?a.l:b.l,a.r>b.r?a.r:b.r,a.b<b.b?a.b:b.b,a.t>b.t?a.t:b.t};
}

/* does a fully contain b */
static inline bool aabb_contains(aabb a,aabb b){
  return a.l<=b.l && a.r>=b.r && a.b<=b.b && a.t>=b.t;
}

/* surface area heuristic uses the perimeter (2d analogue of surface area) */
static inline float aabb_perimeter(aabb a){
  return 2.0f*((a.r-a.l)+(a.t-a.b));
}

static inline aabb aabb_extend(aabb a,float mx,float my){
  return (aabb){a.l-mx,a.r+mx,a.b-my,a.t+my};
}

#endif //AABB_H
