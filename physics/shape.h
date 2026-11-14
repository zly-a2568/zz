#ifndef SHAPE_H
#define SHAPE_H
#include "vec.h"
#include <stddef.h>

typedef struct circle{
  vec2* p;
  float r;
}circle;
typedef struct segment{
  vec2 p1,p2;
}segment;
typedef struct polygon{
  vec2 vertices[32];
  size_t vertex_count;
}polygon ;

typedef struct shape{
  size_t id;
  circle c;
  segment s;
  polygon p;
}shape;

static const shape default_shape={
  .id = 0,
  .c = {NULL,1.0},
  .s = {VEC2_ZERO,VEC2_ZERO},
  .p = {{{0.0,0.0}},0}
};
#endif //SHAPE_H
