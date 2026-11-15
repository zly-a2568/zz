#ifndef SHAPE_H
#define SHAPE_H
#include "vec.h"
#include <stdint.h>
#include <stddef.h>

#define TYPE_CIRCLE 0b001
#define TYPE_SEGMENT 0b010
#define TYPE_POLYGON 0b100

typedef struct circle{
  vec2 p;
  float r;
}circle;
typedef struct segment{
  vec2 p1,p2;//relative position to mass center(position)
}segment;
typedef struct polygon{
  vec2 vertices[32];
  size_t vertex_count;
}polygon ;

typedef struct shape{
  uint8_t id;
  circle c;
  segment s;
  polygon p;
}shape;

static const shape default_shape={
  .id = TYPE_CIRCLE,
  .c = {VEC2_ZERO,1.0},
  .s = {VEC2_ZERO,VEC2_ZERO},
  .p = {{{0.0,0.0}},0}
};
#endif //SHAPE_H
