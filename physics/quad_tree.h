#include "array.h"
#include "body.h"

typedef struct qtree_node{
    struct qtree_node *lt,*rt,*lb,*rb;
    size_t depth;
    array data;
    aabb boundary;
    bool divided;
}qtree_node;

