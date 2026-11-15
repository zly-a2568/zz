#ifndef AABB_TREE_H
#define AABB_TREE_H
#include "aabb.h"
#include <stddef.h>

/* A dynamic AABB tree (Box2D style).
 * Nodes live in a growable array and refer to each other by index, so the
 * array can be reallocated freely without invalidating anything.
 * Leaves carry a `user_data` value (here: the body index in the world). */

#define AABB_TREE_NULL ((size_t)-1)

typedef struct aabb_tree_node{
    aabb box;          /* fat AABB for leaves, combined AABB for internal nodes */
    size_t parent;
    size_t child1;
    size_t child2;
    size_t next;       /* free-list link */
    size_t user_data;  /* body index, only meaningful for leaves */
    int height;        /* 0 for leaves, -1 for free nodes */
} aabb_tree_node;

typedef struct aabb_tree{
    aabb_tree_node* nodes;
    size_t capacity;
    size_t node_count; /* number of in-use nodes */
    size_t free_list;
    size_t root;
} aabb_tree;

void aabb_tree_init(aabb_tree* t);
void aabb_tree_destroy(aabb_tree* t);

/* Insert a leaf, returns a proxy id (AABB_TREE_NULL on allocation failure). */
size_t aabb_tree_create_proxy(aabb_tree* t, aabb box, size_t user_data);
/* Remove and free a leaf. */
void aabb_tree_destroy_proxy(aabb_tree* t, size_t id);
/* Re-insert a leaf if `box` escaped its current (fat) AABB. */
void aabb_tree_move_proxy(aabb_tree* t, size_t id, aabb box);
void aabb_tree_set_user_data(aabb_tree* t, size_t id, size_t user_data);

typedef void (*aabb_tree_query_fn)(size_t user_data, void* ctx);
/* Call fn(user_data, ctx) for every leaf whose AABB overlaps `box`. */
void aabb_tree_query(const aabb_tree* t, aabb box, aabb_tree_query_fn fn, void* ctx);

#endif //AABB_TREE_H
