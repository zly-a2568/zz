#include "aabb_tree.h"
#include <stdlib.h>
#include <stdbool.h>

/* Fatten leaves by this fraction of their size so small motions don't force a
 * re-insertion every frame. */
#define AABB_TREE_MARGIN_FRACTION 0.1f
/* Iterative query stack; the tree is height-balanced so depth stays O(log n). */
#define AABB_TREE_QUERY_STACK 256

static bool node_is_leaf(const aabb_tree* t, size_t id){
    return t->nodes[id].child1 == AABB_TREE_NULL;
}

static int max_int(int a, int b){ return a > b ? a : b; }

static size_t allocate_node(aabb_tree* t){
    if (t->free_list == AABB_TREE_NULL){
        size_t old_capacity = t->capacity;
        size_t new_capacity = old_capacity == 0 ? 16 : old_capacity * 2;
        aabb_tree_node* new_nodes = realloc(t->nodes, new_capacity * sizeof(aabb_tree_node));
        if (!new_nodes) return AABB_TREE_NULL;
        t->nodes = new_nodes;
        t->capacity = new_capacity;
        for (size_t i = old_capacity; i < new_capacity; i++){
            t->nodes[i].height = -1;
            t->nodes[i].next = (i + 1 < new_capacity) ? (i + 1) : AABB_TREE_NULL;
        }
        t->free_list = old_capacity;
    }
    size_t id = t->free_list;
    t->free_list = t->nodes[id].next;
    t->nodes[id].parent = AABB_TREE_NULL;
    t->nodes[id].child1 = AABB_TREE_NULL;
    t->nodes[id].child2 = AABB_TREE_NULL;
    t->nodes[id].next = AABB_TREE_NULL;
    t->nodes[id].height = 0;
    t->nodes[id].user_data = AABB_TREE_NULL;
    t->node_count++;
    return id;
}

static void free_node(aabb_tree* t, size_t id){
    t->nodes[id].height = -1;
    t->nodes[id].next = t->free_list;
    t->free_list = id;
    t->node_count--;
}

/* AVL-style rotation to keep the tree balanced. Returns the new subtree root. */
static size_t balance(aabb_tree* t, size_t iA){
    aabb_tree_node* nodes = t->nodes;
    if (node_is_leaf(t, iA) || nodes[iA].height < 2) return iA;

    size_t iB = nodes[iA].child1;
    size_t iC = nodes[iA].child2;
    int bf = nodes[iC].height - nodes[iB].height;

    if (bf > 1){
        size_t iF = nodes[iC].child1;
        size_t iG = nodes[iC].child2;

        nodes[iC].child1 = iA;
        nodes[iC].parent = nodes[iA].parent;
        nodes[iA].parent = iC;

        if (nodes[iC].parent != AABB_TREE_NULL){
            if (nodes[nodes[iC].parent].child1 == iA) nodes[nodes[iC].parent].child1 = iC;
            else nodes[nodes[iC].parent].child2 = iC;
        } else {
            t->root = iC;
        }

        if (nodes[iF].height > nodes[iG].height){
            nodes[iC].child2 = iF;
            nodes[iA].child2 = iG;
            nodes[iG].parent = iA;
            nodes[iA].box = aabb_combine(nodes[iB].box, nodes[iG].box);
            nodes[iC].box = aabb_combine(nodes[iA].box, nodes[iF].box);
            nodes[iA].height = 1 + max_int(nodes[iB].height, nodes[iG].height);
            nodes[iC].height = 1 + max_int(nodes[iA].height, nodes[iF].height);
        } else {
            nodes[iC].child2 = iG;
            nodes[iA].child2 = iF;
            nodes[iF].parent = iA;
            nodes[iA].box = aabb_combine(nodes[iB].box, nodes[iF].box);
            nodes[iC].box = aabb_combine(nodes[iA].box, nodes[iG].box);
            nodes[iA].height = 1 + max_int(nodes[iB].height, nodes[iF].height);
            nodes[iC].height = 1 + max_int(nodes[iA].height, nodes[iG].height);
        }
        return iC;
    }

    if (bf < -1){
        size_t iD = nodes[iB].child1;
        size_t iE = nodes[iB].child2;

        nodes[iB].child1 = iA;
        nodes[iB].parent = nodes[iA].parent;
        nodes[iA].parent = iB;

        if (nodes[iB].parent != AABB_TREE_NULL){
            if (nodes[nodes[iB].parent].child1 == iA) nodes[nodes[iB].parent].child1 = iB;
            else nodes[nodes[iB].parent].child2 = iB;
        } else {
            t->root = iB;
        }

        if (nodes[iD].height > nodes[iE].height){
            nodes[iB].child2 = iD;
            nodes[iA].child1 = iE;
            nodes[iE].parent = iA;
            nodes[iA].box = aabb_combine(nodes[iC].box, nodes[iE].box);
            nodes[iB].box = aabb_combine(nodes[iA].box, nodes[iD].box);
            nodes[iA].height = 1 + max_int(nodes[iC].height, nodes[iE].height);
            nodes[iB].height = 1 + max_int(nodes[iA].height, nodes[iD].height);
        } else {
            nodes[iB].child2 = iE;
            nodes[iA].child1 = iD;
            nodes[iD].parent = iA;
            nodes[iA].box = aabb_combine(nodes[iC].box, nodes[iD].box);
            nodes[iB].box = aabb_combine(nodes[iA].box, nodes[iE].box);
            nodes[iA].height = 1 + max_int(nodes[iC].height, nodes[iD].height);
            nodes[iB].height = 1 + max_int(nodes[iA].height, nodes[iE].height);
        }
        return iB;
    }

    return iA;
}

/* Insert an already-allocated leaf node into the tree using the
 * surface-area heuristic to pick a sibling. */
static void insert_leaf(aabb_tree* t, size_t leaf){
    if (t->root == AABB_TREE_NULL){
        t->root = leaf;
        t->nodes[leaf].parent = AABB_TREE_NULL;
        return;
    }

    aabb box = t->nodes[leaf].box;

    size_t index = t->root;
    while (!node_is_leaf(t, index)){
        size_t child1 = t->nodes[index].child1;
        size_t child2 = t->nodes[index].child2;

        float area = aabb_perimeter(t->nodes[index].box);
        float combined = aabb_perimeter(aabb_combine(t->nodes[index].box, box));
        float cost = 2.0f * combined;
        float inheritance = 2.0f * (combined - area);

        float cost1;
        if (node_is_leaf(t, child1)){
            cost1 = 2.0f * aabb_perimeter(aabb_combine(box, t->nodes[child1].box)) + inheritance;
        } else {
            float old_area = aabb_perimeter(t->nodes[child1].box);
            float new_area = aabb_perimeter(aabb_combine(box, t->nodes[child1].box));
            cost1 = (new_area - old_area) + inheritance;
        }

        float cost2;
        if (node_is_leaf(t, child2)){
            cost2 = 2.0f * aabb_perimeter(aabb_combine(box, t->nodes[child2].box)) + inheritance;
        } else {
            float old_area = aabb_perimeter(t->nodes[child2].box);
            float new_area = aabb_perimeter(aabb_combine(box, t->nodes[child2].box));
            cost2 = (new_area - old_area) + inheritance;
        }

        if (cost < cost1 && cost < cost2) break;
        index = (cost1 < cost2) ? child1 : child2;
    }

    size_t sibling = index;
    size_t old_parent = t->nodes[sibling].parent;
    size_t new_parent = allocate_node(t);
    if (new_parent == AABB_TREE_NULL) return; /* out of memory: leave leaf detached */

    t->nodes[new_parent].parent = old_parent;
    t->nodes[new_parent].box = aabb_combine(box, t->nodes[sibling].box);
    t->nodes[new_parent].height = t->nodes[sibling].height + 1;
    t->nodes[new_parent].user_data = AABB_TREE_NULL;

    if (old_parent != AABB_TREE_NULL){
        if (t->nodes[old_parent].child1 == sibling) t->nodes[old_parent].child1 = new_parent;
        else t->nodes[old_parent].child2 = new_parent;
    } else {
        t->root = new_parent;
    }

    t->nodes[new_parent].child1 = sibling;
    t->nodes[new_parent].child2 = leaf;
    t->nodes[sibling].parent = new_parent;
    t->nodes[leaf].parent = new_parent;

    /* walk back up fixing heights and AABBs */
    index = t->nodes[leaf].parent;
    while (index != AABB_TREE_NULL){
        index = balance(t, index);
        size_t c1 = t->nodes[index].child1;
        size_t c2 = t->nodes[index].child2;
        t->nodes[index].height = 1 + max_int(t->nodes[c1].height, t->nodes[c2].height);
        t->nodes[index].box = aabb_combine(t->nodes[c1].box, t->nodes[c2].box);
        index = t->nodes[index].parent;
    }
}

/* Detach a leaf from the tree without freeing the leaf node itself. */
static void detach_leaf(aabb_tree* t, size_t leaf){
    if (leaf == t->root){
        t->root = AABB_TREE_NULL;
    } else {
        size_t parent = t->nodes[leaf].parent;
        size_t grand = t->nodes[parent].parent;
        size_t sibling = (t->nodes[parent].child1 == leaf)
                       ? t->nodes[parent].child2 : t->nodes[parent].child1;

        if (grand != AABB_TREE_NULL){
            if (t->nodes[grand].child1 == parent) t->nodes[grand].child1 = sibling;
            else t->nodes[grand].child2 = sibling;
            t->nodes[sibling].parent = grand;
            free_node(t, parent);

            size_t index = grand;
            while (index != AABB_TREE_NULL){
                index = balance(t, index);
                size_t c1 = t->nodes[index].child1;
                size_t c2 = t->nodes[index].child2;
                t->nodes[index].height = 1 + max_int(t->nodes[c1].height, t->nodes[c2].height);
                t->nodes[index].box = aabb_combine(t->nodes[c1].box, t->nodes[c2].box);
                index = t->nodes[index].parent;
            }
        } else {
            t->root = sibling;
            t->nodes[sibling].parent = AABB_TREE_NULL;
            free_node(t, parent);
        }
    }

    t->nodes[leaf].parent = AABB_TREE_NULL;
    t->nodes[leaf].child1 = AABB_TREE_NULL;
    t->nodes[leaf].child2 = AABB_TREE_NULL;
    t->nodes[leaf].height = 0;
}

static aabb fatten(aabb box){
    float mx = (box.r - box.l) * AABB_TREE_MARGIN_FRACTION;
    float my = (box.t - box.b) * AABB_TREE_MARGIN_FRACTION;
    return aabb_extend(box, mx, my);
}

void aabb_tree_init(aabb_tree* t){
    t->nodes = NULL;
    t->capacity = 0;
    t->node_count = 0;
    t->free_list = AABB_TREE_NULL;
    t->root = AABB_TREE_NULL;
}

void aabb_tree_destroy(aabb_tree* t){
    free(t->nodes);
    aabb_tree_init(t);
}

size_t aabb_tree_create_proxy(aabb_tree* t, aabb box, size_t user_data){
    size_t id = allocate_node(t);
    if (id == AABB_TREE_NULL) return AABB_TREE_NULL;
    t->nodes[id].box = fatten(box);
    t->nodes[id].user_data = user_data;
    t->nodes[id].height = 0;
    insert_leaf(t, id);
    return id;
}

void aabb_tree_destroy_proxy(aabb_tree* t, size_t id){
    if (id == AABB_TREE_NULL) return;
    detach_leaf(t, id);
    free_node(t, id);
}

void aabb_tree_move_proxy(aabb_tree* t, size_t id, aabb box){
    if (id == AABB_TREE_NULL) return;
    if (aabb_contains(t->nodes[id].box, box)) return; /* still inside the fat AABB */
    detach_leaf(t, id);
    t->nodes[id].box = fatten(box);
    insert_leaf(t, id);
}

void aabb_tree_set_user_data(aabb_tree* t, size_t id, size_t user_data){
    if (id == AABB_TREE_NULL) return;
    t->nodes[id].user_data = user_data;
}

void aabb_tree_query(const aabb_tree* t, aabb box, aabb_tree_query_fn fn, void* ctx){
    if (t->root == AABB_TREE_NULL) return;

    size_t stack[AABB_TREE_QUERY_STACK];
    size_t sp = 0;
    stack[sp++] = t->root;

    while (sp > 0){
        size_t id = stack[--sp];
        if (id == AABB_TREE_NULL) continue;
        if (!aabb_intersect(t->nodes[id].box, box)) continue;

        if (node_is_leaf(t, id)){
            fn(t->nodes[id].user_data, ctx);
        } else if (sp + 2 <= AABB_TREE_QUERY_STACK){
            stack[sp++] = t->nodes[id].child1;
            stack[sp++] = t->nodes[id].child2;
        }
    }
}
