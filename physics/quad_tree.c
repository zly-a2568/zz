#include "quad_tree.h"
#include <stdbool.h>
#include <stdlib.h>

void quad_tree_init(qtree_node* node,aabb bound,size_t depth){
    array_init(sizeof(body*), &node->data);
    node->boundary=bound;
    node->depth=depth;
    node->divided=false;
}

void quad_tree_divide(qtree_node* node){
    float cx=(node->boundary.l+node->boundary.r)*0.5f;
    float cy=(node->boundary.b+node->boundary.t)*0.5f;
    node->lb=(qtree_node*)malloc(sizeof(qtree_node));
    node->lt=(qtree_node*)malloc(sizeof(qtree_node));
    node->rb=(qtree_node*)malloc(sizeof(qtree_node));
    node->rt=(qtree_node*)malloc(sizeof(qtree_node));
    quad_tree_init(node->lb,(aabb){node->boundary.l,cx,node->boundary.b,cy},node->depth+1);
    quad_tree_init(node->lt, (aabb){node->boundary.l,cx,cy,node->boundary.t},node->depth+1);
    quad_tree_init(node->rb, (aabb){cx,node->boundary.r,node->boundary.b,cy},node->depth+1);
    quad_tree_init(node->rt, (aabb){cy,node->boundary.r,cy,node->boundary.t},node->depth+1);
    node->divided=true;
}

void quad_tree_insert(qtree_node* root,body* b){
    if(!root->divided){
        if (!aabb_intersect(root->boundary, b->box)){
            return;
        }
        if(root->data.size<=2||root->depth>=8){
            array_push(&b,&root->data);
        }
        else{
            quad_tree_divide(root);
            for(size_t i=0;i<root->data.size;i++){
                quad_tree_insert(root->lb,(body*)array_get(i,&root->data));
                quad_tree_insert(root->lt,(body*)array_get(i,&root->data));
                quad_tree_insert(root->rb,(body*)array_get(i,&root->data));
                quad_tree_insert(root->rt,(body*)array_get(i,&root->data));
            }
            array_clear(&root->data);
            quad_tree_insert(root->lb,b);
            quad_tree_insert(root->lt,b);
            quad_tree_insert(root->rb,b);
            quad_tree_insert(root->rt,b);    
        }
    }
    else{
        quad_tree_insert(root->lb,b);
        quad_tree_insert(root->lt,b);
        quad_tree_insert(root->rb,b);
        quad_tree_insert(root->rt,b);
    }
}
