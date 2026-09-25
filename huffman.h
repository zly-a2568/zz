#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  unsigned char ch;
  int weight;
  int parent, left, right;
} huffman_node, *huffman_tree;

typedef struct {
  char **item;
  size_t element_size;
  size_t size;
  size_t capacity;
} list;

void init_list(list *l, size_t ele_size);

int list_add(list *l, void *element);

int list_resize(list *l, size_t size);

int list_free(list *l);

void select_node(huffman_tree tree, int end, int *p1, int *p2);

int build_tree(int *w, huffman_tree *tree, int n);

void free_codes(void);

size_t encoded_size(const char *src);

int encode(const char *src, char *dst);
