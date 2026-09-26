#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HUFF_MAGIC 0x48
#define HUFF_HEADER_FIXED 11

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

size_t encoded_size(const unsigned char *src, size_t len);

int encode(const unsigned char *src, size_t len, unsigned char *dst,
           size_t dst_cap);

size_t decoded_size(const unsigned char *src, size_t src_len);

size_t decode(const unsigned char *src, size_t src_len, unsigned char *dst,
              size_t dst_cap);
