#include "huffman.h"

char *codes[256];

void swap(int *a, int *b) {
  int temp = *a;
  *a = *b;
  *b = temp;
}

void init_list(list *l, size_t ele_size) {
  l->item = NULL;
  l->size = 0;
  l->element_size = ele_size;
  l->capacity = 0;
}

int list_add(list *l, void *element) {
  if (l->capacity <= l->size) {
    list_resize(l, l->capacity > 0 ? l->capacity * 2 : 16);
  }
  l->item[l->size++] = element;
  return 0;
}

int list_resize(list *l, size_t size) {
  if (l->size >= size) {
    return 0;
  } else {
    int new_capacity = size;
    char **new_items =
        (char **)realloc(l->item, new_capacity * l->element_size);
    if (new_items == NULL)
      return -1;
    l->item = new_items;
    l->capacity = new_capacity;
  }
  return 0;
}

int list_free(list *l) {
  if (l != NULL) {
    free(l->item);
    free(l);
  }
  return 0;
}

int partition(int arr[], int low, int high) {
  int pivot = arr[high];
  int i = low - 1;

  for (int j = low; j < high; j++) {
    if (arr[j] <= pivot) {
      i++;
      swap(&arr[i], &arr[j]);
    }
  }

  swap(&arr[i + 1], &arr[high]);
  return i + 1;
}

void quick_sort(int arr[], int low, int high) {
  if (low < high) {
    int pi = partition(arr, low, high);

    quick_sort(arr, low, pi - 1);
    quick_sort(arr, pi + 1, high);
  }
}

void select_node(huffman_tree tree, int end, int *p1, int *p2) {
  int min1, min2;
  int i = 1, j;
  while (i <= end && tree[i].parent != 0) {
    i++;
  }
  if (i > end)
    return;
  *p1 = i;
  min1 = tree[i].weight;
  i++;
  while (i <= end && tree[i].parent != 0) {
    i++;
  }
  if (i > end) {
    *p2 = 0;
    return;
  }
  if (tree[i].weight < min1) {
    min2 = min1;
    *p2 = *p1;
    min1 = tree[i].weight;
    *p1 = i;
  } else {
    min2 = tree[i].weight;
    *p2 = i;
  }
  for (j = i + 1; j <= end; j++) {
    if (tree[j].parent != 0)
      continue;
    if (tree[j].weight < min1) {
      min2 = min1;
      min1 = tree[j].weight;
      *p2 = *p1;
      *p1 = j;
    } else if (tree[j].weight >= min1 && tree[j].weight < min2) {
      min2 = tree[j].weight;
      *p2 = j;
    }
  }
}

int build_tree(int *w, huffman_tree *tree, int n) {
  int m, i;
  if (n <= 0) {
    *tree = NULL;
    return 0;
  }
  m = 2 * n - 1;
  *tree = (huffman_tree)malloc((m + 1) * sizeof(huffman_node));
  if (*tree == NULL)
    return -1;
  huffman_tree p = *tree;
  for (i = 1; i <= n; i++) {
    (p + i)->weight = *(w + i - 1);
    (p + i)->ch = (unsigned char)(i - 1);
    (p + i)->parent = 0;
    (p + i)->left = 0;
    (p + i)->right = 0;
  }
  for (i = n + 1; i <= m; i++) {
    (p + i)->weight = 0;
    (p + i)->parent = 0;
    (p + i)->left = 0;
    (p + i)->right = 0;
  }
  for (i = n + 1; i <= m; i++) {
    int p1, p2;
    select_node(*tree, i - 1, &p1, &p2);
    (*tree)[p1].parent = (*tree)[p2].parent = i;
    (*tree)[i].left = p1;
    (*tree)[i].right = p2;
    (*tree)[i].weight = (*tree)[p1].weight + (*tree)[p2].weight;
  }
  return 0;
}

static char *dup_str(const char *s) {
  size_t len = strlen(s) + 1;
  char *copy = (char *)malloc(len);
  if (copy != NULL)
    memcpy(copy, s, len);
  return copy;
}

void free_codes(void) {
  for (int i = 0; i < 256; i++) {
    free(codes[i]);
    codes[i] = NULL;
  }
}

static void build_codes(huffman_tree tree, int idx, char *path, int depth) {
  huffman_node *node = tree + idx;
  if (node->left == 0 && node->right == 0) {
    path[depth] = '\0';
    codes[node->ch] = dup_str(path);
    return;
  }
  path[depth] = '0';
  build_codes(tree, node->left, path, depth + 1);
  path[depth] = '1';
  build_codes(tree, node->right, path, depth + 1);
}

static int prepare_codes(const char *src, huffman_tree *tree) {
  int weights[256];
  int cut_weights[256];
  unsigned char chars[256];
  int available_count = 0;

  memset(weights, 0, sizeof(weights));
  for (size_t i = 0; src[i] != '\0'; i++)
    weights[(unsigned char)src[i]]++;

  for (int j = 0; j < 256; j++) {
    if (weights[j] > 0) {
      chars[available_count] = (unsigned char)j;
      cut_weights[available_count] = weights[j];
      available_count++;
    }
  }

  free_codes();
  if (available_count == 0) {
    *tree = NULL;
    return 0;
  }
  if (build_tree(cut_weights, tree, available_count) != 0)
    return -1;
  for (int i = 1; i <= available_count; i++)
    (*tree)[i].ch = chars[i - 1];

  if (available_count == 1) {
    codes[chars[0]] = dup_str("0");
  } else {
    char path[257];
    path[0] = '\0';
    build_codes(*tree, 2 * available_count - 1, path, 0);
  }
  return available_count;
}

size_t encoded_size(const char *src) {
  huffman_tree tree;
  size_t bits = 0;

  if (prepare_codes(src, &tree) <= 0)
    return 0;
  for (size_t i = 0; src[i] != '\0'; i++)
    bits += strlen(codes[(unsigned char)src[i]]);
  free(tree);
  free_codes();
  return (bits + 7) / 8;
}

int encode(const char *src, char *dst) {
  huffman_tree tree;
  unsigned char acc = 0;
  int nbits = 0, pos = 0;

  if (prepare_codes(src, &tree) <= 0)
    return 0;

  for (size_t i = 0; src[i] != '\0'; i++) {
    const char *code = codes[(unsigned char)src[i]];
    for (const char *p = code; *p != '\0'; p++) {
      acc = (unsigned char)((acc << 1) | (unsigned)(*p - '0'));
      if (++nbits == 8) {
        dst[pos++] = (char)acc;
        acc = 0;
        nbits = 0;
      }
    }
  }
  if (nbits > 0)
    dst[pos++] = (char)(acc << (8 - nbits));

  free(tree);
  free_codes();
  return pos;
}
