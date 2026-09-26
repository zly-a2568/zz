#include "huffman.h"
#include <stdint.h>

char *codes[256];

typedef struct {
  int child[2];
  int symbol;
} decode_node;

static void put_u16(unsigned char *p, size_t v) {
  p[0] = (unsigned char)(v & 0xff);
  p[1] = (unsigned char)((v >> 8) & 0xff);
}

static void put_u64(unsigned char *p, size_t v) {
  for (int i = 0; i < 8; i++)
    p[i] = (unsigned char)((v >> (8 * i)) & 0xff);
}

static size_t get_u64(const unsigned char *p) {
  size_t v = 0;
  for (int i = 0; i < 8; i++)
    v |= (size_t)p[i] << (8 * i);
  return v;
}

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

static void collect_lengths(huffman_tree tree, int idx, unsigned char *lengths,
                            int depth) {
  huffman_node *node = tree + idx;
  if (node->left == 0 && node->right == 0) {
    lengths[node->ch] = (unsigned char)depth;
    return;
  }
  collect_lengths(tree, node->left, lengths, depth + 1);
  collect_lengths(tree, node->right, lengths, depth + 1);
}

static int canonical_codes(const unsigned char *lengths) {
  int keys[256], n = 0, cur_len = 0;
  unsigned char bits[256];
  char raw[257];

  free_codes();
  for (int i = 0; i < 256; i++)
    if (lengths[i] > 0)
      keys[n++] = lengths[i] * 256 + i;
  if (n == 0)
    return 0;
  quick_sort(keys, 0, n - 1);
  memset(bits, 0, sizeof(bits));

  for (int k = 0; k < n; k++) {
    int sym = keys[k] % 256;
    int len = keys[k] / 256;
    if (len < cur_len)
      return -1;
    int shift = len - cur_len;
    if (k > 0) {
      int b = 0;
      while (b < cur_len && bits[b] == 1)
        bits[b++] = 0;
      if (b == cur_len)
        return -1;
      bits[b] = 1;
    }
    for (int b = cur_len - 1; b >= 0; b--)
      bits[b + shift] = bits[b];
    for (int b = 0; b < shift; b++)
      bits[b] = 0;
    cur_len = len;
    for (int b = 0; b < len; b++)
      raw[b] = bits[len - 1 - b] ? '1' : '0';
    raw[len] = '\0';
    codes[sym] = dup_str(raw);
  }
  for (int b = 0; b < cur_len; b++)
    if (n > 1 && bits[b] == 0)
      return -1;
  return n;
}

static int prepare_codes(const unsigned char *src, size_t len,
                         huffman_tree *tree) {
  int weights[256];
  int cut_weights[256];
  unsigned char chars[256], lengths[256];
  int available_count = 0;

  memset(weights, 0, sizeof(weights));
  for (size_t i = 0; i < len; i++)
    weights[src[i]]++;

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

  memset(lengths, 0, sizeof(lengths));
  if (available_count == 1) {
    lengths[chars[0]] = 1;
  } else {
    collect_lengths(*tree, 2 * available_count - 1, lengths, 0);
  }
  if (canonical_codes(lengths) < 0)
    return -1;
  return available_count;
}

size_t encoded_size(const unsigned char *src, size_t len) {
  huffman_tree tree;
  size_t bits = 0;
  int n = prepare_codes(src, len, &tree);

  if (n <= 0) {
    free(tree);
    free_codes();
    return n == 0 ? HUFF_HEADER_FIXED : 0;
  }
  for (size_t i = 0; i < len; i++)
    bits += strlen(codes[src[i]]);
  free(tree);
  free_codes();
  return HUFF_HEADER_FIXED + 2 * (size_t)n + (bits + 7) / 8;
}

int encode(const unsigned char *src, size_t len, unsigned char *dst,
           size_t dst_cap) {
  huffman_tree tree;
  unsigned char acc = 0;
  size_t need, pos = 0;
  int nbits = 0, n = prepare_codes(src, len, &tree);

  if (n < 0)
    return -1;
  need = HUFF_HEADER_FIXED + 2 * (size_t)n;
  if (n > 0) {
    size_t bits = 0;
    for (size_t i = 0; i < len; i++)
      bits += strlen(codes[src[i]]);
    need += (bits + 7) / 8;
  }
  if (dst == NULL || dst_cap < need) {
    free(tree);
    free_codes();
    return -1;
  }

  dst[pos++] = HUFF_MAGIC;
  put_u16(dst + pos, (size_t)n);
  pos += 2;
  put_u64(dst + pos, len);
  pos += 8;
  for (int s = 0; s < 256; s++) {
    if (codes[s] == NULL)
      continue;
    dst[pos++] = (unsigned char)s;
    dst[pos++] = (unsigned char)strlen(codes[s]);
  }

  for (size_t i = 0; i < len; i++) {
    const char *code = codes[src[i]];
    for (const char *p = code; *p != '\0'; p++) {
      acc = (unsigned char)((acc << 1) | (unsigned)(*p - '0'));
      if (++nbits == 8) {
        dst[pos++] = acc;
        acc = 0;
        nbits = 0;
      }
    }
  }
  if (nbits > 0)
    dst[pos++] = (unsigned char)(acc << (8 - nbits));

  free(tree);
  free_codes();
  return (int)pos;
}

size_t decoded_size(const unsigned char *src, size_t src_len) {
  if (src == NULL || src_len < HUFF_HEADER_FIXED || src[0] != HUFF_MAGIC)
    return 0;
  return get_u64(src + 3);
}

size_t decode(const unsigned char *src, size_t src_len, unsigned char *dst,
              size_t dst_cap) {
  unsigned char lengths[256];
  size_t cap = 1, used = 1, out = 0, need = 0;
  decode_node *nodes;
  int cur = 0, nbits = 0, n;

  if (src == NULL || src_len < HUFF_HEADER_FIXED || src[0] != HUFF_MAGIC)
    return (size_t)-1;
  n = (int)src[1] | ((int)src[2] << 8);
  if (src_len < HUFF_HEADER_FIXED + 2 * (size_t)n)
    return (size_t)-1;
  need = decoded_size(src, src_len);
  if (dst == NULL || need > dst_cap)
    return (size_t)-1;

  memset(lengths, 0, sizeof(lengths));
  for (int i = 0; i < n; i++) {
    int sym = src[HUFF_HEADER_FIXED + 2 * i];
    int len = src[HUFF_HEADER_FIXED + 2 * i + 1];
    if (len < 1 || lengths[sym] != 0)
      return (size_t)-1;
    lengths[sym] = (unsigned char)len;
    cap += (size_t)len;
  }
  if (canonical_codes(lengths) < 0)
    return (size_t)-1;

  nodes = (decode_node *)malloc(cap * sizeof(decode_node));
  if (nodes == NULL) {
    free_codes();
    return (size_t)-1;
  }
  nodes[0].child[0] = nodes[0].child[1] = -1;
  nodes[0].symbol = -1;
  for (int s = 0; s < 256; s++) {
    if (codes[s] == NULL)
      continue;
    cur = 0;
    for (const char *p = codes[s]; *p != '\0'; p++) {
      int bit = *p - '0';
      if (nodes[cur].child[bit] < 0) {
        int next = (int)used++;
        nodes[cur].child[bit] = next;
        nodes[next].child[0] = nodes[next].child[1] = -1;
        nodes[next].symbol = -1;
      }
      cur = nodes[cur].child[bit];
    }
    nodes[cur].symbol = s;
  }
  cur = 0;
  free_codes();

  size_t pos = HUFF_HEADER_FIXED + 2 * (size_t)n;
  while (out < need) {
    if (pos >= src_len)
      break;
    int bit = (src[pos] >> (7 - nbits)) & 1;
    if (++nbits == 8) {
      nbits = 0;
      pos++;
    }
    int next = nodes[cur].child[bit];
    if (next < 0)
      break;
    cur = next;
    if (nodes[cur].symbol >= 0) {
      dst[out++] = (unsigned char)nodes[cur].symbol;
      cur = 0;
    }
  }
  free(nodes);
  if (out != need)
    return (size_t)-1;
  return out;
}
