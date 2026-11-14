#include <stdlib.h>
#include <string.h>

typedef struct list_node {
  struct list_node* prev;
  struct list_node* next;
  void* data;
} list_node;

typedef struct {
  list_node* head;
  list_node* tail;
  size_t data_size;
} list;

void list_init(list* l, size_t data_size) {
  l->head = NULL;
  l->tail = NULL;
  l->data_size = data_size;
}

void list_add_head(list* l, void* data) {
  list_node* new_head = (list_node*)malloc(sizeof(list_node) + l->data_size);
  if (!new_head) return; 

  new_head->data = new_head + 1; 
  
  memcpy(new_head->data, data, l->data_size);

  new_head->prev = NULL;
  new_head->next = l->head;

  if (l->head) {
    l->head->prev = new_head;
  } else {
    l->tail = new_head;
  }
  
  l->head = new_head;
}

void list_add_tail(list* l, void* data) {
  list_node* new_tail = (list_node*)malloc(sizeof(list_node) + l->data_size);
  if (!new_tail) return; 

  new_tail->data = new_tail + 1;

  memcpy(new_tail->data, data, l->data_size);

  new_tail->next = NULL;
  new_tail->prev = l->tail;

  if (l->tail) {
    l->tail->next = new_tail;
  } else {
    l->head = new_tail;
  }

  l->tail = new_tail;
}
