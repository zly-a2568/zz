#include "array.h"
#include <stdlib.h>
#include <string.h>

void array_init(size_t elem_size,array* a){
  a->size=0;
  a->capacity=0;
  a->elem_size=elem_size;
  a->data=NULL;
}

int array_push(const void* elem,array* a){
  if (a->size==a->capacity){
    size_t new_capacity=a->size==0?4:a->capacity*2;
    void *tmp_data = realloc(a->data, new_capacity*a->elem_size);
    if (!tmp_data)return -1;
    a->data=tmp_data;
    a->capacity=new_capacity;
  }
  memcpy(a->data+a->size*a->elem_size,elem,a->elem_size);
  a->size++;
  return 0;
}

void* array_get(size_t pos,array* a){
  if(pos>a->size-1)return NULL;
  return a->data+pos*a->elem_size;
}

int array_set(size_t pos,const void* elem,array* a){
  if(pos>a->size-1)return -1;
  memcpy(a->data+pos*a->elem_size,elem,a->elem_size);
  return 0;
}

int array_pop(array* a){
  if(a->size==0)return -1;
  a->size--;
  return 0;
}

int array_remove(size_t pos, array* a) {
    if(a->size <= 0) return 0;
    if(pos > a->size - 1) return -1;
    
    size_t bytes_to_move = (a->size - pos - 1) * a->elem_size;
    
    if (bytes_to_move > 0) {
        memmove(a->data + pos * a->elem_size, 
                a->data + (pos + 1) * a->elem_size, 
                bytes_to_move);
    }
    
    a->size--;
    return 0;
}

int array_clear(array*a){
  if(a->data&&a->size) memset(a->data,0,a->size*a->elem_size);
  a->size=0;
  return 0;
}

size_t array_length(array* a){
  return a->size;
}

void array_free(array* a){
  free(a->data);
  a->size=0;
  a->capacity=0;
  a->elem_size=0;
  a->data=NULL;
}

