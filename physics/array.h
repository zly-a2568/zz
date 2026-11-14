#ifndef ARRAY_H
#define ARRAY_H
#include <stdlib.h>

typedef struct{
  void* data;
  size_t size;
  size_t capacity;
  size_t elem_size;
} array;


void array_init(size_t elem_size,array* a);
int array_push(const void* elem,array* a);
void* array_get(size_t pos,array* a);
int array_set(size_t pos,const void* elem,array* a);
int array_pop(array* a);
int array_remove(size_t pos,array* a);
void array_free(array* a);
int array_clear(array*a);

#endif //ARRAY_H