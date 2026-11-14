#include "../array.h"
#include <stdio.h>

void print_array(array* a){
    for(int j=0;j<a->size;j++){
        printf("%d",*((int*)array_get(j,a)));
    }
    printf("\n");
}

int main(){
    array a;
    array_init(sizeof(int),&a);
    for(int i=0;i<5;i++){
        array_push(&i,&a);
    }
    print_array(&a);
    array_pop(&a);
    print_array(&a);
    array_remove(2,&a);
    print_array(&a);
    printf("%d\n",*((int*)array_get(0,&a)));
    printf("\n");
    array_free(&a);
    return 0;
}