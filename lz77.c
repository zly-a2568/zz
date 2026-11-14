#include <string.h>
#include <stdio.h>


void find_substring(const char* src,int left,int upcoming_start,int* offset,int* length,int* next){
  int i=left;
  int len=0;
  while(i<upcoming_start){
    if(src[i]==src[upcoming_start+i]){
      len++;
      i++;
    }
    else{
      if(len<2){
        *offset=0;
        *length=0;
        *next=i+1;
      }
      else{
        *length=len;
        *offset=i-left;
        *next=i+1;
      }
    }
  }
}

void lz77(const char* src,char* dst){
  
}
