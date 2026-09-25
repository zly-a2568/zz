#include "huffman.h"

int main() {
  const char *test_char = "hello,world";
  char dst_char[1024];
  memset(dst_char, 0, sizeof(dst_char));
  encode(test_char, dst_char);
  printf("%s", dst_char);
  return 0;
}
