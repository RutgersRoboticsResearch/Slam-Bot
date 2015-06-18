#include "dict.h"

int main() {
  dict_t *dict;
  dict = dict_create();
  dict_insert(dict, "cat", (void *)1);
  dict_insert(dict, "car", (void *)1);
  dict_insert(dict, "hello", (void *)1);
  dict_print(dict);
  return 0;
}
