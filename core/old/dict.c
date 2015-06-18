#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dict.h"

#define 

// basic implementation of the dictionary object
// although was using a trie, that cannot be optimized until the todo
// list has been completed
// todo:
// support general length branches (array alloc + binary search)
// support generic data type (DONE)
// compress trie (multichar keys)
// remove recursion (iteration)
// check for valid keys

#define CHARSPACE "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_"

void dictnode_insert(dictnode_t *this, char *key, dictobj_t value);
void dictnode_print(dictnode_t *this, char *buf);
int dict_hash(char *key);

dict_t *dict_create(void) {
  return (dict_t *)calloc(1, sizeof(dict_t));
}

// this func is very convoluted (think about what happens when there is no key)
// OLD
void dict_insert(dict_t *this, char *key, void *value) {
  if (!this) {
    return;
  }
  if (!this->root) { // will have to create a root anyway, no matter what
    this->root = (dictnode_t *)calloc(1, sizeof(dictnode_t));
  }
  // recursive algorithm, not so hot for compression
  dictnode_insert(this->root, key, value);
}

void dict_set(dict_t *this, char *key, dictobj_t value) {
  dictnode_t *curr = this->root;
  int i = 0, j = 0;
  if (!this) {
    return;
  }
  for (; i < strlen(key); i++, j++) {
    char c1 = key[i];
    char c2;
    int stopSearching = 0;
    
    // if lies out of bounds, then go to the next one
    while (!curr->key || strlen(curr->key) >= j) {
      int hash;

      // if the child array does not exist, then create it
      if (!curr->child) {
        curr->csize = sizeof(CHARSPACE);
        curr->child = (dictnode_t **)calloc(curr->csize, sizeof(dictnode_t *));
      }
      // if the child does not exist, then create it
      hash = dict_hash(key);
      if (!curr->child[hash]) {
        dictnode_t *next = (dictnode_t *)calloc(1, sizeof(dictnode_t));
        // place the rest of the word as the next key
        char *subkey = &key[j];
        next->key = (char *)malloc((strlen(subkey) + 1) * sizeof(char));
        strcpy(next->key, subkey);
        next->value = value;
        curr->child[hash] = next;
        stopSearching = 1;
        break; // optimized break
      }
      // set the new curr
      curr = curr->child[hash];
      j = 0;
    }
    if (stopSearching) {
      break;
    }

    // find the next instance of difference between the two words
    c2 = curr->key[j];
  }
}

// do later:
  // we have a child array, so we need to search through it
  // in order to get the next child item (binary search)

void dictnode_insert(dictnode_t *this, char *key, void *value) {
  // this is a recursive algorithm
  dictnode_t *child;
  int index;
  if (!this) {
    return;
  }
  // if the key is "empty", then we are done
  if (strlen(key) == 0) {
    this->value = value;
    this->isWord = 1;
    printf("reached end of word\n");
    return;
  }
  // try to look at the current key (single character)
  // and then insert it
  if ((index = dict_hash(key)) == -1) {
    return; // cannot continue (invalid key)
  }
  // create the structure to store the children addresses
  if (!this->child) {
    this->child = (dictnode_t **)calloc(strlen(CHARSPACE), sizeof(dictnode_t *));
    this->csize = strlen(CHARSPACE);
    printf("created arr  [%s].child\n", this->key);
  }
  // insert a node below the parent
  if (!(child = this->child[dict_hash(key)])) {
    child = (dictnode_t *)calloc(1, sizeof(dictnode_t));
    child->parent = this;
    child->key = (char *)malloc(2 * sizeof(char));
    child->key[0] = key[0];
    child->key[1] = '\0';
    this->child[dict_hash(key)] = child;
    printf("created edge [%s]=>[%s]\n", this->key, child->key);
  }
  // recursively traverse
  printf("moving2 edge [%s]=>[%s]\n", this->key, child->key);
  dictnode_insert(child, &key[1], value);
}

int dict_hash(char *key) {
  // for now, use direct mapped hash algorithm
  if (!strchr(CHARSPACE, key[0])) {
    return -1;
  } else {
    int index = (int)((size_t)strchr(CHARSPACE, key[0]) - (size_t)CHARSPACE);
    return index;
  }
}

void dict_print(dict_t *this) {
  char buf[1]; // buffer for 1024 chars, will have to change to dynamic later
  if (!this) {
    return;
  }
  buf[0] = '\0';
  dictnode_print(this->root, buf);
}

void dictnode_print(dictnode_t *this, char *buf) {
  char newbuf[strlen(buf) + 2];
  int i;
  if (!this) {
    return;
  }
  if (this->key) {
    sprintf(newbuf, "%s%s", buf, this->key);
    if (this->isWord) {
      printf("%s\n", newbuf);
    }
  } else {
    strcpy(newbuf, buf);
  }
  for (i = 0; i < this->csize; i++) {
    dictnode_print(this->child[i], newbuf);
  }
}

/////////////////////////////////
//  HASH TABLE IMPLEMENTATION  //
/////////////////////////////////



//////////////////////////////////////
//  DICTIONARY OBJECT INITIALIZERS  //
//////////////////////////////////////

dictobj_t dobjnull(void) {
  dictobj_t obj;
  obj.type = DICT_TYPE_NULL;
  return obj;
}

dictobj_t dobjint(int x) {
  dictobj_t obj;
  obj.asInt = x;
  obj.type = DICT_TYPE_INT;
  return obj;
}

dictobj_t dobjdbl(double dbl) {
  dictobj_t obj;
  obj.asDouble = dbl;
  obj.type = DICT_TYPE_DOUBLE;
  return obj;
}

dictobj_t dobjstr(char *str) {
  dictobj_t obj;
  obj.asString = (char *)malloc((strlen(str) + 1) * sizeof(char));
  strcpy(obj.asString, str);
  obj.type = DICT_TYPE_STRING;
  return obj;
}

dictobj_t dobjbool(int boolean) {
  dictobj_t obj;
  obj.asBool = boolean;
  obj.type = DICT_TYPE_BOOL;
  return obj;
}

dictobj_t dobjarr(dictobj_t *arr, int n) {
  dictobj_t obj;
  obj.asArray = (dictobj_t *)malloc(n * sizeof(dictobj_t));
  memcpy(obj.asArray, arr, n * sizeof(dictobj_t));
  obj.type = DICT_TYPE_ARRAY;
  return obj;
}

dictobj_t dobjdict(dict_t *dict) {
  dictobj_t obj;
  obj.asDict = dict_copy(dict);
  obj.type = DICT_TYPE_DICT;
  return obj;
}
