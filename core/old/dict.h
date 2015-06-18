#ifndef __SB_DICT_H__
#define __SB_DICT_H__

#define DICT_TYPE_NULL    0
#define DICT_TYPE_INT     1
#define DICT_TYPE_DOUBLE  2
#define DICT_TYPE_STRING  3
#define DICT_TYPE_BOOL    4
#define DICT_TYPE_ARRAY   5
#define DICT_TYPE_DICT    6

typedef struct dictobj  dictobj_t;
typedef struct dictnode dictnode_t;
typedef struct dict     dict_t;

struct dictobj {
  union {
    int asInt;
    double asDouble;
    char *asString;
    int asBoolean;
    struct {
      dictobj_t *asArray;
      int arrayLength;
    };
    dict_t *asDict;
  };
  unsigned char type;
};

struct dictnode {
  char *key;
  dictobj_t value;
  dictnode_t *parent;
  dictnode_t **child;
//  int nchild;
  int csize;
  int isWord;
};

// start with the most basic implementation of the trie
struct dict {
  dictnode_t *root;
};

dict_t *dict_create(void);
void dict_set(dict_t *dict, char *key, dictobj_t value);
dictobj_t dict_get(dict_t *dict, char *key);
void dict_remove(dict_t *dict, char *key);
void dict_print(dict_t *dict);

// remember, these copy everything!
dictobj_t dobjnull(void);
dictobj_t dobjint(int x);
dictobj_t dobjdbl(double f);
dictobj_t dobjstr(char *str);
dictobj_t dobjbool(int b);
dictobj_t dobjarr(dictobj_t *arr, int n);
dictobj_t dobjdict(dict_t *dict);

#endif
