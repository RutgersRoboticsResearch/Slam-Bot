#include "assert.h"
#include "misc.h"

#define MIN(a, b) (((a)<(b))?(a):(b))
#define MAX(a, b) (((a)>(b))?(a):(b))

void draw_rect(const mat &I, double v, uword x, uword y, uword width, uword height) {
  assert(width > 0 && height > 0);
  for (uword i = MAX(0, y); i < MIN(y + height, I.n_rows); i++) {
    I(i, x) = v;
    I(i, x+width-1) = v;
  }
  for (uword j = MAX(0, x); j < MIN(x + width, I.n_cols); j++) {
    I(y, j) = v;
    I(y, j+height-1) = v;
  }
}

void draw_rect(const cube &I, const vec &v, uword x, uword y, uword width, uword height) {
  for (uword i = MAX(0, y); i < MIN(y + height, I.n_rows); i++) {
    for (uword k = 0; k < v.n_elem; k++) {
      I(i, x, k) = v(k);
      I(i, x+width-1, k) = v(k);
    }
  }
  for (uword j = MAX(0, x); j < MIN(x + width, I.n_cols); j++) {
    for (uword k = 0; k < v.n_elem; k++) {
      I(y, j, k) = v(k);
      I(y, j+height-1, k) = v(k);
    }
  }
}
