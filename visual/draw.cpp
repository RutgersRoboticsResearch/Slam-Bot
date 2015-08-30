#include "assert.h"
#include "misc.h"
#include <cstdio>

#define MIN(a, b) (((a)<(b))?(a):(b))
#define MAX(a, b) (((a)>(b))?(a):(b))
#define LIMIT(x, a, b) (((x)<(a))?(a):(((b)<(x))?(b):(x)))
#define WITHIN(x, a, b) (((x)>=(a))&&((x)<=(b)))
#define SWAP(a, b) {(a)^=(b);(b)^=(a);(a)^=(b);}

using namespace arma;

void draw_rect(mat &I, double v, int x, int y, int width, int height) {
  assert(width >= 0 && height >= 0);
  int y1 = y;
  int y2 = y+height;
  if (y1 > y2) {
    SWAP(y1, y2);
  }
  int i1 = LIMIT(y1, 0, (int)I.n_rows-1);
  int i2 = LIMIT(y2, 0, (int)I.n_rows-1);
  int x1 = x;
  int x2 = x+width;
  if (x1 > x2) {
    SWAP(x1, x2);
  }
  int j1 = LIMIT(x1, 0, (int)I.n_cols-1);
  int j2 = LIMIT(x2, 0, (int)I.n_cols-1);
  if (WITHIN(x1, 0, (int)I.n_cols-1)) {
    for (int i = i1; i <= i2; i++) {
      I(i, x1) = v;
    }
  }
  if (WITHIN(x2, 0, (int)I.n_cols-1)) {
    for (int i = i1; i <= i2; i++) {
      I(i, x2) = v;
    }
  }
  if (WITHIN(y1, 0, (int)I.n_rows-1)) {
    for (int j = j1; j <= j2; j++) {
      I(y1, j) = v;
    }
  }
  if (WITHIN(y2, 0, (int)I.n_rows-1)) {
    for (int j = j1; j <= j2; j++) {
      I(y2, j) = v;
    }
  }
}

void draw_rect(cube &I, const vec &v, int x, int y, int width, int height) {
  for (uword k = 0; k < v.n_elem; k++) {
    draw_rect(I.slice(k), v(k), x, y, width, height);
  }
}

void draw_line(mat &I, double v, int x1, int y1, int x2, int y2) {
  if (x2 == x1) { // special case
    if (x1 < 0 || x2 >= (int)I.n_cols) {
      return; // dont do anything
    }
    y1 = LIMIT(y1, 0, (int)I.n_rows-1);
    y2 = LIMIT(y2, 0, (int)I.n_rows-1);
    int dy = (y2-y1>=0) ? 1 : -1;
    for (int i = y1; i != y2; i+=dy) {
      I(i, x1) = v;
    }
    return;
  }
  double slope = ((double)y2-(double)y1)/((double)x2-(double)x1);
  x1 = LIMIT(x1, 0, (int)I.n_cols-1);
  x2 = LIMIT(x2, 0, (int)I.n_cols-1);
  int dx = x2-x1 >= 0 ? 1 : -1;
  for (int j = x1; j != x2; j+=dx) {
    int i = (int)round((double)(j-x1)*slope)+y1;
    if (WITHIN(i, 0, (int)I.n_rows-1)) {
      I(i, j) = v;
    }
  }
}

void draw_line(cube &I, const vec &v, int x1, int y1, int x2, int y2) {
  for (uword k = 0; k < v.n_elem; k++) {
    draw_line(I.slice(k), v(k), x1, y1, x2, y2);
  }
}

// breaks at limiting cases
void draw_circle(mat &I, double v, int x, int y, double radius) {
  int x1 = MAX(0, (int)round((double)x-radius));
  int x2 = MIN((int)round((double)x+radius), (int)I.n_cols-1);
  double r_2 = radius * radius;
  for (int j = x1; j <= x2; j++) {
    double _x = (double)j - (double)x;
    double dy = sqrt(r_2 - (_x * _x));
    int i = (int)round((double)y - dy);
    if (i >= 0) {
      I(i, j) = v;
    }
    i = (int)round((double)y + dy);
    if (i <= (int)I.n_rows-1) {
      I(i, j) = v;
    }
  }
}

void draw_circle(cube &I, const vec &v, int x, int y, double radius) {
  for (uword k = 0; k < v.n_elem; k++) {
    draw_circle(I.slice(k), v(k), x, y, radius);
  }
}
