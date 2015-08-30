#ifndef __TK_MISC_H__
#define __TK_MISC_H__

#include <armadillo>

void draw_rect(arma::mat &I, double v, int x, int y, int width, int height);
void draw_rect(arma::cube &I, const arma::vec &v, int x, int y, int width, int height);
void draw_line(arma::mat &I, double v, int x1, int y1, int x2, int y2);
void draw_line(arma::cube &I, const arma::vec &v, int x1, int y1, int x2, int y2);
void draw_circle(arma::mat &I, double v, int x, int y, double radius);
void draw_circle(arma::cube &I, const arma::vec &v, int x, int y, double radius);

#endif
