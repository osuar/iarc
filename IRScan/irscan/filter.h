#ifndef FILTER_H 
#define FILTER_H 

#include "struct.h"
#include "all.h"
void sweep(int *stuff);
int set(int raw);
void derivitive(int* dummy, int* deriv);
void average(coord* data, coord* dummy, int c);
void smooth(int* dummy, int * deriv);


#endif
