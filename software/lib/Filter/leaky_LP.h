// leaky_LP.h
// simple LF filter

#ifndef LEAKY_LP_H
#define LEAKY_LP_H

#define VECTOR_SIZE 3 // Size of the vectors for angular velocity and acceleration (xyz)

typedef struct
{
  float alpha;      // The leaky coefficient [0,1] "how much of new data we are using?"
  int size;
  float prev[VECTOR_SIZE];
} leaky_lp;


void leaky_init(leaky_lp *filter, float alpha);
void leaky_update(leaky_lp *filter, float input[VECTOR_SIZE], float output[VECTOR_SIZE]);

#endif // LEAKY_LP_H