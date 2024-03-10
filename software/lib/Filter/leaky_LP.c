// leaky_LP.c
#include "leaky_LP.h"

// Initialize the leaky integrator filter
void leaky_init(leaky_lp *filter, float alpha)
{
  filter->alpha = alpha;
  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    filter->prev[i] = 0.0f;
  }
}

// Update the leaky integrator filter with a new input
void leaky_update(leaky_lp *filter, float input[VECTOR_SIZE], float output[VECTOR_SIZE])
{
  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    output[i] = filter->alpha * input[i] + (1 - filter->alpha) * filter->prev[i];
    filter->prev[i] = output[i];
  }
  
}