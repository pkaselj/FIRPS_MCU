#include "cma.h"

#ifndef NULL
#define NULL (void*)0x00
#endif

void cma_init(cumulative_moving_average_t* h_cma)
{
    if (h_cma == NULL)
    {
        return;
    }

    h_cma->cma_value = 0;
    h_cma->n_window = 0;
}

void cma_reset(cumulative_moving_average_t* h_cma)
{
    if (h_cma == NULL)
    {
        return;
    }

    h_cma->cma_value = 0;
    h_cma->n_window = 0;
}

float cma_feed_sample(cumulative_moving_average_t* h_cma, float x)
{
    if (h_cma == NULL)
    {
        return 0;
    }

    const uint32_t n_plus_1 = (h_cma->n_window + 1 == 0) ? h_cma->n_window : (h_cma->n_window + 1);
    const uint32_t n = h_cma->n_window;

    h_cma->cma_value = (x + n * h_cma->cma_value) / n_plus_1;
    h_cma->n_window = n_plus_1;

    return h_cma->cma_value;
}

float cma_get_value(cumulative_moving_average_t* h_cma)
{
    if (h_cma == NULL)
    {
        return 0;
    }

    return h_cma->cma_value;
}