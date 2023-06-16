#ifndef CUMULATIVE_MOVING_AVERAGE_H_
#define CUMULATIVE_MOVING_AVERAGE_H_

#include <stdint.h>

#ifdef __cplusplus
    extern "C" {
#endif

typedef struct
{
    uint32_t n_window;
    float cma_value;
} cumulative_moving_average_t;

// Initializes `cumulative_moving_average_t` structure
void cma_init(cumulative_moving_average_t* h_cma);

// Resets internal average to zero.
void cma_reset(cumulative_moving_average_t* h_cma);

// Calculates CMA (Cumulative Moving Average) with new value `x`.
// CMA window consists of all samples fed after `cma_reset()` call.
// Returns new moving average value after incorporating `x`.
// Formula for calculating CMA is:
//      CMA[n] = (x[n+1] + N * CMA[n-1])/(N + 1)
// Where:
//      CMA[n] - Cumulative Moving Average after incorporating sample x[n]
//      CMA[n-1] - CMA value before incorporating x[n]
//      x[n] - New sample
//      N - Size of CMA window (before incorporating x[n])
//
//  Note:
//      CMA window size (N) will never overflow, instead it
//      will be clamped to maximal possible number that can
//      be represented by the underlying type (`uint32_t` currently)
float cma_feed_sample(cumulative_moving_average_t* h_cma, float x);

// Returns current CMA value
float cma_get_value(cumulative_moving_average_t* h_cma);

#ifdef __cplusplus
    }
#endif

#endif