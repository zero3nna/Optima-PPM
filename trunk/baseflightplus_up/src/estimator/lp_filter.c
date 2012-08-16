#include "board.h"
#include "estimator/lp_filter.h"

///////////////////////////////////////////////////////////////////////////////
//  4th Order Filter
///////////////////////////////////////////////////////////////////////////////

float fourthOrderFilter(float input, fourthOrderData_t *filterData, float *A, float *B)
{
    float output;

    output = B[0] * input + B[1] * filterData->input[1-1] + B[2] * filterData->input[2-1]
                    + B[3] * filterData->input[3-1] + B[4] * filterData->input[4-1] - A[1-1] * filterData->output[1-1]
                    - A[2-1] * filterData->output[2-1] - A[3-1] * filterData->output[3-1] - A[4-1] * filterData->output[4-1];

    filterData->input[4-1] = filterData->input[3-1];
    filterData->input[3-1] = filterData->input[2-1];
    filterData->input[2-1] = filterData->input[1-1];
    filterData->input[1-1] = input;

    filterData->output[4-1] = filterData->output[3-1];
    filterData->output[3-1] = filterData->output[2-1];
    filterData->output[2-1] = filterData->output[1-1];
    filterData->output[1-1] = output;

    return output;
}

///////////////////////////////////////////////////////////////////////////////
