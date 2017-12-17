/*
  compile with:

  gcc -O3 -ffast-math -msse2 -msse3 -lm main2.c && ./a.out
*/

#include <stdio.h>
#include <stdlib.h>
#include <xmmintrin.h>  // For SSE compiler intrinsics
#include <immintrin.h>  // for AVX compiler intrinsics
#include <math.h>       // For sqrt in CPU-only version
#include <time.h>

int main(int argc, char* argv[]) {
  printf("Starting calculation...\n");

  const int length = 64000;

  /* We will be calculating Y = Sin(x) / x, for x = 1->64000 */

  // If you do not properly align your data for SSE instructions, you may take a huge performance hit.
  float* pResult = (float*) aligned_alloc(16, length * sizeof(float));  // align to 16-byte for SSE
  __m128 x;
  __m128 xDelta = _mm_set1_ps(4.0f);    // Set the xDelta to (4,4,4,4)
  __m128* pResultSSE = (__m128*) pResult;

  const int SSELength = length / 4;

  time_t start = clock();
  for(int stress = 0; stress < 100000; stress++) {

    x = _mm_set_ps(4.0f, 3.0f, 2.0f, 1.0f); // Set the initial values of x to (4,3,2,1)
    for(int i = 0; i < SSELength; i++) {
      __m128 xSqrt = _mm_sqrt_ps(x);
      // Note! Division is slow. It's actually faster to take the reciprocal of a number and multiply
      // Also note that Division is more accurate than taking the reciprocal and multiplying

#define USE_DIVISION_METHOD
#ifdef USE_FAST_METHOD
      __m128 xRecip = _mm_rcp_ps(x);
      pResultSSE[i] = _mm_mul_ps(xRecip, xSqrt);
#endif //USE_FAST_METHOD
#ifdef USE_DIVISION_METHOD
      pResultSSE[i] = _mm_div_ps(xSqrt, x);
#endif  // USE_DIVISION_METHOD

      // NOTE! Sometimes, the order in which things are done in SSE may seem reversed.
      // When the command above executes, the four floating elements are actually flipped around
      // We have already compensated for that flipping by setting the initial x vector to (4,3,2,1) instead of (1,2,3,4)

      x = _mm_add_ps(x, xDelta);  // Advance x to the next set of numbers
    }

  }
  time_t end = clock();
  double elapsed_time = ((double)end - (double)start) / CLOCKS_PER_SEC;
  printf("elapsed time using SSE: %f\n", elapsed_time);

  /* verify output of program */
  for(int i = 0; i < 20; i++) {
    printf("Result[%d] = %f\n", i, pResult[i]);
  }


  start = clock();
  for(int stress = 0; stress < 100000; stress++) {
    float xFloat = 1.0f;
    for(int i = 0 ; i < length; i++) {
      pResult[i] = sqrt(xFloat) / xFloat;
      xFloat += 1.0f;
    }
  }
  end = clock();
  elapsed_time = ((double)end - (double)start) / CLOCKS_PER_SEC;
  printf("elapsed time without SSE: %f\n", elapsed_time);

  /* verify output of program */
  for(int i = 0; i < 20; i++) {
    printf("Result[%d] = %f\n", i, pResult[i]);
  }

  // Results for my system
  // 1,65 seconds for SSE with reciprocal/multiplication method
  // 2.30 seconds for SSE with division method
  // 48.89 seconds for CPU

  return 0;
}
