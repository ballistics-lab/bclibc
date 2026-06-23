/* Math shims for natmod embedded libm (fdlibm / musl libm_dbl).
 *
 * sincosf / sincos: GCC -O2 merges adjacent sin()+cos() into a single
 * sincos() call (GNU extension absent from embedded libm). -O0 prevents
 * the compiler from applying the same optimisation to these wrappers,
 * which would cause infinite recursion.
 *
 * tanf / tan: not provided by fdlibm or libm_dbl; implemented as sin/cos.
 * -O0 also prevents GCC from folding sin(x)/cos(x) back into tan(x).
 */
#pragma GCC optimize("O0")
#include <math.h>

#ifdef TINY_BCLIBC_USE_FLOAT
void  sincosf(float x, float *s, float *c) { *s = sinf(x); *c = cosf(x); }
float tanf(float x) { return sinf(x) / cosf(x); }
#else
void   sincos(double x, double *s, double *c) { *s = sin(x); *c = cos(x); }
double tan(double x) { return sin(x) / cos(x); }
#endif
