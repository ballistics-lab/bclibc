/* tiny_bclibc_impl.c — compile this one file to build a shared/static library.
 * All symbols are emitted here; consumers use TINY_BCLIBC_USE_SHARED + link against
 * the resulting library.
 *
 * Build:
 *   gcc -O2 -shared -fPIC -DTINY_BCLIBC_BUILD_SHARED -Iinclude \
 *       src/tiny_bclibc_impl.c -o libtiny_bclibc.so
 */
#ifndef TINY_BCLIBC_BUILD_SHARED
#  define TINY_BCLIBC_BUILD_SHARED
#endif
#include "tiny_bclibc.h"
