/* Includes the installed C ABI header as C99 and calls into the installed shared library. */
#include <stdio.h>
#include <bclibc/ffi/bclibc_ffi.h>

int main(void)
{
    const char *version = BCLIBCFFI_get_version();
    if (!version || !*version) return 1;
    printf("bclibc_ffi %s\n", version);
    return 0;
}
