/* Includes the installed C ABI header as C99 and calls into the installed shared library. */
#include <stdio.h>
#include <bclibc/ffi/bclibc_ffi.h>

int main(void)
{
    const char *version = BCLIBCFFI_get_version();
    if (!version || !*version) return 1;
    printf("bclibc_ffi %s\n", version);

    /* the buffer and the interpolation of the C ABI, from C99: three points on a line, the middle one asked for */
    BCLIBCFFI_BaseTrajBuf buf = {0, 0, 0};
    BCLIBCFFI_Error err;
    for (int i = 0; i < 3; ++i)
    {
        BCLIBCFFI_BaseTrajData p = {0};
        p.time = i;
        p.px = 10.0 * i;
        p.mach = 2.0;
        if (BCLIBCFFI_base_traj_buf_append(&buf, &p, &err) != BCLIBCFFI_OK) return 2;
    }
    BCLIBCFFI_BaseTrajData out;
    if (BCLIBCFFI_base_traj_interpolate_at(buf.data, buf.count, 1, BCLIBCFFI_INTERP_KEY_TIME, 1.5, &out, &err) != BCLIBCFFI_OK) return 3;
    if (out.time != 1.5 || out.px < 14.9 || out.px > 15.1) return 4;
    if (BCLIBCFFI_base_traj_interpolate_at(buf.data, buf.count, 0, BCLIBCFFI_INTERP_KEY_TIME, 0.5, &out, &err) != BCLIBCFFI_ERR_OUT_OF_BOUNDS) return 5;
    BCLIBCFFI_base_traj_buf_free(&buf);
    return buf.data == 0 ? 0 : 6;
}
