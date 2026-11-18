/* Includes the installed header. At -O0 its inline code is emitted and calls libm, so this also checks that the
   package hands libm to the consumer. */
#include <stdio.h>
#include <tiny_bclibc.h>

int main(void)
{
    printf("tiny_bclibc ok, sizeof(real_t)=%zu\n", sizeof(real_t));
    return 0;
}
