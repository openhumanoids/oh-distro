#include "cpuid.h"

#define CPUID(func,ax,bx,cx,dx)\
    __asm__ __volatile__ ( \
            "xchgl %%ebx, %1    \n\t" \
            "cpuid              \n\t" \
            "xchgl %%ebx, %1    \n\t" \
            : "=a" (ax), "=r" (bx), "=c" (cx), "=d" (dx) \
            : "a" (func) \
            : "cc")

void
cpuid_detect (int * sse2, int * sse3)
{
    int a, b, c, d;
    CPUID (1, a, b, c, d);

    if (sse2)
        *sse2 = (d >> 26) & 1;
    if (sse3)
        *sse3 = c & 1;
}
