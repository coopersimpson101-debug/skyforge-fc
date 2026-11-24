/*
 * libc_min.c - the four memory routines a freestanding C compiler may emit
 * calls to (struct copies, zero-initialisation). No other libc is linked.
 * The byte pointers are volatile so the compiler cannot turn these loops
 * back into calls to themselves.
 */
#include <stddef.h>
#include <stdint.h>

__attribute__((used)) void *memcpy(void *dst, const void *src, size_t n)
{
    volatile uint8_t *d = (volatile uint8_t *)dst;
    const volatile uint8_t *s = (const volatile uint8_t *)src;
    while (n--) {
        *d++ = *s++;
    }
    return dst;
}

__attribute__((used)) void *memmove(void *dst, const void *src, size_t n)
{
    volatile uint8_t *d = (volatile uint8_t *)dst;
    const volatile uint8_t *s = (const volatile uint8_t *)src;
    if (d < s) {
        while (n--) {
            *d++ = *s++;
        }
    } else {
        d += n;
        s += n;
        while (n--) {
            *--d = *--s;
        }
    }
    return dst;
}

__attribute__((used)) void *memset(void *dst, int c, size_t n)
{
    volatile uint8_t *d = (volatile uint8_t *)dst;
    while (n--) {
        *d++ = (uint8_t)c;
    }
    return dst;
}

__attribute__((used)) int memcmp(const void *a, const void *b, size_t n)
{
    const uint8_t *x = (const uint8_t *)a, *y = (const uint8_t *)b;
    for (size_t i = 0; i < n; i++) {
        if (x[i] != y[i]) {
            return x[i] < y[i] ? -1 : 1;
        }
    }
    return 0;
}

/* ARM run-time ABI aliases (clang emits these for struct copies/zeroing;
 * note the __aeabi_memset argument order: dest, n, c). */
__attribute__((used)) void __aeabi_memcpy(void *d, const void *s, size_t n) { memcpy(d, s, n); }
__attribute__((used)) void __aeabi_memcpy4(void *d, const void *s, size_t n) { memcpy(d, s, n); }
__attribute__((used)) void __aeabi_memcpy8(void *d, const void *s, size_t n) { memcpy(d, s, n); }
__attribute__((used)) void __aeabi_memmove(void *d, const void *s, size_t n) { memmove(d, s, n); }
__attribute__((used)) void __aeabi_memmove4(void *d, const void *s, size_t n) { memmove(d, s, n); }
__attribute__((used)) void __aeabi_memset(void *d, size_t n, int c) { memset(d, c, n); }
__attribute__((used)) void __aeabi_memset4(void *d, size_t n, int c) { memset(d, c, n); }
__attribute__((used)) void __aeabi_memclr(void *d, size_t n) { memset(d, 0, n); }
__attribute__((used)) void __aeabi_memclr4(void *d, size_t n) { memset(d, 0, n); }
__attribute__((used)) void __aeabi_memclr8(void *d, size_t n) { memset(d, 0, n); }
