/*
 * tinytest.h - minimal single-header unit test helper.
 */
#ifndef TINYTEST_H
#define TINYTEST_H

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int tt_failures = 0;
static int tt_checks = 0;
static int tt_tests = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        tt_checks++;                                                         \
        if (!(cond)) {                                                       \
            tt_failures++;                                                   \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                \
    do {                                                                     \
        double tt_a = (double)(a), tt_b = (double)(b);                       \
        tt_checks++;                                                         \
        if (!(fabs(tt_a - tt_b) <= (double)(tol))) {                         \
            tt_failures++;                                                   \
            printf("  FAIL %s:%d: %s = %.6g, expected %.6g (tol %.3g)\n",    \
                   __FILE__, __LINE__, #a, tt_a, tt_b, (double)(tol));       \
        }                                                                    \
    } while (0)

#define CHECK_EQ_INT(a, b)                                                   \
    do {                                                                     \
        long tt_a = (long)(a), tt_b = (long)(b);                             \
        tt_checks++;                                                         \
        if (tt_a != tt_b) {                                                  \
            tt_failures++;                                                   \
            printf("  FAIL %s:%d: %s = %ld (0x%lx), expected %ld (0x%lx)\n", \
                   __FILE__, __LINE__, #a, tt_a, tt_a, tt_b, tt_b);          \
        }                                                                    \
    } while (0)

#define RUN(fn)                                                              \
    do {                                                                     \
        int tt_before = tt_failures;                                         \
        tt_tests++;                                                          \
        fn();                                                                \
        printf("[%s] %s\n", tt_failures == tt_before ? " ok " : "FAIL", #fn);\
    } while (0)

#define TT_SUMMARY()                                                         \
    (printf("%d tests, %d checks, %d failures\n", tt_tests, tt_checks,       \
            tt_failures),                                                    \
     tt_failures ? 1 : 0)

#endif
