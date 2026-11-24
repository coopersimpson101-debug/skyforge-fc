/*
 * fmt.h - tiny libc-free string formatting/parsing for the CLI.
 */
#ifndef FMT_H
#define FMT_H

#include <stdbool.h>
#include <stdint.h>

int  fmt_strlen(const char *s);
bool fmt_streq(const char *a, const char *b);
/* Append src to dst (capacity cap incl. NUL). Returns new length. */
int  fmt_append(char *dst, int cap, const char *src);
/* Format integer / float into buf (capacity cap). Return chars written. */
int  fmt_int(char *buf, int cap, int32_t v);
int  fmt_uint(char *buf, int cap, uint32_t v);
int  fmt_float(char *buf, int cap, float v, int decimals);
/* Parse decimal float ("-12.5", "3", "+0.25", "1e-3"). */
bool fmt_parse_float(const char *s, float *out);

#endif
