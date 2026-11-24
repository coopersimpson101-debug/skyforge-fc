#include "fmt.h"

int fmt_strlen(const char *s)
{
    int n = 0;
    while (s[n]) {
        n++;
    }
    return n;
}

bool fmt_streq(const char *a, const char *b)
{
    while (*a && *a == *b) {
        a++;
        b++;
    }
    return *a == *b;
}

int fmt_append(char *dst, int cap, const char *src)
{
    int n = fmt_strlen(dst);
    while (*src && n < cap - 1) {
        dst[n++] = *src++;
    }
    dst[n] = 0;
    return n;
}

int fmt_uint(char *buf, int cap, uint32_t v)
{
    char tmp[11];
    int n = 0;
    do {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    } while (v && n < 10);
    int w = 0;
    while (n > 0 && w < cap - 1) {
        buf[w++] = tmp[--n];
    }
    buf[w] = 0;
    return w;
}

int fmt_int(char *buf, int cap, int32_t v)
{
    if (v < 0) {
        if (cap < 3) {
            buf[0] = 0;
            return 0;
        }
        buf[0] = '-';
        return 1 + fmt_uint(buf + 1, cap - 1, (uint32_t)(-(int64_t)v));
    }
    return fmt_uint(buf, cap, (uint32_t)v);
}

int fmt_float(char *buf, int cap, float v, int decimals)
{
    int w = 0;
    buf[0] = 0;
    if (v != v) { /* NaN */
        return fmt_append(buf, cap, "nan");
    }
    if (decimals < 0) decimals = 0;
    if (decimals > 6) decimals = 6;
    if (v < 0.0f) {
        buf[w++] = '-';
        v = -v;
    }
    float scale = 1.0f;
    for (int i = 0; i < decimals; i++) {
        scale *= 10.0f;
    }
    if (v > 2.0e9f) {
        buf[w] = 0;
        return fmt_append(buf, cap, "inf");
    }
    /* integer and fractional parts, rounded at the last printed digit
     * (32-bit only: no 64-bit division helpers needed on the MCU) */
    uint32_t ip = (uint32_t)v;
    uint32_t fp = (uint32_t)((v - (float)ip) * scale + 0.5f);
    if (fp >= (uint32_t)scale) {
        ip++;
        fp -= (uint32_t)scale;
    }
    w += fmt_uint(buf + w, cap - w, ip);
    if (decimals > 0 && w < cap - 1) {
        buf[w++] = '.';
        char d[8];
        for (int i = decimals - 1; i >= 0; i--) {
            d[i] = (char)('0' + fp % 10);
            fp /= 10;
        }
        for (int i = 0; i < decimals && w < cap - 1; i++) {
            buf[w++] = d[i];
        }
    }
    buf[w] = 0;
    return w;
}

bool fmt_parse_float(const char *s, float *out)
{
    float sign = 1.0f, val = 0.0f;
    bool digits = false;
    while (*s == ' ') s++;
    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') { s++; }
    while (*s >= '0' && *s <= '9') {
        val = val * 10.0f + (float)(*s - '0');
        s++;
        digits = true;
    }
    if (*s == '.') {
        s++;
        float f = 0.1f;
        while (*s >= '0' && *s <= '9') {
            val += f * (float)(*s - '0');
            f *= 0.1f;
            s++;
            digits = true;
        }
    }
    if (!digits) {
        return false;
    }
    if (*s == 'e' || *s == 'E') {
        s++;
        int es = 1, e = 0;
        if (*s == '-') { es = -1; s++; }
        else if (*s == '+') { s++; }
        if (!(*s >= '0' && *s <= '9')) {
            return false;
        }
        while (*s >= '0' && *s <= '9') {
            e = e * 10 + (*s - '0');
            s++;
        }
        for (int i = 0; i < e && i < 38; i++) {
            val = es > 0 ? val * 10.0f : val * 0.1f;
        }
    }
    while (*s == ' ') s++;
    if (*s != 0) {
        return false;
    }
    *out = sign * val;
    return true;
}
