/*!
    \file  imu_link.c
    \brief IMU-DXL servo-bus reader: framing, decoding and JSON rendering.

    Mirrors sdk/python/imu_link.py operation for operation; the two are kept
    byte-compatible on purpose so a capture decodes to the same JSON text.
*/

#include "imu_link.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define GYRO_LSB_PER_DPS 16.4
#define DXL_HEADER_LEN   4

static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    size_t i;
    int bit;
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x8005) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static int16_t i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static double half_to_float(uint16_t bits)
{
    double sign = (bits & 0x8000) ? -1.0 : 1.0;
    int exp = (bits >> 10) & 0x1F;
    int frac = bits & 0x3FF;
    if (exp == 0) return sign * (double)frac * pow(2.0, -24.0);
    if (exp == 0x1F) return frac ? NAN : sign * INFINITY;
    return sign * (1.0 + (double)frac / 1024.0) * pow(2.0, (double)(exp - 15));
}

static void drop_front(imu_parser_t *p, size_t n)
{
    if (n > p->len) n = p->len;
    memmove(p->buf, p->buf + n, p->len - n);
    p->len -= n;
}

int imu_decode_pose(const uint8_t block[12], double gyro_dps[3],
                    double quat_wxyz[4], double euler_deg[3])
{
    double x, y, z, w, n;
    int i;
    if (!block) return 1;

    for (i = 0; i < 3; i++) gyro_dps[i] = (double)i16(block + 2 * i) / GYRO_LSB_PER_DPS;

    x = half_to_float((uint16_t)(block[6] | (block[7] << 8)));
    y = half_to_float((uint16_t)(block[8] | (block[9] << 8)));
    z = half_to_float((uint16_t)(block[10] | (block[11] << 8)));
    w = sqrt(fmax(0.0, 1.0 - x * x - y * y - z * z));
    n = sqrt(w * w + x * x + y * y + z * z);
    if (n == 0.0) n = 1.0;
    w /= n; x /= n; y /= n; z /= n;
    quat_wxyz[0] = w; quat_wxyz[1] = x; quat_wxyz[2] = y; quat_wxyz[3] = z;

    {
        double roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        double sp = 2.0 * (w * y - z * x);
        double pitch, yaw;
        if (sp > 1.0) sp = 1.0;
        if (sp < -1.0) sp = -1.0;
        pitch = asin(sp);
        yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        euler_deg[0] = roll * (180.0 / 3.14159265358979323846);
        euler_deg[1] = pitch * (180.0 / 3.14159265358979323846);
        euler_deg[2] = yaw * (180.0 / 3.14159265358979323846);
    }
    return 0;
}

void imu_parser_init(imu_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->status = -1;
}

/* 0 = need more, 1 = consumed without a sample, 2 = emitted a sample */
static int parse_feetech(imu_parser_t *p, imu_sample_cb cb, void *ctx)
{
    size_t total = (size_t)p->buf[3] + 4;
    size_t i;
    uint8_t sum = 0;

    if (total < 6 || total > 140) { drop_front(p, 1); return 1; }
    if (p->len < total) return 0;
    for (i = 2; i < total; i++) sum = (uint8_t)(sum + p->buf[i]);
    if (sum != 0xFF) { drop_front(p, 1); return 1; }
    if (p->buf[2] != IMU_ID) { drop_front(p, total); return 1; }

    p->seen = 1;
    if (total - 6 == IMU_FEETECH_POSE_LEN) {
        imu_sample_t s;
        s.proto = "feetech";
        s.seq = p->buf[5 + 12];
        s.status = p->buf[5 + 13];
        imu_decode_pose(p->buf + 5, s.gyro_dps, s.quat_wxyz, s.euler_deg);
        drop_front(p, total);
        if (cb) cb(&s, ctx);
        return 2;
    }
    drop_front(p, total);
    return 1;
}

static int parse_dxl(imu_parser_t *p, imu_sample_cb cb, void *ctx)
{
    size_t total, i, blen = 0;
    uint16_t want;
    uint8_t body[260];

    if (p->len < 7) return 0;
    total = 7 + ((size_t)p->buf[5] | ((size_t)p->buf[6] << 8));
    if (total < 10 || total > 260) { drop_front(p, 1); return 1; }
    if (p->len < total) return 0;
    want = (uint16_t)(p->buf[total - 2] | (p->buf[total - 1] << 8));
    if (crc16(p->buf, total - 2) != want) { drop_front(p, 1); return 1; }
    if (p->buf[4] != IMU_ID) { drop_front(p, total); return 1; }

    p->seen = 2;
    for (i = 7; i < total - 2; i++) {
        uint8_t b = p->buf[i];
        if (blen >= 3 && b == 0xFD && body[blen - 1] == 0xFD &&
            body[blen - 2] == 0xFF && body[blen - 3] == 0xFF) continue;
        body[blen++] = b;
    }
    if (blen < 2 || body[0] != 0x55) { drop_front(p, total); return 1; }

    if (blen - 2 == IMU_DXL_POSE_LEN) {
        imu_sample_t s;
        s.proto = "dxl";
        s.seq = -1;
        s.status = p->status;
        imu_decode_pose(body + 2, s.gyro_dps, s.quat_wxyz, s.euler_deg);
        drop_front(p, total);
        if (cb) cb(&s, ctx);
        return 2;
    }
    if (blen - 2 == 1) p->status = body[2];
    drop_front(p, total);
    return 1;
}

static int parse_one(imu_parser_t *p, imu_sample_cb cb, void *ctx)
{
    size_t i;

    if (p->len < 2) return 0;
    for (i = 0; i + 1 < p->len; i++) {
        if (p->buf[i] == 0xFF && p->buf[i + 1] == 0xFF) break;
    }
    if (i + 1 >= p->len) {
        /* no header: keep one trailing 0xFF, it may start the next one */
        if (p->len && p->buf[p->len - 1] == 0xFF) {
            p->buf[0] = 0xFF;
            p->len = 1;
        } else {
            p->len = 0;
        }
        return 0;
    }
    if (i) drop_front(p, i);
    if (p->len < 4) return 0;
    if (p->buf[0] == 0xFF && p->buf[1] == 0xFF && p->buf[2] == 0xFD && p->buf[3] == 0x00) {
        return parse_dxl(p, cb, ctx);
    }
    return parse_feetech(p, cb, ctx);
}

size_t imu_parser_feed(imu_parser_t *p, const uint8_t *data, size_t len,
                       imu_sample_cb cb, void *ctx)
{
    size_t produced = 0;

    if (len > sizeof(p->buf)) {                 /* keep the tail, that is where frames start */
        data += len - sizeof(p->buf);
        len = sizeof(p->buf);
    }
    if (p->len + len > sizeof(p->buf)) drop_front(p, p->len + len - sizeof(p->buf));
    memcpy(p->buf + p->len, data, len);
    p->len += len;

    for (;;) {
        int r = parse_one(p, cb, ctx);
        if (r == 0) break;
        if (r == 2) produced++;
    }
    return produced;
}

/* ------------------------------ JSON output ------------------------------- */

static void fmt4(double v, char *dst, size_t len)
{
    if (v == 0.0) v = 0.0;                      /* clear -0.0 */
    snprintf(dst, len, "%.4f", v);
}

void imu_sample_to_json(const imu_sample_t *s, char *dst, size_t dst_len)
{
    char g[3][24], q[4][24], e[3][24], seq[16], status[16];
    int i;

    for (i = 0; i < 3; i++) fmt4(s->gyro_dps[i], g[i], sizeof(g[i]));
    for (i = 0; i < 4; i++) fmt4(s->quat_wxyz[i], q[i], sizeof(q[i]));
    for (i = 0; i < 3; i++) fmt4(s->euler_deg[i], e[i], sizeof(e[i]));
    if (s->seq < 0) strcpy(seq, "null"); else snprintf(seq, sizeof(seq), "%d", s->seq);
    if (s->status < 0) strcpy(status, "null"); else snprintf(status, sizeof(status), "%d", s->status);

    snprintf(dst, dst_len,
             "{\"proto\":\"%s\",\"seq\":%s,\"status\":%s,"
             "\"gyro_dps\":[%s,%s,%s],\"quat_wxyz\":[%s,%s,%s,%s],"
             "\"euler_deg\":[%s,%s,%s]}",
             s->proto, seq, status,
             g[0], g[1], g[2],
             q[0], q[1], q[2], q[3],
             e[0], e[1], e[2]);
}

/* ---------------------------- request builders ---------------------------- */

size_t imu_feetech_read(uint8_t addr, uint8_t len, uint8_t *out)
{
    uint8_t sum = 0;
    size_t i;
    out[0] = 0xFF; out[1] = 0xFF; out[2] = IMU_ID; out[3] = 4;
    out[4] = 0x02; out[5] = addr; out[6] = len;
    for (i = 2; i < 7; i++) sum = (uint8_t)(sum + out[i]);
    out[7] = (uint8_t)~sum;
    return 8;
}

size_t imu_feetech_ping(uint8_t *out)
{
    uint8_t sum = 0;
    size_t i;
    out[0] = 0xFF; out[1] = 0xFF; out[2] = IMU_ID; out[3] = 2; out[4] = 0x01;
    for (i = 2; i < 5; i++) sum = (uint8_t)(sum + out[i]);
    out[5] = (uint8_t)~sum;
    return 6;
}

static size_t dxl_packet(uint8_t instruction, const uint8_t *params, size_t nparams, uint8_t *out)
{
    uint8_t body[32];
    uint8_t stuffed[64];
    size_t body_len = 0, n = 0, frame_len, i;
    uint16_t crc;

    body[body_len++] = instruction;
    for (i = 0; i < nparams; i++) body[body_len++] = params[i];

    /* stuff FF FF FD inside the instruction/parameter region */
    for (i = 0; i < body_len; i++) {
        stuffed[n++] = body[i];
        if (n >= 3 && stuffed[n - 1] == 0xFD && stuffed[n - 2] == 0xFF && stuffed[n - 3] == 0xFF) {
            stuffed[n++] = 0xFD;
        }
    }
    memcpy(body, stuffed, n);
    body_len = n;

    out[0] = 0xFF; out[1] = 0xFF; out[2] = 0xFD; out[3] = 0x00; out[4] = IMU_ID;
    out[5] = (uint8_t)((body_len + 2) & 0xFF);
    out[6] = (uint8_t)((body_len + 2) >> 8);
    memcpy(out + 7, body, body_len);
    frame_len = 7 + body_len;
    crc = crc16(out, frame_len);
    out[frame_len] = (uint8_t)(crc & 0xFF);
    out[frame_len + 1] = (uint8_t)(crc >> 8);
    return frame_len + 2;
}

size_t imu_dxl_read(uint16_t addr, uint16_t len, uint8_t *out)
{
    uint8_t params[4];
    params[0] = (uint8_t)(addr & 0xFF);
    params[1] = (uint8_t)(addr >> 8);
    params[2] = (uint8_t)(len & 0xFF);
    params[3] = (uint8_t)(len >> 8);
    return dxl_packet(0x02, params, 4, out);
}

size_t imu_dxl_ping(uint8_t *out)
{
    return dxl_packet(0x01, NULL, 0, out);
}
