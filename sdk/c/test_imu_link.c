/*!
    \file  test_imu_link.c
    \brief Offline tests: frame builders, decoding, resynchronisation, and the
           shared golden capture.

    Run from sdk/c:   make test   (or: test_imu_link ../testdata/capture.bin)
*/

#include "imu_link.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int checks, failures;

static void check(int ok, const char *name, const char *detail)
{
    checks++;
    if (!ok) failures++;
    printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", name,
           detail && *detail ? " ??" : "", detail ? detail : "");
}

static void to_hex(const uint8_t *data, size_t len, char *dst)
{
    size_t i;
    dst[0] = 0;
    for (i = 0; i < len; i++) sprintf(dst + 3 * i, "%02X ", data[i]);
    if (len) dst[3 * len - 1] = 0;
}

typedef struct {
    char lines[16][512];
    int count;
} collector;

static void collect(const imu_sample_t *sample, void *ctx)
{
    collector *c = (collector *)ctx;
    if (c->count >= 16) return;
    imu_sample_to_json(sample, c->lines[c->count], sizeof(c->lines[0]));
    c->count++;
}

static int load_file(const char *path, uint8_t *buf, size_t cap, size_t *len)
{
    FILE *file = fopen(path, "rb");
    if (!file) return -1;
    *len = fread(buf, 1, cap, file);
    fclose(file);
    return 0;
}

int main(int argc, char **argv)
{
    const char *capture = argc > 1 ? argv[1] : "../testdata/capture.bin";
    const char *expected = argc > 2 ? argv[2] : "../testdata/expected.ndjson";
    uint8_t frame[32];
    size_t n;
    char hex[128];

    printf("IMU-DXL C SDK tests\n");

    printf("\n1. request frames match docs/PROTOCOL.md\n");
    n = imu_feetech_ping(frame); to_hex(frame, n, hex);
    check(!strcmp(hex, "FF FF C8 02 01 34"), "Feetech PING", hex);
    n = imu_feetech_read(56, 15, frame); to_hex(frame, n, hex);
    check(!strcmp(hex, "FF FF C8 04 02 38 0F EA"), "Feetech READ 56/15", hex);
    n = imu_dxl_ping(frame); to_hex(frame, n, hex);
    check(!strcmp(hex, "FF FF FD 00 C8 03 00 01 3B FA"), "DYNAMIXEL PING", hex);
    n = imu_dxl_read(124, 12, frame); to_hex(frame, n, hex);
    check(!strcmp(hex, "FF FF FD 00 C8 07 00 02 7C 00 0C 00 AB D7"), "DYNAMIXEL READ 124/12", hex);

    printf("\n2. decoding a known block\n");
    {
        /* gyro -1, 509, 0 ; quaternion x = 0.5 (half 0x3800), y = z = 0 */
        const uint8_t block[12] = {0xFF, 0xFF, 0xFD, 0x01, 0, 0, 0x00, 0x38, 0, 0, 0, 0};
        double g[3], q[4], e[3];
        check(imu_decode_pose(block, g, q, e) == 0, "block decodes", "");
        check(fabs(g[0] + 0.06097560975609756) < 1e-9, "gyro X = -1/16.4", "");
        check(fabs(g[1] - 31.036585365853657) < 1e-9, "gyro Y = 509/16.4", "");
        check(fabs(q[0] - 0.8660254037844386) < 1e-6, "quat w rebuilt", "");
        check(fabs(q[1] - 0.5) < 1e-6, "quat x", "");
        check(fabs(e[0] - 60.0) < 1e-6, "roll = 60 deg", "");
        check(fabs(e[1]) < 1e-9 && fabs(e[2]) < 1e-9, "pitch/yaw = 0", "");
    }

    printf("\n3. capture: whole file, then one byte at a time\n");
    {
        uint8_t data[4096];
        size_t len = 0, i;
        collector whole = {0}, drip = {0}, expect = {0};
        imu_parser_t parser;

        if (load_file(capture, data, sizeof(data), &len) != 0) {
            printf("  [FAIL] cannot read %s\n", capture);
            failures++;
            return 1;
        }
        imu_parser_init(&parser);
        imu_parser_feed(&parser, data, len, collect, &whole);
        check(whole.count == 4, "four samples in the capture", "");

        imu_parser_init(&parser);
        for (i = 0; i < len; i++) imu_parser_feed(&parser, data + i, 1, collect, &drip);
        check(drip.count == whole.count, "byte-at-a-time gives the same count", "");

        {
            FILE *file = fopen(expected, "r");
            char line[512];
            if (!file) {
                printf("  [FAIL] cannot read %s\n", expected);
                failures++;
                return 1;
            }
            while (fgets(line, sizeof(line), file) && expect.count < 16) {
                line[strcspn(line, "\r\n")] = 0;
                if (*line) strcpy(expect.lines[expect.count++], line);
            }
            fclose(file);
        }
        check(expect.count == whole.count, "golden file line count matches", "");
        for (i = 0; i < (size_t)whole.count && i < (size_t)expect.count; i++) {
            check(!strcmp(whole.lines[i], expect.lines[i]), "golden line matches", whole.lines[i]);
            check(!strcmp(drip.lines[i], expect.lines[i]), "drip line matches", drip.lines[i]);
        }
    }

    printf("\n%d/%d checks passed\n", checks - failures, checks);
    return failures ? 1 : 0;
}
