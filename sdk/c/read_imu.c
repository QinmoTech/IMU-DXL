/*!
    \file  read_imu.c
    \brief Read IMU-DXL pose frames and print one JSON line per sample.

        read_imu --port COM5              (Windows)
        read_imu --port /dev/ttyUSB0      (Linux)
        read_imu --protocol dxl --count 20
        read_imu --replay ../testdata/capture.bin

    All platform code lives in imu_port.c; this file is the same on both.
    Output is byte-identical to the Python and Rust readers for a capture.
*/

#include "imu_link.h"
#include "imu_port.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int target_count;
static int printed;

static void on_sample(const imu_sample_t *sample, void *ctx)
{
    char line[512];
    (void)ctx;
    if (target_count && printed >= target_count) return;
    imu_sample_to_json(sample, line, sizeof(line));
    puts(line);
    fflush(stdout);
    printed++;
}

static int replay(const char *path)
{
    FILE *file = fopen(path, "rb");
    imu_parser_t parser;
    uint8_t chunk[256];
    size_t n;

    if (!file) {
        fprintf(stderr, "cannot open %s\n", path);
        return 1;
    }
    imu_parser_init(&parser);
    while ((n = fread(chunk, 1, sizeof(chunk), file)) > 0) {
        imu_parser_feed(&parser, chunk, n, on_sample, NULL);
        if (target_count && printed >= target_count) break;
    }
    fclose(file);
    if (!printed) {
        fprintf(stderr, "no valid frames in the capture\n");
        return 1;
    }
    return 0;
}

static int wait_for_frame(imu_parser_t *parser, imu_port_t *port, int want_seen, int ms)
{
    uint8_t buf[256];
    int waited = 0;
    while (waited < ms) {
        int got = imu_port_read(port, buf, sizeof(buf), 20);
        if (got > 0) {
            imu_parser_feed(parser, buf, (size_t)got, NULL, NULL);
            if (parser->seen == want_seen) return 0;
        }
        waited += 20;
    }
    return -1;
}

static int live(const char *name, const char *protocol)
{
    imu_port_t port;
    imu_parser_t parser;
    uint8_t frame[32];
    size_t len;
    int chosen = 0;
    int last_status_ms = -1000;          /* read address 143 before the first pose */
    int elapsed = 0;

    if (imu_port_open(&port, name) != 0) return 1;
    imu_parser_init(&parser);

    if (strcmp(protocol, "feetech") == 0) chosen = 1;
    else if (strcmp(protocol, "dxl") == 0) chosen = 2;
    else {
        len = imu_feetech_ping(frame);
        imu_port_write(&port, frame, len);
        if (wait_for_frame(&parser, &port, 1, 300) == 0) {
            chosen = 1;
        } else {
            imu_port_flush_input(&port);
            len = imu_dxl_ping(frame);
            imu_port_write(&port, frame, len);
            if (wait_for_frame(&parser, &port, 2, 300) == 0) chosen = 2;
        }
    }
    if (!chosen) {
        fprintf(stderr, "no reply from ID %d: check wiring, ground, baud rate and protocol\n", IMU_ID);
        imu_port_close(&port);
        return 1;
    }
    fprintf(stderr, "# %s @ %d 8N1 - protocol %s\n", name, IMU_BAUD,
            chosen == 1 ? "feetech" : "dxl");

    for (;;) {
        int got;
        if (chosen == 2 && elapsed - last_status_ms >= 1000) {
            last_status_ms = elapsed;
            len = imu_dxl_read(IMU_DXL_STATUS_ADDR, IMU_DXL_STATUS_LEN, frame);
            imu_port_write(&port, frame, len);
        }
        len = chosen == 1
                  ? imu_feetech_read(IMU_FEETECH_POSE_ADDR, IMU_FEETECH_POSE_LEN, frame)
                  : imu_dxl_read(IMU_DXL_POSE_ADDR, IMU_DXL_POSE_LEN, frame);
        imu_port_write(&port, frame, len);

        got = imu_port_read(&port, frame, sizeof(frame), 20);
        if (got > 0) imu_parser_feed(&parser, frame, (size_t)got, on_sample, NULL);
        if (target_count && printed >= target_count) break;
        imu_port_sleep_ms(10);
        elapsed += 30;
    }
    imu_port_close(&port);
    return 0;
}

int main(int argc, char **argv)
{
    const char *port = NULL, *protocol = "auto", *replay_path = NULL;
    int i;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = argv[++i];
        else if (!strcmp(argv[i], "--protocol") && i + 1 < argc) protocol = argv[++i];
        else if (!strcmp(argv[i], "--count") && i + 1 < argc) target_count = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--replay") && i + 1 < argc) replay_path = argv[++i];
        else {
            fprintf(stderr,
                    "usage: %s [--port COM5|/dev/ttyUSB0] "
                    "[--protocol auto|feetech|dxl] [--count N] [--replay FILE]\n", argv[0]);
            return 2;
        }
    }
    if (replay_path) return replay(replay_path);
    if (!port) {
        fprintf(stderr, "either --port or --replay is required\n");
        return 2;
    }
    return live(port, protocol);
}
