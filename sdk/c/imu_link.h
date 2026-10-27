#ifndef IMU_LINK_H
#define IMU_LINK_H

/*!
    \file  imu_link.h
    \brief IMU-DXL servo-bus reader: framing, decoding, and one JSON line per
           sample.

    The device answers as a read-only virtual servo, ID 200, on a 1 Mbps 8N1
    single-wire bus. Two wire formats carry the same 12-byte core block:

        Feetech SCS/STS : FF FF ID LEN INSTRUCTION|ERROR DATA... CHECKSUM
                          pose at address 56, 15 bytes
        DYNAMIXEL 2.0   : FF FF FD 00 ID LEN_L LEN_H ... CRC_L CRC_H
                          pose at address 124, 12 bytes
                          status byte at address 143, read separately

        0..5   int16 LE x3      gyro raw counts, /16.4 = deg/s
        6..11  binary16 LE x3   quaternion x, y, z; w = sqrt(max(0, 1-x^2-y^2-z^2))
        (Feetech only) 12 = sample counter, 13 = status, 14 = reserved (0)

    The parser detects each frame's format from its header, so a capture that
    mixes both decodes without being told which protocol it is.
*/

#include <stddef.h>
#include <stdint.h>

#define IMU_ID                   200
#define IMU_BAUD                 1000000
#define IMU_FEETECH_POSE_ADDR    56
#define IMU_FEETECH_POSE_LEN     15
#define IMU_DXL_POSE_ADDR        124
#define IMU_DXL_POSE_LEN         12
#define IMU_DXL_STATUS_ADDR      143
#define IMU_DXL_STATUS_LEN       1

typedef struct {
    const char *proto;          /* "feetech" or "dxl" */
    int seq;                    /* Feetech sample counter, -1 when absent */
    int status;                 /* status byte, -1 when unknown */
    double gyro_dps[3];
    double quat_wxyz[4];
    double euler_deg[3];
} imu_sample_t;

typedef struct {
    uint8_t buf[1024];
    size_t len;
    int status;                 /* last DYNAMIXEL address-143 byte, -1 unknown */
    int seen;                   /* 0 none, 1 feetech, 2 dxl */
} imu_parser_t;

typedef void (*imu_sample_cb)(const imu_sample_t *sample, void *ctx);

void imu_parser_init(imu_parser_t *parser);

/*! \brief Feed bytes; each complete sample is passed to `cb`. Returns the
    number of samples produced. */
size_t imu_parser_feed(imu_parser_t *parser, const uint8_t *data, size_t len,
                       imu_sample_cb cb, void *ctx);

/*! \brief Render one sample as the shared JSON line (no trailing newline). */
void imu_sample_to_json(const imu_sample_t *sample, char *dst, size_t dst_len);

/*! \brief Decode the 12-byte core block. Returns 0 on success. */
int imu_decode_pose(const uint8_t block[12], double gyro_dps[3],
                    double quat_wxyz[4], double euler_deg[3]);

/* ---- request builders (return the frame length) ---- */
size_t imu_feetech_ping(uint8_t *out);
size_t imu_feetech_read(uint8_t addr, uint8_t len, uint8_t *out);
size_t imu_dxl_ping(uint8_t *out);
size_t imu_dxl_read(uint16_t addr, uint16_t len, uint8_t *out);

#endif /* IMU_LINK_H */
