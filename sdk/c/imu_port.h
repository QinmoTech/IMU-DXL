#ifndef IMU_PORT_H
#define IMU_PORT_H

/*!
    \file  imu_port.h
    \brief Minimal serial-port layer: Windows (Win32 COMM API) and Linux
           (termios). One API, two implementations, chosen at compile time.

    The port is always opened 1 Mbps 8N1, no flow control, which is what the
    IMU-DXL bus runs at.

    Windows: `"COM5"` (the `\\.\` prefix is added for COM10 and above).
    Linux:   `"/dev/ttyUSB0"` (usually needs membership of `dialout`).

    Reads are bounded by a timeout so a caller can poll without blocking.
*/

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
typedef struct { void *handle; } imu_port_t;    /* HANDLE, kept out of the header */
#else
typedef struct { int fd; } imu_port_t;
#endif

/*! \brief Open the port at 1 Mbps 8N1. Returns 0 on success. */
int imu_port_open(imu_port_t *port, const char *name);

void imu_port_close(imu_port_t *port);

/*! \brief Write all bytes (blocking, bounded). Returns bytes written or -1. */
int imu_port_write(imu_port_t *port, const uint8_t *data, size_t len);

/*! \brief Read up to len bytes, waiting at most timeout_ms.
    Returns bytes read, 0 on timeout, -1 on a hard error. */
int imu_port_read(imu_port_t *port, uint8_t *buf, size_t len, int timeout_ms);

/*! \brief Drop anything already received. */
void imu_port_flush_input(imu_port_t *port);

/*! \brief Sleep, so a poll loop paces the same way on both platforms. */
void imu_port_sleep_ms(int ms);

#endif /* IMU_PORT_H */
