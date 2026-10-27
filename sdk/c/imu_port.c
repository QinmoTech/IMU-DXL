/*!
    \file  imu_port.c
    \brief Serial port, Windows (Win32) and Linux (termios).
*/

/* glibc hides cfmakeraw(), the B1000000 constant and select()'s pieces behind
   feature macros; ask for them explicitly so the build does not depend on the
   compiler default (see also the gnu99 note in the Makefile). */
#if !defined(_WIN32) && !defined(_DEFAULT_SOURCE)
#define _DEFAULT_SOURCE 1
#endif

#include "imu_port.h"

#include <stdio.h>
#include <string.h>

#define IMU_PORT_BAUD 1000000

#if defined(_WIN32)

#include <windows.h>

int imu_port_open(imu_port_t *port, const char *name)
{
    char path[64];
    DCB dcb;
    COMMTIMEOUTS timeouts;

    /* COM10 and above need the device namespace prefix; harmless below it. */
    if (strncmp(name, "\\\\.\\", 4) == 0) snprintf(path, sizeof(path), "%s", name);
    else snprintf(path, sizeof(path), "\\\\.\\%s", name);

    port->handle = (void *)CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                       OPEN_EXISTING, 0, NULL);
    if (port->handle == (void *)INVALID_HANDLE_VALUE) {
        fprintf(stderr, "imu_port: cannot open %s (Win32 error %lu)\n", path, GetLastError());
        port->handle = NULL;
        return -1;
    }
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState((HANDLE)port->handle, &dcb)) {
        fprintf(stderr, "imu_port: GetCommState failed\n");
        imu_port_close(port);
        return -1;
    }
    dcb.BaudRate = IMU_PORT_BAUD;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fAbortOnError = FALSE;
    if (!SetCommState((HANDLE)port->handle, &dcb)) {
        fprintf(stderr,
                "imu_port: SetCommState failed - this adapter may not do 1 Mbps (Win32 error %lu)\n",
                GetLastError());
        imu_port_close(port);
        return -1;
    }
    SetupComm((HANDLE)port->handle, 4096, 4096);
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 100;
    SetCommTimeouts((HANDLE)port->handle, &timeouts);
    return 0;
}

void imu_port_close(imu_port_t *port)
{
    if (port && port->handle) {
        CloseHandle((HANDLE)port->handle);
        port->handle = NULL;
    }
}

int imu_port_write(imu_port_t *port, const uint8_t *data, size_t len)
{
    DWORD written = 0;
    if (!port || !port->handle) return -1;
    if (!WriteFile((HANDLE)port->handle, data, (DWORD)len, &written, NULL)) return -1;
    return (int)written;
}

int imu_port_read(imu_port_t *port, uint8_t *buf, size_t len, int timeout_ms)
{
    COMMTIMEOUTS timeouts;
    DWORD got = 0;
    if (!port || !port->handle) return -1;
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = (DWORD)timeout_ms;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 100;
    SetCommTimeouts((HANDLE)port->handle, &timeouts);
    if (!ReadFile((HANDLE)port->handle, buf, (DWORD)len, &got, NULL)) return -1;
    return (int)got;                      /* 0 when the timeout expired */
}

void imu_port_flush_input(imu_port_t *port)
{
    if (port && port->handle) PurgeComm((HANDLE)port->handle, PURGE_RXCLEAR);
}

void imu_port_sleep_ms(int ms)
{
    Sleep((DWORD)ms);
}

#else /* POSIX: Linux and other termios systems */

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/types.h>

int imu_port_open(imu_port_t *port, const char *name)
{
    struct termios tty;
    int fd = open(name, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd < 0) {
        fprintf(stderr, "imu_port: cannot open %s: %s\n", name, strerror(errno));
        return -1;
    }
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "imu_port: tcgetattr(%s) failed: %s\n", name, strerror(errno));
        close(fd);
        return -1;
    }
    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(CSTOPB | PARENB);
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;              /* no hardware flow control */
#endif
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
#ifdef B1000000
    cfsetispeed(&tty, B1000000);
    cfsetospeed(&tty, B1000000);
#else
    fprintf(stderr,
            "imu_port: this libc has no B1000000; the 1 Mbps bus needs a Linux "
            "termios that defines it\n");
    close(fd);
    return -1;
#endif
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "imu_port: tcsetattr(%s) failed: %s\n", name, strerror(errno));
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    port->fd = fd;
    return 0;
}

void imu_port_close(imu_port_t *port)
{
    if (port && port->fd >= 0) {
        close(port->fd);
        port->fd = -1;
    }
}

int imu_port_write(imu_port_t *port, const uint8_t *data, size_t len)
{
    size_t sent = 0;
    if (!port || port->fd < 0) return -1;
    while (sent < len) {
        ssize_t n = write(port->fd, data + sent, len - sent);
        if (n > 0) { sent += (size_t)n; continue; }
        if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
            fd_set set;
            struct timeval tv = {0, 1000};
            FD_ZERO(&set);
            FD_SET(port->fd, &set);
            select(port->fd + 1, NULL, &set, NULL, &tv);
            continue;
        }
        return -1;
    }
    return (int)sent;
}

int imu_port_read(imu_port_t *port, uint8_t *buf, size_t len, int timeout_ms)
{
    fd_set set;
    struct timeval tv;
    int ready;
    ssize_t n;

    if (!port || port->fd < 0) return -1;
    FD_ZERO(&set);
    FD_SET(port->fd, &set);
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    ready = select(port->fd + 1, &set, NULL, NULL, &tv);
    if (ready < 0) {
        if (errno == EINTR) return 0;
        return -1;
    }
    if (ready == 0) return 0;             /* timeout */
    n = read(port->fd, buf, len);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
        return -1;
    }
    return (int)n;
}

void imu_port_flush_input(imu_port_t *port)
{
    if (port && port->fd >= 0) tcflush(port->fd, TCIFLUSH);
}

void imu_port_sleep_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {
    }
}

#endif /* _WIN32 */
