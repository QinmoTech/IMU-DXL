# `cc` is make's built-in default; prefer gcc when the caller did not choose.
ifeq ($(origin CC),default)
CC := gcc
endif
# gnu99, not c99: strict ISO mode makes glibc hide the POSIX declarations the
# termios branch needs (fd_set/select, cfmakeraw, nanosleep, ...). MSVC/MinGW
# do not care, so one flag serves both platforms.
CFLAGS  ?= -O2 -Wall -Wextra -std=gnu99
LDLIBS  ?= -lm

ifeq ($(OS),Windows_NT)
EXE := .exe
else
EXE :=
endif

BIN  = read_imu$(EXE)
TEST = test_imu_link$(EXE)
SRC  = imu_link.c imu_port.c
HDR  = imu_link.h imu_port.h

.PHONY: all test clean

all: $(BIN)

$(BIN): read_imu.c $(SRC) $(HDR)
	$(CC) $(CFLAGS) -o $@ read_imu.c $(SRC) $(LDLIBS)

$(TEST): test_imu_link.c $(SRC) $(HDR)
	$(CC) $(CFLAGS) -o $@ test_imu_link.c $(SRC) $(LDLIBS)

test: $(TEST)
	./$(TEST) ../testdata/capture.bin ../testdata/expected.ndjson

clean:
	-rm -f $(BIN) $(TEST)
