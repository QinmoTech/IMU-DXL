# `cc` is make's built-in default; prefer gcc when the caller did not choose.
ifeq ($(origin CC),default)
CC := gcc
endif
CFLAGS  ?= -O2 -Wall -Wextra -std=c99
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
