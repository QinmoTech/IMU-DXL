# IMU-DXL SDK

从同一条 1 Mbps 单线舵机总线上读 IMU（设备 ID 200），支持两种协议：

- 飞特 SCS/STS（姿态块 地址 56 / 15 字节）
- DYNAMIXEL Protocol 2.0（姿态块 地址 124 / 12 字节，状态字节 地址 143 / 1 字节）

C、Python、Rust 三种实现输出**同一种 JSON 行**，同一份抓包解出来的文本逐字节相同，
方便把任意一种嵌进自己的工程。

## 输出格式

每个姿态帧打印一行，无缩进、无空格：

```json
{"proto":"feetech","seq":42,"status":0,"gyro_dps":[-0.0610,31.0366,0.0000],"quat_wxyz":[0.8660,0.5000,0.0000,0.0000],"euler_deg":[60.0000,0.0000,0.0000]}
```

| 字段 | 说明 |
|---|---|
| `proto` | 这一帧来自哪种协议：`feetech` / `dxl` |
| `seq` | 飞特的采样序号；DYNAMIXEL 没有，固定 `null` |
| `status` | 飞特块内状态字节，或 DYNAMIXEL 地址 143；未知时 `null` |
| `gyro_dps` | 三轴角速度，原始计数 ÷ 16.4 |
| `quat_wxyz` | 四元数，W 由 `sqrt(max(0,1-x²-y²-z²))` 恢复并归一化 |
| `euler_deg` | ZYX 欧拉角，单位度 |

## 命令行

三种语言的参数一致：

```
--port COM5                 串口（1 Mbps 8N1）
--protocol auto|feetech|dxl 默认 auto：先发飞特 PING，无应答再发 DYNAMIXEL PING
--count N                   ​打印 N 帧后退出（0 = 一直读）
--replay FILE               不进串口，直接解一份抓包（离线/CI 用）
```

### Python

```bash
cd sdk/python
pip install pyserial            # 只有 --port 需要；--replay 不需要任何依赖
python read_imu.py --port COM5
python read_imu.py --replay ../testdata/capture.bin
```

`imu_link.py` 是个普通模块，可以直接 `import`：`Parser().feed(chunk)` 返回 `Sample` 列表，
`Sample.to_json()` 给出上面那行文本。

### C

```bash
cd sdk/c
make                            # -> read_imu.exe（gcc / mingw32-make）
read_imu --port COM5
read_imu --replay ../testdata/capture.bin
```

只有 `imu_link.c/h` 与平台无关；串口部分在 Windows 走 Win32，在 Linux/macOS 走
termios（`B1000000`）。把这两个文件加进工程即可，不需要额外依赖。

### Rust

```bash
cd sdk/rust
cargo run --release -- --port COM5           # 默认 feature "serial" 会拉 serialport crate
cargo run --release --no-default-features -- --replay ../testdata/capture.bin
```

`src/lib.rs` 不依赖任何 crate（解析、解码、JSON 全在 std 里）；只有 CLI 的实时串口
需要 `serial` feature 与 `serialport` 依赖。`--replay` 在 `--no-default-features` 下也能用，
所以离线环境可以完整跑测试。

## 测试

三方共用同一份合成抓包与同一份期望输出：

- `testdata/capture.bin` — 由 `testdata/make_capture.py` 生成：飞特两帧、DYNAMIXEL 状态帧、
  DYNAMIXEL 两帧，中间夹了噪声字节和一帧坏校验，用来验证重同步。
- `testdata/expected.ndjson` — 四行期望输出。

```bash
python sdk/testdata/make_capture.py        # 重新生成抓包（一般不需要）

cd sdk/python && python -m unittest -v     # 9 项
cd sdk/c      && make test                 # 22 项
cd sdk/rust   && cargo test                # 单元测试 + tests/replay.rs
```

C 与 Rust 的测试都会把自己的输出和 `expected.ndjson` 逐行比较，所以三种实现只要有一种
跑偏（浮点格式、四元数符号、序号/状态归属、帧同步），测试就会失败。

### 实时串口（Linux，PTY）

单元测试只喂字节，不碰串口。`testdata/live_linux.py` 补这一段：`fake_device.py` 用
`pty.openpty()` 造一个假设备（按请求回帧，帧由它自己独立构造），真正的 CLI 通过
`/dev/pts/N` 连上去，验证 termios 配置、协议探测、轮询与退出：

```bash
cd sdk/c && make                                # 可选：连 C 实现一起测
python sdk/testdata/live_linux.py --c-binary sdk/c/read_imu
```

Windows 没有 PTY，这一项在 Windows 上跳过；Windows 的实时路径需要真实适配器加设备，
解析与解码部分已由 `--replay` 覆盖。

### CI

`.github/workflows/sdk.yml`：

- `ubuntu-latest` 与 `windows-latest` 各跑一遍 Python 单测、C 编译 + 单测、`--replay` 与 golden 对比；
- Linux 额外跑上面的 PTY 实时测试（C 与 Python 两个客户端都跑）；
- Ubuntu 上跑 Rust 的 `cargo test`（先 `--no-default-features`，再带 `serialport`）。

## Windows / Linux

串口层只有两套实现，编译时二选一，对外 API 相同：Windows 走 Win32 COMM API，Linux 走 termios
（`c/imu_port.c`）。Python 用 pyserial、Rust 用 serialport，这两个库本身跨平台。

| | Windows 10 | Linux |
|---|---|---|
| 端口名 | `COM5`（COM10 及以上自动加 `\\.\` 前缀） | `/dev/ttyUSB0`、`/dev/ttyACM0` |
| 权限 | 无需额外设置 | 用户在 `dialout`（Debian/Ubuntu）或 `uucp`（Arch）组：`sudo usermod -aG dialout $USER`，然后重新登录 |
| 1 Mbps | 适配器要支持；CH340 / CP210x / FTDI 都可以 | 需要 termios 的 `B1000000`；没有该常量的 libc 会给出明确报错 |
| 列端口 | `python read_imu.py --list` | 同左 |
| C 依赖 | 无（只多 `-lm`） | 无（只多 `-lm`） |
| Rust 依赖 | `serialport`，默认特性即可 | 本 SDK 关掉了 `serialport` 的默认特性，所以不需要 `libudev-dev` |

C 的两个文件可以直接拿走复用：`imu_link.c/h` 与平台无关（帧解析、解码、JSON），
`imu_port.c/h` 是串口层。Windows 下适配器不支持 1 Mbps 时，`SetCommState` 会失败并打印
Win32 错误码；Linux 下常见失败是权限（`EACCES`）或端口被别的程序占用（`EBUSY`）。

## 协议要点

完整说明见 [`../docs/PROTOCOL.md`](../docs/PROTOCOL.md)。SDK 只做三件事：发请求、找帧边界、解码。

- 飞特：`FF FF ID LEN INSTRUCTION|ERROR DATA... CHECKSUM`，校验是 `~sum(ID..最后一个数据字节)`。
- DYNAMIXEL：`FF FF FD 00 ID LEN_L LEN_H ... CRC_L CRC_H`，CRC-16 多项式 `0x8005`、初值 0，
  指令/参数区里的 `FF FF FD` 要插入一个 `FD`。
- 一帧里 `ID` 不是 200、校验/CRC 错、或长度不合理时，解析器只丢掉一个字节再重新找帧头，
  不会因为一帧坏数据丢掉后面的数据。
- 自动识别按帧头：`FF FF FD 00` 是 DYNAMIXEL，其余 `FF FF <id>` 是飞特。所以一份混有
  两种协议的抓包也能直接解。
