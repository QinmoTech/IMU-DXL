//! IMU-DXL servo-bus reader: framing, decoding, and one JSON line per sample.
//!
//! The device answers as a read-only virtual servo, ID 200, on a 1 Mbps 8N1
//! single-wire bus. Two wire formats carry the same 12-byte core block:
//!
//! ```text
//! Feetech SCS/STS : FF FF ID LEN INSTRUCTION|ERROR DATA... CHECKSUM
//!                   pose at address 56, 15 bytes
//! DYNAMIXEL 2.0   : FF FF FD 00 ID LEN_L LEN_H ... CRC_L CRC_H
//!                   pose at address 124, 12 bytes
//!                   status byte at address 143, read separately
//!
//! 0..5   int16 LE x3      gyro raw counts, /16.4 = deg/s
//! 6..11  binary16 LE x3   quaternion x, y, z; w = sqrt(max(0, 1-x^2-y^2-z^2))
//! (Feetech only) 12 = sample counter, 13 = status, 14 = reserved (0)
//! ```
//!
//! The parser detects each frame's format from its header, so a capture that
//! mixes both decodes without being told which protocol it is. Its output is
//! byte-identical to the Python and C readers; see `sdk/README.md`.

pub const ID: u8 = 200;
pub const BAUD: u32 = 1_000_000;
pub const FEETECH_POSE_ADDR: u8 = 56;
pub const FEETECH_POSE_LEN: u8 = 15;
pub const DXL_POSE_ADDR: u16 = 124;
pub const DXL_POSE_LEN: u16 = 12;
pub const DXL_STATUS_ADDR: u16 = 143;
pub const DXL_STATUS_LEN: u16 = 1;

const GYRO_LSB_PER_DPS: f64 = 16.4;
const DXL_HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Proto {
    Feetech,
    Dxl,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    pub proto: Proto,
    pub seq: Option<u16>,
    pub status: Option<u8>,
    pub gyro_dps: [f64; 3],
    pub quat_wxyz: [f64; 4],
    pub euler_deg: [f64; 3],
}

fn fmt4(v: f64, out: &mut String) {
    let v = if v == 0.0 { 0.0 } else { v }; // clear -0.0
    out.push_str(&format!("{v:.4}"));
}

impl Sample {
    /// The shared one-line JSON shape, without a trailing newline.
    pub fn to_json(&self) -> String {
        let proto = match self.proto {
            Proto::Feetech => "feetech",
            Proto::Dxl => "dxl",
        };
        let mut s = String::with_capacity(200);
        s.push_str("{\"proto\":\"");
        s.push_str(proto);
        s.push_str("\",\"seq\":");
        match self.seq {
            Some(v) => s.push_str(&v.to_string()),
            None => s.push_str("null"),
        }
        s.push_str(",\"status\":");
        match self.status {
            Some(v) => s.push_str(&v.to_string()),
            None => s.push_str("null"),
        }
        s.push_str(",\"gyro_dps\":[");
        for (i, v) in self.gyro_dps.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            fmt4(*v, &mut s);
        }
        s.push_str("],\"quat_wxyz\":[");
        for (i, v) in self.quat_wxyz.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            fmt4(*v, &mut s);
        }
        s.push_str("],\"euler_deg\":[");
        for (i, v) in self.euler_deg.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            fmt4(*v, &mut s);
        }
        s.push_str("]}");
        s
    }
}

/// IEEE-754 binary16 -> f64.
pub fn half_to_f64(bits: u16) -> f64 {
    let sign = if bits & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exp = ((bits >> 10) & 0x1F) as i32;
    let frac = (bits & 0x3FF) as f64;
    if exp == 0 {
        sign * frac * 2f64.powi(-24)
    } else if exp == 0x1F {
        if frac == 0.0 {
            sign * f64::INFINITY
        } else {
            f64::NAN
        }
    } else {
        sign * (1.0 + frac / 1024.0) * 2f64.powi(exp - 15)
    }
}

/// 12-byte core block -> (gyro deg/s, quaternion wxyz, ZYX euler deg).
pub fn decode_pose(block: &[u8]) -> ([f64; 3], [f64; 4], [f64; 3]) {
    assert!(block.len() >= 12, "pose block needs 12 bytes");
    let g = |i: usize| {
        i16::from_le_bytes([block[i], block[i + 1]]) as f64 / GYRO_LSB_PER_DPS
    };
    let gyro = [g(0), g(2), g(4)];
    let h = |i: usize| half_to_f64(u16::from_le_bytes([block[i], block[i + 1]]));
    let (mut x, mut y, mut z) = (h(6), h(8), h(10));
    let mut w = (1.0 - x * x - y * y - z * z).max(0.0).sqrt();
    let n = (w * w + x * x + y * y + z * z).sqrt();
    let n = if n == 0.0 { 1.0 } else { n };
    w /= n;
    x /= n;
    y /= n;
    z /= n;

    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let sp = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0);
    let pitch = sp.asin();
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    let to_deg = 180.0 / std::f64::consts::PI;
    (
        gyro,
        [w, x, y, z],
        [roll * to_deg, pitch * to_deg, yaw * to_deg],
    )
}

// ---------------------------------------------------------------------------
// frame construction
// ---------------------------------------------------------------------------

pub fn feetech_packet(instruction: u8, params: &[u8]) -> Vec<u8> {
    let mut body = vec![ID, params.len() as u8 + 2, instruction];
    body.extend_from_slice(params);
    let mut frame = vec![0xFF, 0xFF];
    frame.extend_from_slice(&body);
    frame.push(!body.iter().fold(0u8, |a, b| a.wrapping_add(*b)));
    frame
}

pub fn feetech_ping() -> Vec<u8> {
    feetech_packet(0x01, &[])
}

pub fn feetech_read(addr: u8, len: u8) -> Vec<u8> {
    feetech_packet(0x02, &[addr, len])
}

pub fn dxl_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for byte in data {
        crc ^= (*byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x8005
            } else {
                crc << 1
            };
        }
    }
    crc
}

pub fn dxl_packet(instruction: u8, params: &[u8]) -> Vec<u8> {
    let mut body = vec![instruction];
    body.extend_from_slice(params);
    let mut stuffed = Vec::with_capacity(body.len() + 4);
    for byte in body {
        stuffed.push(byte);
        let n = stuffed.len();
        if n >= 3 && stuffed[n - 1] == 0xFD && stuffed[n - 2] == 0xFF && stuffed[n - 3] == 0xFF {
            stuffed.push(0xFD);
        }
    }
    let mut frame = Vec::with_capacity(stuffed.len() + 9);
    frame.extend_from_slice(&DXL_HEADER);
    frame.push(ID);
    let len = (stuffed.len() + 2) as u16;
    frame.extend_from_slice(&len.to_le_bytes());
    frame.extend_from_slice(&stuffed);
    let crc = dxl_crc16(&frame);
    frame.extend_from_slice(&crc.to_le_bytes());
    frame
}

pub fn dxl_ping() -> Vec<u8> {
    dxl_packet(0x01, &[])
}

pub fn dxl_read(addr: u16, len: u16) -> Vec<u8> {
    let mut params = Vec::with_capacity(4);
    params.extend_from_slice(&addr.to_le_bytes());
    params.extend_from_slice(&len.to_le_bytes());
    dxl_packet(0x02, &params)
}

// ---------------------------------------------------------------------------
// stream parser
// ---------------------------------------------------------------------------

pub struct Parser {
    buf: Vec<u8>,
    pub status: Option<u8>,
    pub seen: Option<Proto>,
}

enum Step {
    NeedMore,
    Consumed,
    Sample(Sample),
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    pub fn new() -> Self {
        Self {
            buf: Vec::with_capacity(1024),
            status: None,
            seen: None,
        }
    }

    pub fn feed(&mut self, data: &[u8]) -> Vec<Sample> {
        self.buf.extend_from_slice(data);
        let mut out = Vec::new();
        loop {
            match self.parse_one() {
                Step::NeedMore => break,
                Step::Consumed => {}
                Step::Sample(s) => out.push(s),
            }
        }
        out
    }

    fn drop_front(&mut self, n: usize) {
        let n = n.min(self.buf.len());
        self.buf.drain(..n);
    }

    fn parse_one(&mut self) -> Step {
        if self.buf.len() < 2 {
            return Step::NeedMore;
        }
        let start = (0..self.buf.len() - 1)
            .find(|i| self.buf[*i] == 0xFF && self.buf[*i + 1] == 0xFF);
        let Some(start) = start else {
            let keep = usize::from(self.buf.last() == Some(&0xFF));
            let len = self.buf.len();
            self.drop_front(len - keep);
            return Step::NeedMore;
        };
        if start > 0 {
            self.drop_front(start);
        }
        if self.buf.len() < 4 {
            return Step::NeedMore;
        }
        if self.buf[0] == 0xFF && self.buf[1] == 0xFF && self.buf[2] == 0xFD && self.buf[3] == 0x00 {
            self.parse_dxl()
        } else {
            self.parse_feetech()
        }
    }

    fn parse_feetech(&mut self) -> Step {
        let total = self.buf[3] as usize + 4;
        if !(6..=140).contains(&total) {
            self.drop_front(1);
            return Step::Consumed;
        }
        if self.buf.len() < total {
            return Step::NeedMore;
        }
        let sum = self.buf[2..total]
            .iter()
            .fold(0u8, |a, b| a.wrapping_add(*b));
        if sum != 0xFF {
            self.drop_front(1);
            return Step::Consumed;
        }
        if self.buf[2] != ID {
            self.drop_front(total);
            return Step::Consumed;
        }
        self.seen = Some(Proto::Feetech);
        let data_len = total - 6;
        let result = if data_len == FEETECH_POSE_LEN as usize {
            let (gyro, quat, euler) = decode_pose(&self.buf[5..5 + 12]);
            Step::Sample(Sample {
                proto: Proto::Feetech,
                seq: Some(self.buf[5 + 12] as u16),
                status: Some(self.buf[5 + 13]),
                gyro_dps: gyro,
                quat_wxyz: quat,
                euler_deg: euler,
            })
        } else {
            Step::Consumed
        };
        self.drop_front(total);
        result
    }

    fn parse_dxl(&mut self) -> Step {
        if self.buf.len() < 7 {
            return Step::NeedMore;
        }
        let total = 7 + (self.buf[5] as usize | ((self.buf[6] as usize) << 8));
        if !(10..=260).contains(&total) {
            self.drop_front(1);
            return Step::Consumed;
        }
        if self.buf.len() < total {
            return Step::NeedMore;
        }
        let want = u16::from_le_bytes([self.buf[total - 2], self.buf[total - 1]]);
        if dxl_crc16(&self.buf[..total - 2]) != want {
            self.drop_front(1);
            return Step::Consumed;
        }
        if self.buf[4] != ID {
            self.drop_front(total);
            return Step::Consumed;
        }
        self.seen = Some(Proto::Dxl);
        let mut body: Vec<u8> = Vec::with_capacity(total - 9);
        for i in 7..total - 2 {
            let b = self.buf[i];
            let n = body.len();
            if n >= 3 && b == 0xFD && body[n - 1] == 0xFD && body[n - 2] == 0xFF && body[n - 3] == 0xFF {
                continue;
            }
            body.push(b);
        }
        let result = if body.len() >= 2 && body[0] == 0x55 {
            let data = &body[2..];
            if data.len() == DXL_POSE_LEN as usize {
                let (gyro, quat, euler) = decode_pose(data);
                Step::Sample(Sample {
                    proto: Proto::Dxl,
                    seq: None,
                    status: self.status,
                    gyro_dps: gyro,
                    quat_wxyz: quat,
                    euler_deg: euler,
                })
            } else {
                if data.len() == 1 {
                    self.status = Some(data[0]);
                }
                Step::Consumed
            }
        } else {
            Step::Consumed
        };
        self.drop_front(total);
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(data: &[u8]) -> String {
        data.iter()
            .map(|b| format!("{b:02X}"))
            .collect::<Vec<_>>()
            .join(" ")
    }

    #[test]
    fn request_frames_match_the_protocol_doc() {
        assert_eq!(hex(&feetech_ping()), "FF FF C8 02 01 34");
        assert_eq!(hex(&feetech_read(56, 15)), "FF FF C8 04 02 38 0F EA");
        assert_eq!(hex(&dxl_ping()), "FF FF FD 00 C8 03 00 01 3B FA");
        assert_eq!(hex(&dxl_read(124, 12)), "FF FF FD 00 C8 07 00 02 7C 00 0C 00 AB D7");
    }

    #[test]
    fn known_block_decodes() {
        let block = [0xFF, 0xFF, 0xFD, 0x01, 0, 0, 0x00, 0x38, 0, 0, 0, 0];
        let (gyro, quat, euler) = decode_pose(&block);
        assert!((gyro[0] + 0.06097560975609756).abs() < 1e-9);
        assert!((gyro[1] - 31.036585365853657).abs() < 1e-9);
        assert!((quat[0] - 0.8660254037844386).abs() < 1e-6);
        assert!((quat[1] - 0.5).abs() < 1e-6);
        assert!((euler[0] - 60.0).abs() < 1e-6);
    }

    #[test]
    fn a_corrupt_frame_is_skipped_and_the_next_decodes() {
        let mut parser = Parser::new();
        let mut bad = feetech_packet(0x02, &[0; 13]);
        let last = bad.len() - 1;
        bad[last] ^= 0xFF;
        let mut good = vec![0xFF, 0xFF, ID, 17, 0];
        good.extend_from_slice(&[0u8; 15]);
        good.push(0);
        let sum: u8 = good[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        let last = good.len() - 1;
        good[last] = !sum;
        assert!(parser.feed(&bad).is_empty());
        assert_eq!(parser.feed(&good).len(), 1);
    }
}
