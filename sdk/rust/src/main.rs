//! Read IMU-DXL pose frames and print one JSON line per sample.
//!
//! ```text
//! read_imu --port COM5
//! read_imu --port COM5 --protocol dxl --count 20
//! read_imu --replay ../testdata/capture.bin
//! ```
//!
//! Output is byte-identical to the Python and C readers for the same capture.
//! `--port` needs the `serial` feature (on by default); `--replay` works with
//! `--no-default-features`, which is how the offline tests build.

use std::process::ExitCode;

fn usage() {
    eprintln!(
        "usage: read_imu [--port COM5] [--protocol auto|feetech|dxl] [--count N] [--replay FILE]"
    );
}

fn print_sample(sample: &imu_dxl::Sample, printed: &mut usize, limit: usize) -> bool {
    if limit != 0 && *printed >= limit {
        return false;
    }
    println!("{}", sample.to_json());
    *printed += 1;
    true
}

fn replay(path: &str, limit: usize) -> Result<usize, String> {
    let data = std::fs::read(path).map_err(|e| format!("cannot read {path}: {e}"))?;
    let mut parser = imu_dxl::Parser::new();
    let mut printed = 0;
    for sample in parser.feed(&data) {
        if !print_sample(&sample, &mut printed, limit) {
            break;
        }
    }
    Ok(printed)
}

#[cfg(feature = "serial")]
fn live(port: &str, protocol: &str, limit: usize) -> Result<usize, String> {
    use std::io::{Read, Write};
    use std::time::{Duration, Instant};

    let mut link = serialport::new(port, imu_dxl::BAUD)
        .timeout(Duration::from_millis(50))
        .open()
        .map_err(|e| format!("cannot open {port}: {e}"))?;
    let mut parser = imu_dxl::Parser::new();

    let chosen = match protocol {
        "feetech" => imu_dxl::Proto::Feetech,
        "dxl" => imu_dxl::Proto::Dxl,
        _ => {
            let mut found = None;
            for (proto, ping) in [
                (imu_dxl::Proto::Feetech, imu_dxl::feetech_ping()),
                (imu_dxl::Proto::Dxl, imu_dxl::dxl_ping()),
            ] {
                let _ = link.write_all(&ping);
                let end = Instant::now() + Duration::from_millis(300);
                while Instant::now() < end {
                    let mut buf = [0u8; 256];
                    if let Ok(n) = link.read(&mut buf) {
                        if n > 0 {
                            parser.feed(&buf[..n]);
                            if parser.seen == Some(proto) {
                                found = Some(proto);
                                break;
                            }
                        }
                    }
                }
                if found.is_some() {
                    break;
                }
            }
            found.ok_or_else(|| {
                format!(
                    "no reply from ID {}: check wiring, ground, baud rate and protocol",
                    imu_dxl::ID
                )
            })?
        }
    };
    eprintln!(
        "# {port} @ {} 8N1 · protocol {}",
        imu_dxl::BAUD,
        match chosen {
            imu_dxl::Proto::Feetech => "feetech",
            imu_dxl::Proto::Dxl => "dxl",
        }
    );

    let mut printed = 0;
    let mut last_status: Option<Instant> = None;
    loop {
        let status_due = match last_status {
            None => true,                       // read 143 before the first pose
            Some(at) => at.elapsed() > Duration::from_secs(1),
        };
        if chosen == imu_dxl::Proto::Dxl && status_due {
            last_status = Some(Instant::now());
            let _ = link.write_all(&imu_dxl::dxl_read(
                imu_dxl::DXL_STATUS_ADDR,
                imu_dxl::DXL_STATUS_LEN,
            ));
        }
        let request = match chosen {
            imu_dxl::Proto::Feetech => imu_dxl::feetech_read(
                imu_dxl::FEETECH_POSE_ADDR,
                imu_dxl::FEETECH_POSE_LEN,
            ),
            imu_dxl::Proto::Dxl => {
                imu_dxl::dxl_read(imu_dxl::DXL_POSE_ADDR, imu_dxl::DXL_POSE_LEN)
            }
        };
        let _ = link.write_all(&request);
        let mut buf = [0u8; 256];
        if let Ok(n) = link.read(&mut buf) {
            if n > 0 {
                for sample in parser.feed(&buf[..n]) {
                    if !print_sample(&sample, &mut printed, limit) {
                        return Ok(printed);
                    }
                }
            }
        }
        if limit != 0 && printed >= limit {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    Ok(printed)
}

#[cfg(not(feature = "serial"))]
fn live(_port: &str, _protocol: &str, _limit: usize) -> Result<usize, String> {
    Err("built without the `serial` feature: use --replay, or rebuild with --features serial".into())
}

fn main() -> ExitCode {
    let mut port: Option<String> = None;
    let mut protocol = String::from("auto");
    let mut count = 0usize;
    let mut replay_path: Option<String> = None;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--port" | "-p" => port = args.next(),
            "--protocol" => protocol = args.next().unwrap_or_else(|| "auto".into()),
            "--count" | "-n" => count = args.next().and_then(|v| v.parse().ok()).unwrap_or(0),
            "--replay" => replay_path = args.next(),
            "-h" | "--help" => {
                usage();
                return ExitCode::SUCCESS;
            }
            other => {
                eprintln!("unknown argument: {other}");
                usage();
                return ExitCode::from(2);
            }
        }
    }

    let outcome = if let Some(path) = replay_path {
        replay(&path, count)
    } else if let Some(name) = port {
        live(&name, &protocol, count)
    } else {
        eprintln!("either --port or --replay is required");
        usage();
        return ExitCode::from(2);
    };

    match outcome {
        Ok(0) => {
            eprintln!("no valid frames");
            ExitCode::FAILURE
        }
        Ok(_) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("{message}");
            ExitCode::FAILURE
        }
    }
}
