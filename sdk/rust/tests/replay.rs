//! The shared golden capture must decode to exactly the same lines as the
//! Python and C readers.

use std::fs;

fn testdata(name: &str) -> String {
    format!("{}/../testdata/{}", env!("CARGO_MANIFEST_DIR"), name)
}

#[test]
fn golden_capture_matches() {
    let capture = fs::read(testdata("capture.bin")).expect("capture.bin");
    let expected = fs::read_to_string(testdata("expected.ndjson")).expect("expected.ndjson");
    let want: Vec<&str> = expected.lines().collect();

    let mut parser = imu_dxl::Parser::new();
    let got: Vec<String> = parser
        .feed(&capture)
        .iter()
        .map(|s| s.to_json())
        .collect();

    assert_eq!(got.len(), 4, "expected four samples, got {got:#?}");
    assert_eq!(got, want);
}

#[test]
fn byte_at_a_time_matches_the_whole_file() {
    let capture = fs::read(testdata("capture.bin")).expect("capture.bin");
    let expected = fs::read_to_string(testdata("expected.ndjson")).expect("expected.ndjson");
    let want: Vec<&str> = expected.lines().collect();

    let mut parser = imu_dxl::Parser::new();
    let mut got = Vec::new();
    for byte in &capture {
        got.extend(parser.feed(std::slice::from_ref(byte)).iter().map(|s| s.to_json()));
    }
    assert_eq!(got, want);
}
