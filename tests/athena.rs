//! Athena decoding, against traffic recorded from a Muse S Athena (fw 3.1.19,
//! preset p1041) as well as hand-built packets.
//!
//! The fixture is eighty consecutive notifications from the sensor
//! characteristic, one hex string per line. Across them: 275 EEG subpackets
//! (tag 0x12), 142 optical (0x36), 38 IMU (0x47), 10 DRL/REF (0x53) and two
//! battery (0x88) — and two gaps the headset really produced, optical counter
//! 107 → 110 and IMU 30 → 32.

use muse_rs::parse::{parse_athena_notification, AthenaDecoder};
use muse_rs::types::{MuseEvent, Sensor};

fn fixture() -> Vec<Vec<u8>> {
    include_str!("fixtures/athena_fw3.1.19_p1041.hex")
        .lines()
        .map(|line| {
            (0..line.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&line[i..i + 2], 16).unwrap())
                .collect()
        })
        .collect()
}

#[derive(Default, Debug, PartialEq)]
struct Counts {
    eeg: usize,
    ppg: usize,
    accel: usize,
    gyro: usize,
    drl: usize,
    battery: usize,
    gaps: usize,
}

fn count(events: &[MuseEvent]) -> Counts {
    let mut c = Counts::default();
    for e in events {
        match e {
            MuseEvent::Eeg(_) => c.eeg += 1,
            MuseEvent::Ppg(_) => c.ppg += 1,
            MuseEvent::Accelerometer(_) => c.accel += 1,
            MuseEvent::Gyroscope(_) => c.gyro += 1,
            MuseEvent::DrlRef(_) => c.drl += 1,
            MuseEvent::Telemetry(_) => c.battery += 1,
            MuseEvent::Gap(_) => c.gaps += 1,
            _ => {}
        }
    }
    c
}

/// Every subpacket in the recording is found, and nothing else is.
#[test]
fn decodes_every_subpacket_of_a_real_recording() {
    let events: Vec<MuseEvent> = fixture()
        .iter()
        .flat_map(|n| parse_athena_notification(n))
        .collect();
    assert_eq!(
        count(&events),
        Counts {
            eeg: 275 * 8,
            ppg: 142 * 16,
            accel: 38,
            gyro: 38,
            drl: 10,
            battery: 2,
            gaps: 0,
        }
    );
}

/// The headset's own gaps come out as `Gap` events, in samples per channel.
#[test]
fn reports_the_gaps_in_a_real_recording() {
    let mut decoder = AthenaDecoder::new();
    let gaps: Vec<_> = fixture()
        .iter()
        .flat_map(|n| decoder.decode(n))
        .filter_map(|e| match e {
            MuseEvent::Gap(g) => Some((g.sensor, g.missing_packets, g.missing_samples)),
            _ => None,
        })
        .collect();
    // Optical 107 → 110: two 16-channel subpackets of one sample each.
    // IMU 30 → 32: one subpacket of three samples.
    assert_eq!(gaps, vec![(Sensor::Optical, 2, 2), (Sensor::Imu, 1, 3)]);
}

/// Dropping a notification on the host side is seen as a gap too.
#[test]
fn reports_a_dropped_notification() {
    let notifications = fixture();
    let mut decoder = AthenaDecoder::new();
    // Notification 3 opens with EEG; the first ten have no gap of their own.
    assert_eq!(notifications[3][9], 0x12);
    let gaps: Vec<_> = notifications[..10]
        .iter()
        .enumerate()
        .filter(|(i, _)| *i != 3)
        .flat_map(|(_, n)| decoder.decode(n))
        .filter_map(|e| match e {
            MuseEvent::Gap(g) => Some(g.sensor),
            _ => None,
        })
        .collect();
    assert!(gaps.contains(&Sensor::Eeg), "{gaps:?}");
}

/// Two packets in one notification decode as the two did apart.
///
/// The parser used to read on past the first packet's end, taking the
/// second's header for subpackets: any header byte equal to a sensor tag
/// produced a reading made of header bytes, and the second packet's real
/// data was lost.
#[test]
fn splits_notifications_holding_several_packets() {
    let notifications = fixture();
    for pair in notifications.windows(2) {
        let joined = [pair[0].clone(), pair[1].clone()].concat();
        if joined.len() > 512 {
            continue;
        }
        let apart = count(
            &[
                parse_athena_notification(&pair[0]),
                parse_athena_notification(&pair[1]),
            ]
            .concat(),
        );
        assert_eq!(count(&parse_athena_notification(&joined)), apart);
    }
}

/// A header byte that happens to equal a tag is not read as one.
#[test]
fn header_bytes_are_not_taken_for_tags() {
    // Two packets of one IMU subpacket each, the device clock in the second
    // header holding 0x12 — the EEG tag — at every byte.
    let packet = |counter: u8| {
        let mut p = vec![0u8; 14 + 36];
        p[0] = p.len() as u8;
        p[2..6].copy_from_slice(&[0x12; 4]);
        p[9] = 0x47;
        p[10] = counter;
        p
    };
    let joined = [packet(0), packet(1)].concat();
    let c = count(&parse_athena_notification(&joined));
    assert_eq!((c.eeg, c.accel, c.gyro), (0, 2, 2));
}

/// An unknown tag ends its packet; the next packet still decodes.
#[test]
fn unknown_tag_ends_only_its_packet() {
    let notifications = fixture();
    let mut broken = notifications[0].clone();
    assert_eq!(broken[9], 0x12);
    broken[14 + 28] = 0xEE; // the second subpacket's tag
    let joined = [broken, notifications[1].clone()].concat();
    let c = count(&parse_athena_notification(&joined));
    let second = count(&parse_athena_notification(&notifications[1]));
    assert_eq!(c.eeg, 8, "first subpacket of the broken packet only");
    assert_eq!(c.accel, second.accel);
}

/// Truncated and nonsensical input decodes to nothing, without panicking.
#[test]
fn malformed_input_is_dropped() {
    let notifications = fixture();
    let n = &notifications[0];
    for len in 0..n.len() {
        // A truncated packet's declared length exceeds what arrived.
        assert!(parse_athena_notification(&n[..len]).is_empty(), "len {len}");
    }
    assert!(parse_athena_notification(&[0u8; 40]).is_empty());
    assert!(parse_athena_notification(&[5u8; 40]).is_empty());
}

/// Counters land in the events, and DRL/REF carries two steady channels.
#[test]
fn counters_and_drl_ref_values() {
    let events: Vec<MuseEvent> = fixture()
        .iter()
        .flat_map(|n| parse_athena_notification(n))
        .collect();

    let eeg_indices: Vec<u16> = events
        .iter()
        .filter_map(|e| match e {
            MuseEvent::Eeg(r) if r.electrode == 0 => Some(r.index),
            _ => None,
        })
        .collect();
    assert!(eeg_indices
        .windows(2)
        .all(|w| (w[1] as u8).wrapping_sub(w[0] as u8) == 1));

    let drl: Vec<[u16; 2]> = events
        .iter()
        .filter_map(|e| match e {
            MuseEvent::DrlRef(d) => Some(d.samples.clone()),
            _ => None,
        })
        .flatten()
        .collect();
    assert_eq!(drl.len(), 60);
    for [a, b] in drl {
        assert!((8200..8260).contains(&a), "{a}");
        assert!((8250..8300).contains(&b), "{b}");
    }
}
