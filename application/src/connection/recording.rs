use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::{Arc, Mutex};

use super::TelemetryData;

/// State of the CSV stream recorder.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RecordingState {
    Stopped,
    Recording,
    Paused,
}

/// Shared CSV recorder for the binary telemetry stream.
///
/// Samples are written into an in-memory buffer (`BufWriter`) and flushed to
/// disk in chunks, so the producer (the binary stream reader thread) never
/// blocks on file I/O and no sample is dropped because of a slow disk.
#[derive(Clone)]
pub struct CsvRecorder {
    inner: Arc<Mutex<RecordingInner>>,
}

struct RecordingInner {
    file: Option<BufWriter<File>>,
    state: RecordingState,
    samples_since_flush: u64,
}

impl CsvRecorder {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(RecordingInner {
                file: None,
                state: RecordingState::Stopped,
                samples_since_flush: 0,
            })),
        }
    }

    /// Starts a new recording: creates the file, writes the header and switches
    /// to [`RecordingState::Recording`].
    pub fn start(&self, path: &str) -> std::io::Result<()> {
        let mut inner = self.inner.lock().unwrap();
        let file = File::create(path)?;
        let mut writer = BufWriter::with_capacity(64 * 1024, file);
        writer.write_all(CSV_HEADER.as_bytes())?;
        writer.flush()?;
        inner.file = Some(writer);
        inner.state = RecordingState::Recording;
        inner.samples_since_flush = 0;
        Ok(())
    }

    pub fn pause(&self) {
        let mut inner = self.inner.lock().unwrap();
        if inner.state == RecordingState::Recording {
            // Flush any buffered samples so they are on disk before pausing.
            if let Some(file) = inner.file.as_mut() {
                let _ = file.flush();
            }
            inner.state = RecordingState::Paused;
            inner.samples_since_flush = 0;
        }
    }

    pub fn resume(&self) {
        let mut inner = self.inner.lock().unwrap();
        if inner.state == RecordingState::Paused {
            inner.state = RecordingState::Recording;
        }
    }

    /// Stops recording, flushing any buffered data and closing the file.
    pub fn stop(&self) {
        let mut inner = self.inner.lock().unwrap();
        if let Some(mut file) = inner.file.take() {
            let _ = file.flush();
        }
        inner.state = RecordingState::Stopped;
        inner.samples_since_flush = 0;
    }

    /// Records one sample. No-op unless actively recording. The write is
    /// buffered and flushed to disk every [`FLUSH_EVERY_SAMPLES`] samples (or
    /// when the buffer fills), so data reaches the file promptly without a
    /// write syscall per sample.
    pub fn write_sample(&self, data: &TelemetryData) {
        let mut inner = self.inner.lock().unwrap();
        if inner.state != RecordingState::Recording {
            return;
        }

        // Write the row (scoped so the file borrow is released before we
        // touch the flush counter).
        {
            let Some(file) = inner.file.as_mut() else {
                return;
            };
            // Best-effort: a formatting/write failure is ignored so the
            // producer thread keeps running and no data is lost.
            let _ = writeln!(
                file,
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                data.time_ms,
                data.power_w,
                data.current_a,
                data.voltage_v,
                data.total_consumption_j,
                data.pwm_input_us,
                data.pwm_output_us,
                data.pwm_control_us,
                data.pwm_ctrl_raw,
                data.status.to_raw(),
                data.flags,
                data.z1_mw,
                data.z2_mws,
                data.z3_mws,
                data.measured_power_w,
                data.target_power_w,
            );
        }

        inner.samples_since_flush += 1;
        if inner.samples_since_flush >= FLUSH_EVERY_SAMPLES {
            {
                let Some(file) = inner.file.as_mut() else {
                    return;
                };
                let _ = file.flush();
            }
            inner.samples_since_flush = 0;
        }
    }
}

/// Flush the in-memory buffer to disk every N samples so the file stays
/// up-to-date while recording (at 1 kHz this is every ~0.1 s).
const FLUSH_EVERY_SAMPLES: u64 = 100;

const CSV_HEADER: &str = "time_ms,max_power_w,current_a,voltage_v,total_consumption_j,pwm_input_us,pwm_output_us,pwm_control_us,pwm_ctrl_raw,status,flags,z1_mw,z2_mws,z3_mws,measured_power_w,target_power_w\n";
