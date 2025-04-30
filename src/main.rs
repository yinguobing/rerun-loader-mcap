//! MCAP data-loader plugin for the Rerun Viewer.

use argh;
use log::warn;
use rerun::EXTERNAL_DATA_LOADER_INCOMPATIBLE_EXIT_CODE;
use rerun_loader_mcap::process;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

/// [`rerun-loader-`]: `rerun::EXTERNAL_DATA_LOADER_PREFIX`
#[derive(argh::FromArgs, Debug)]
struct Args {
    #[argh(positional)]
    filepath: std::path::PathBuf,

    /// the recommended ApplicationId to log the data to
    #[argh(option)]
    application_id: Option<String>,

    /// the ApplicationId that is currently opened in the viewer, if any.
    #[argh(option)]
    opened_application_id: Option<String>,

    /// optional recommended ID for the recording
    #[argh(option)]
    recording_id: Option<String>,

    /// optional recommended ID for the recording
    #[argh(option)]
    opened_recording_id: Option<String>,

    /// optional prefix for all entity paths
    #[argh(option)]
    entity_path_prefix: Option<String>,

    /// optionally mark data to be logged statically
    #[argh(arg_name = "static", switch)]
    static_: bool,

    /// optional sequences to log at (e.g. `--time_sequence sim_frame=42`) (repeatable)
    #[argh(option)]
    time_sequence: Vec<String>,

    /// optional duration(s) (in nanoseconds) to log at (e.g. `--time_duration_nanos sim_time=123`) (repeatable)
    #[argh(option)]
    time_duration_nanos: Vec<String>,

    /// optional timestamp(s) (in nanoseconds since epochj) to log at (e.g. `--time_timestamp_nanos sim_time=1709203426123456789`) (repeatable)
    #[argh(option)]
    time_timestamp_nanos: Vec<String>,
}

fn extension(path: &std::path::Path) -> String {
    path.extension()
        .unwrap_or_default()
        .to_ascii_lowercase()
        .to_string_lossy()
        .to_string()
}

fn main() -> anyhow::Result<()> {
    let args: Args = argh::from_env();

    let is_file = args.filepath.is_file();
    let is_mcap_file = extension(&args.filepath) == "mcap";

    // Inform the Rerun Viewer that we do not support that kind of file.
    if !is_file || !is_mcap_file {
        #[allow(clippy::exit)]
        std::process::exit(EXTERNAL_DATA_LOADER_INCOMPATIBLE_EXIT_CODE);
    }

    // Rerun stream
    let rec = {
        let app_id = args.application_id.or(args.opened_application_id).unwrap();
        let mut rec = rerun::RecordingStreamBuilder::new(app_id);
        let rec_id = args.recording_id.or(args.opened_recording_id).unwrap();
        rec = rec.recording_id(rec_id);

        // The most important part of this: log to standard output so the Rerun Viewer can ingest it!
        rec.stdout()?
    };

    // Catch SIGINT
    let sigint = Arc::new(AtomicBool::new(false));
    let handler_sigint = sigint.clone();
    ctrlc::set_handler(move || {
        warn!("Ctrl-C received");
        handler_sigint.store(true, std::sync::atomic::Ordering::Relaxed);
    })
    .expect("Error setting Ctrl-C handler");

    // Process the file
    let _ = process(&args.filepath, sigint, rec, i64::MIN, i64::MAX)?;

    Ok::<_, anyhow::Error>(())
}

fn timepoint_from_args(args: &Args) -> anyhow::Result<rerun::TimePoint> {
    let mut timepoint = rerun::TimePoint::default();

    for seq_str in &args.time_sequence {
        let Some((seqline_name, seq)) = seq_str.split_once('=') else {
            continue;
        };
        timepoint.insert_cell(
            seqline_name,
            rerun::TimeCell::from_sequence(seq.parse::<i64>()?),
        );
    }

    for duration_nanos_str in &args.time_duration_nanos {
        let Some((seqline_name, duration_nd)) = duration_nanos_str.split_once('=') else {
            continue;
        };
        timepoint.insert_cell(
            seqline_name,
            rerun::TimeCell::from_duration_nanos(duration_nd.parse::<i64>()?),
        );
    }

    for timestamp_nanos_str in &args.time_timestamp_nanos {
        let Some((seqline_name, timestamp_nd)) = timestamp_nanos_str.split_once('=') else {
            continue;
        };
        timepoint.insert_cell(
            seqline_name,
            rerun::TimeCell::from_timestamp_nanos_since_epoch(timestamp_nd.parse::<i64>()?),
        );
    }

    Ok(timepoint)
}
