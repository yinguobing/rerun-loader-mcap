//! MCAP data-loader plugin for the Rerun Viewer.
use rerun::EXTERNAL_DATA_LOADER_INCOMPATIBLE_EXIT_CODE;
use rerun_loader_mcap::process;

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
    #[allow(dead_code)]
    static_: bool,

    /// optional sequences to log at (e.g. `--time_sequence sim_frame=42`) (repeatable)
    #[argh(option)]
    #[allow(dead_code)]
    time_sequence: Vec<String>,

    /// optional duration(s) (in nanoseconds) to log at (e.g. `--time_duration_nanos sim_time=123`) (repeatable)
    #[argh(option)]
    #[allow(dead_code)]
    time_duration_nanos: Vec<String>,

    /// optional timestamp(s) (in nanoseconds since epochj) to log at (e.g. `--time_timestamp_nanos sim_time=1709203426123456789`) (repeatable)
    #[argh(option)]
    #[allow(dead_code)]
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

    let entity_path_prefix = args
        .entity_path_prefix
        .map_or_else(|| rerun::EntityPath::new(vec![]), rerun::EntityPath::from);

    // Process the file
    process(&args.filepath, &entity_path_prefix, rec)?;

    Ok::<_, anyhow::Error>(())
}
