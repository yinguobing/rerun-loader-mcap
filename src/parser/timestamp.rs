use crate::extractor::Extractor;
use mcap::Message;
use rerun::RecordingStream;
use ros2_interfaces_humble::builtin_interfaces::msg::Time;

const ZSTD_MAGIC_NUMBER: [u8; 4] = [0x28, 0xb5, 0x2f, 0xfd];

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("ZSTD error. {0}")]
    Zstd(#[from] std::io::Error),
    #[error("CDR error. {0}")]
    Cdr(#[from] cdr::Error),
}

pub struct Parser {
    // Visualizer with rerun
    rec_stream: RecordingStream,

    // Entity prefix
    entity_path_prefix: rerun::EntityPath,
}

impl Parser {
    pub fn new(rerun_stream: RecordingStream, entity_path_prefix: &rerun::EntityPath) -> Self {
        Parser {
            rec_stream: rerun_stream,
            entity_path_prefix: entity_path_prefix.to_owned(),
        }
    }
}

impl Extractor for Parser {
    type ExtractorError = Box<dyn std::error::Error>;

    fn step(&mut self, message: &Message) -> Result<(), Self::ExtractorError> {
        let buf = message.data.as_ref();
        let serialized = if message.data[..4] == ZSTD_MAGIC_NUMBER {
            zstd::stream::decode_all(buf).map_err(Error::Zstd)?
        } else {
            message.data.to_vec()
        };
        let stamp = cdr::deserialize_from::<_, Time, _>(serialized.as_slice(), cdr::size::Infinite)
            .map_err(Error::Cdr)?;

        self.rec_stream
            .set_timestamp_secs_since_epoch("main", stamp.sec as f64 + stamp.nanosec as f64 * 1e-9);

        self.rec_stream
            .log(
                self.entity_path_prefix
                    .join(&rerun::EntityPath::from_single_string(
                        message.channel.topic.clone(),
                    )),
                &rerun::Scalars::new([stamp.sec as f64 + stamp.nanosec as f64 * 1e-9]),
            )
            .unwrap();

        Ok(())
    }
}
