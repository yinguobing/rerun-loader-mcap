use crate::extractor::Extractor;
use log::error;
use mcap::Message;
use rerun::RecordingStream;
use ros2_interfaces_humble::sensor_msgs::msg::CompressedImage;

const ZSTD_MAGIC_NUMBER: [u8; 4] = [0x28, 0xb5, 0x2f, 0xfd];

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Interrupted.")]
    Interrupted,
    #[error("Init image from buf failed.")]
    ImageBuf,
    #[error("ZSTD error. {0}")]
    Zstd(#[from] std::io::Error),
    #[error("CDR error. {0}")]
    Cdr(#[from] cdr::Error),
    #[error("Image error. {0}")]
    Image(#[from] image::ImageError),
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
        let deserialized = cdr::deserialize_from::<_, CompressedImage, _>(
            serialized.as_slice(),
            cdr::size::Infinite,
        )
        .map_err(Error::Cdr)?;

        self.rec_stream.set_timestamp_secs_since_epoch(
            "main",
            deserialized.header.stamp.sec as f64 + deserialized.header.stamp.nanosec as f64 * 1e-9,
        );
        self.rec_stream.log(
            self.entity_path_prefix
                .join(&rerun::EntityPath::from_single_string(
                    message.channel.topic.clone(),
                )),
            &rerun::EncodedImage::from_file_contents(deserialized.data),
        )?;

        Ok(())
    }
}
