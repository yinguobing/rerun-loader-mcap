use crate::extractor::Extractor;
use mcap::Message;
use rerun::RecordingStream;
use ros2_interfaces_humble::sensor_msgs::msg::Image;

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
        let image_msg =
            cdr::deserialize_from::<_, Image, _>(serialized.as_slice(), cdr::size::Infinite)
                .map_err(Error::Cdr)?;

        if image_msg.encoding != "nv12" {
            return Ok(());
        }

        let height_rgb = (image_msg.height as f32 / 1.5) as usize;
        let mut rgb = vec![0u8; height_rgb * image_msg.width as usize * 3];
        let _ret = unsafe {
            libyuv::nv12_to_raw(
                image_msg.data.as_ptr(),
                image_msg.width as i32,
                image_msg
                    .data
                    .as_ptr()
                    .add(image_msg.width as usize * height_rgb),
                image_msg.width as i32,
                rgb.as_mut_ptr(),
                image_msg.width as i32 * 3,
                image_msg.width as i32,
                height_rgb as i32,
            )
        };

        self.rec_stream.set_timestamp_secs_since_epoch(
            "main",
            image_msg.header.stamp.sec as f64 + image_msg.header.stamp.nanosec as f64 * 1e-9,
        );
        self.rec_stream.log(
            self.entity_path_prefix
                .join(&rerun::EntityPath::from_single_string(
                    message.channel.topic.clone(),
                )),
            &rerun::Image::from_elements(
                rgb.as_ref(),
                [image_msg.width, height_rgb as u32],
                rerun::ColorModel::RGB,
            ),
        )?;

        Ok(())
    }
}
