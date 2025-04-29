use crate::extractor::Extractor;
use colorgrad::Gradient;
use mcap::Message;
use rerun::RecordingStream;
use ros2_interfaces_humble::sensor_msgs::msg::{PointCloud2, PointField};
use std::{
    collections::HashMap,
    fs,
    path::Path,
    sync::{Arc, atomic::AtomicBool},
};

const ZSTD_MAGIC_NUMBER: [u8; 4] = [0x28, 0xb5, 0x2f, 0xfd];

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("ZSTD error. {0}")]
    Zstd(#[from] std::io::Error),
    #[error("CDR error. {0}")]
    CDR(#[from] cdr::Error),
}

pub struct Parser {
    // Visualizer with rerun
    rec_stream: Option<RecordingStream>,

    // Scale the points in spatial domain? This could be usefull if users want to visualize the pointcloud in a
    // different spatial scale.
    spatial_scale: f32,

    // Intensity scale. This is used to scale the intensity values to a range [0, 1].
    intensity_scale: f32,

    // Color map. Map point cloud intensity to a color.
    color_map: colorgrad::LinearGradient,
}

impl Parser {
    pub fn new(
        output_path: &Path,
        rerun_stream: Option<RecordingStream>,
        dump_data: bool,
        spatial_scale: Option<f32>,
        intensity_scale: Option<f32>,
    ) -> Self {
        // Create output dir
        if dump_data {
            fs::create_dir_all(output_path).unwrap();
        }

        Parser {
            rec_stream: rerun_stream,
            spatial_scale: spatial_scale.unwrap_or(1.0),
            intensity_scale: intensity_scale.unwrap_or(1.0),
            color_map: colorgrad::GradientBuilder::new()
                .html_colors(&["#00F", "#FFF", "gold"])
                .domain(&[0.0, 0.3, 0.6])
                .mode(colorgrad::BlendMode::LinearRgb)
                .build::<colorgrad::LinearGradient>()
                .expect("Color map should be created"),
        }
    }

    fn decode(&self, message: &PointCloud2) -> HashMap<String, Vec<f64>> {
        let mut decoded: HashMap<String, Vec<f64>> = HashMap::new();
        for field in message.fields.iter() {
            let field_name = field.name.as_str();
            let field_size = match field.datatype {
                PointField::INT8 => std::mem::size_of::<i8>(),
                PointField::UINT8 => std::mem::size_of::<u8>(),
                PointField::INT16 => std::mem::size_of::<i16>(),
                PointField::UINT16 => std::mem::size_of::<u16>(),
                PointField::INT32 => std::mem::size_of::<i32>(),
                PointField::UINT32 => std::mem::size_of::<u32>(),
                PointField::FLOAT32 => std::mem::size_of::<f32>(),
                PointField::FLOAT64 => std::mem::size_of::<f64>(),
                0_u8 | 9_u8..=u8::MAX => panic!("Can not get data size, invalid datatype."),
            };
            let decode_fun: fn(&[u8]) -> f64 = match field.datatype {
                PointField::INT8 => |x| f64::from(i8::from_ne_bytes(x.try_into().unwrap())),
                PointField::UINT8 => |x| f64::from(u8::from_ne_bytes(x.try_into().unwrap())),
                PointField::INT16 => |x| f64::from(i16::from_ne_bytes(x.try_into().unwrap())),
                PointField::UINT16 => |x| f64::from(u16::from_ne_bytes(x.try_into().unwrap())),
                PointField::INT32 => |x| f64::from(i32::from_ne_bytes(x.try_into().unwrap())),
                PointField::UINT32 => |x| f64::from(u32::from_ne_bytes(x.try_into().unwrap())),
                PointField::FLOAT32 => |x| f64::from(f32::from_ne_bytes(x.try_into().unwrap())),
                PointField::FLOAT64 => |x| f64::from(f64::from_ne_bytes(x.try_into().unwrap())),
                0_u8 | 9_u8..=u8::MAX => panic!("Can not match decode function, invalid datatype."),
            };
            let mut values: Vec<f64> =
                Vec::with_capacity((message.height * message.width) as usize);
            for idx in 0..message.row_step {
                let idx_start = message.point_step * idx + field.offset;
                let idx_end = idx_start + field.count * field_size as u32;
                let buf = &message.data[idx_start as usize..idx_end as usize];
                values.push(decode_fun(buf));
            }
            decoded.insert(field_name.to_string(), values);
        }
        return decoded;
    }
}

impl Extractor for Parser {
    type ExtractorError = Box<dyn std::error::Error>;

    fn step(&mut self, message: &Message) -> Result<(), Self::ExtractorError> {
        let buf = message.data.as_ref();
        let serialized = if &message.data[..4] == ZSTD_MAGIC_NUMBER {
            zstd::stream::decode_all(buf).map_err(|e| Error::Zstd(e))?
        } else {
            message.data.to_vec()
        };
        let cloud_msg =
            cdr::deserialize_from::<_, PointCloud2, _>(serialized.as_slice(), cdr::size::Infinite)
                .map_err(|e| Error::CDR(e))?;

        // Extract points and intensity
        let decoded = self.decode(&cloud_msg);
        let xyz = decoded["x"]
            .iter()
            .zip(decoded["y"].iter())
            .zip(decoded["z"].iter())
            .map(|((x, y), z)| [*x as f32, *y as f32, *z as f32]);
        let intensity = decoded["intensity"].iter().map(|p| *p as f32);

        // Visualize?
        if let Some(rec) = &self.rec_stream {
            let colors = intensity.map(|i| {
                let [r, g, b, a] = self.color_map.at(i).to_rgba8();
                rerun::Color::from_unmultiplied_rgba(r, g, b, a)
            });
            rec.set_timestamp_secs_since_epoch(
                "main",
                cloud_msg.header.stamp.sec as f64 + cloud_msg.header.stamp.nanosec as f64 * 1e-9,
            );
            rec.log(
                message.channel.topic.clone(),
                &rerun::Points3D::new(xyz)
                    .with_colors(colors)
                    .with_radii([0.01]),
            )?;
        }

        Ok(())
    }

    fn post_process(&mut self, _sigint: Arc<AtomicBool>) -> Result<(), Self::ExtractorError> {
        Ok(())
    }
}
