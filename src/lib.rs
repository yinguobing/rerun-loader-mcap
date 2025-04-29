mod extractor;
mod parser;

use extractor::Extractor;
use log::{error, info, warn};
use parser::{compressed_image, image, pointcloud, timestamp};
use std::sync::{Arc, atomic::AtomicBool};
use std::{collections::HashMap, fs, io, path::PathBuf};

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Failed to read summary info: {0}")]
    NoSummary(String),
    #[error("Failed to read statistics info: {0}")]
    NoStatistics(String),
    #[error("Invalid topic. {0}")]
    InvalidTopic(String),
    #[error("McapError. {0}")]
    McapError(#[from] mcap::McapError),
    #[error("IO error. {0}")]
    IOError(#[from] io::Error),
    #[error("Interrupted")]
    Interrupted,
    #[error("H.264 error. {0}")]
    H264Error(#[from] compressed_image::Error),
    #[error("Failed to parse message. {0}")]
    ParserError(String),
    #[error("unknown error")]
    Unknown,
}

pub struct Topic {
    pub id: u16,
    pub name: String,
    pub format: String,
    pub description: String,
    pub msg_count: Option<u64>,
}

impl std::fmt::Display for Topic {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}, {}, msgs: {}, {}, {}",
            self.id,
            self.name,
            if self.msg_count.is_some() {
                self.msg_count.unwrap().to_string()
            } else {
                "Unknown".to_owned()
            },
            self.format,
            self.description
        )
    }
}

pub fn summary(files: &Vec<PathBuf>) -> Result<Vec<Topic>, Error> {
    // Collect all topics
    let mut topics: HashMap<u16, Topic> = HashMap::new();

    // Enumerate all files
    for file in files {
        // Read summary
        let fd = fs::File::open(file)?;
        let mmap = unsafe { memmap2::Mmap::map(&fd)? };
        let summary = match mcap::read::Summary::read(&mmap) {
            Ok(summary) => summary.unwrap(),
            Err(e) => {
                warn!("Failed to read summary from {}: {}", file.display(), e);
                continue;
            }
        };

        // Statistics
        let stats = summary
            .stats
            .ok_or(Error::NoStatistics(file.display().to_string()))?;

        // Topics
        for chn in summary.channels {
            topics
                .entry(chn.0)
                .and_modify(|t| {
                    t.id = chn.0;
                    t.name.clone_from(&chn.1.topic);
                    t.format.clone_from(&chn.1.schema.as_ref().unwrap().name);
                    t.description =
                        format!("Encoding: {}", chn.1.schema.as_ref().unwrap().encoding);
                    t.msg_count = if t.msg_count.is_some()
                        && stats.channel_message_counts.contains_key(&chn.0)
                    {
                        Some(
                            t.msg_count.unwrap()
                                + stats.channel_message_counts.get(&chn.0).unwrap(),
                        )
                    } else {
                        None
                    };
                })
                .or_insert(Topic {
                    id: chn.0,
                    name: chn.1.topic.clone(),
                    format: chn.1.schema.as_ref().unwrap().name.clone(),
                    description: format!("Encoding: {}", chn.1.schema.as_ref().unwrap().encoding),
                    msg_count: stats.channel_message_counts.get(&chn.0).copied(),
                });
        }
    }
    let mut topics: Vec<Topic> = topics.into_values().collect();
    topics.sort_by_key(|k| k.id);
    Ok(topics)
}

pub fn process(
    file: &PathBuf,
    sigint: Arc<AtomicBool>,
    vis_stream: rerun::RecordingStream,
    trim_start: i64,
    trim_end: i64,
) -> Result<(), Error> {
    // Create a parser group for all different topics.
    let mut parsers: HashMap<
        &str,
        Box<dyn Extractor<ExtractorError = Box<dyn std::error::Error>>>,
    > = HashMap::new();

    let topics = summary(&vec![file.clone()]).expect("Topic names should be available");
    let output_dir = PathBuf::from(file.file_stem().unwrap());
    let dump_data = false;
    for topic in topics.iter() {
        // Create parser by topic format
        match topic.format.as_str() {
            "builtin_interfaces/msg/Time" => {
                parsers.insert(
                    topic.name.as_str(),
                    Box::new(timestamp::Parser::new(Some(vis_stream.clone()))),
                );
            }
            "sensor_msgs/msg/Image" => {
                parsers.insert(
                    topic.name.as_str(),
                    Box::new(image::Parser::new(
                        &output_dir,
                        Some(vis_stream.clone()),
                        dump_data,
                    )),
                );
            }
            "sensor_msgs/msg/CompressedImage" => {
                parsers.insert(
                    topic.name.as_str(),
                    Box::new(compressed_image::Parser::new(
                        &output_dir,
                        Some(vis_stream.clone()),
                        dump_data,
                    )),
                );
            }
            "sensor_msgs/msg/PointCloud2" => {
                parsers.insert(
                    topic.name.as_str(),
                    Box::new(pointcloud::Parser::new(
                        &output_dir,
                        Some(vis_stream.clone()),
                        dump_data,
                        Some(1.0),
                        Some(1.0),
                    )),
                );
            }
            _ => continue,
        }
    }

    // Read in files
    let fd = fs::File::open(file)?;
    let mmap = unsafe { memmap2::Mmap::map(&fd)? };

    // Enumerate all messages
    for message in mcap::MessageStream::new(&mmap)? {
        // Check for interrupt
        if sigint.load(std::sync::atomic::Ordering::Relaxed) {
            return Err(Error::Interrupted);
        }
        let msg = message?;

        // Trim start/end
        if msg.publish_time < trim_start as u64 {
            continue;
        }
        if msg.publish_time > trim_end as u64 {
            break;
        }

        // Parse message
        let topic_name = msg.channel.topic.as_str();
        let Some(parser) = parsers.get_mut(topic_name) else {
            continue;
        };
        parser
            .step(&msg)
            .map_err(|e| Error::ParserError(e.to_string()))?;
    }

    // Post process
    info!("Post processing...");
    for (_, parser) in parsers.iter_mut() {
        parser
            .post_process(sigint.clone())
            .map_err(|e| Error::ParserError(e.to_string()))?;
    }

    Ok(())
}
