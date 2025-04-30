use mcap::Message;

pub trait Extractor {
    type ExtractorError;

    /// Function to be called for every message.
    fn step(&mut self, message: &Message) -> Result<(), Self::ExtractorError>;
}
