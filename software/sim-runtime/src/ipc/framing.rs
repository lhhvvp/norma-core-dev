//! Length-delimited framing for the UDS transport.
//!
//! Every frame is `u32_be length | payload`. Max frame length is
//! 16 MiB — large enough for a full snapshot of an 8-joint arm plus
//! a full-resolution camera frame, small enough that a malformed
//! peer cannot exhaust memory.

use tokio::net::UnixStream;
use tokio_util::codec::{Framed, LengthDelimitedCodec};

pub(crate) fn framed_unix_stream(
    stream: UnixStream,
) -> Framed<UnixStream, LengthDelimitedCodec> {
    let codec = LengthDelimitedCodec::builder()
        .length_field_type::<u32>()
        .big_endian()
        .max_frame_length(16 * 1024 * 1024)
        .new_codec();
    Framed::new(stream, codec)
}
