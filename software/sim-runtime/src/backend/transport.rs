//! UDS transport glue: framed UnixStream → `mpsc::Sender<Envelope>` /
//! `mpsc::Receiver<Envelope>` pair.
//!
//! `spawn_transport` takes an already-connected (or already-accepted)
//! `UnixStream`, wraps it in length-delimited framing, and splits it
//! into two async tasks:
//!   - outbound: pull Envelopes from `out_rx`, prost-encode, write to
//!     the sink. Drops on encode failure; closes on peer close.
//!   - inbound: pull frames from the stream, prost-decode, forward on
//!     `in_tx`. Drops single malformed frames; closes on peer close.
//!
//! Channel capacity 256 matches the plan spec §6.4 QoS lane numbers.

use crate::ipc::framing::framed_unix_stream;
use crate::proto::Envelope;
use futures::{SinkExt, StreamExt};
use prost::Message;
use tokio::net::UnixStream;
use tokio::sync::mpsc;

pub(crate) fn spawn_transport(
    stream: UnixStream,
) -> (mpsc::Sender<Envelope>, mpsc::Receiver<Envelope>) {
    let framed = framed_unix_stream(stream);
    let (mut sink, mut stream) = framed.split();
    let (out_tx, mut out_rx) = mpsc::channel::<Envelope>(256);
    let (in_tx, in_rx) = mpsc::channel::<Envelope>(256);

    // Outbound: local → wire.
    tokio::spawn(async move {
        while let Some(env) = out_rx.recv().await {
            let mut buf = Vec::with_capacity(env.encoded_len());
            if env.encode(&mut buf).is_err() {
                break;
            }
            if sink.send(buf.into()).await.is_err() {
                break;
            }
        }
    });

    // Inbound: wire → local.
    tokio::spawn(async move {
        while let Some(frame) = stream.next().await {
            let frame = match frame {
                Ok(b) => b,
                Err(_) => break,
            };
            let env = match Envelope::decode(&frame[..]) {
                Ok(e) => e,
                Err(_) => continue, // drop malformed frame, keep stream alive
            };
            if in_tx.send(env).await.is_err() {
                break;
            }
        }
    });

    (out_tx, in_rx)
}
