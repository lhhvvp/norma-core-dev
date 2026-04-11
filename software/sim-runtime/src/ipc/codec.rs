//! Thin wrappers around `prost::Message::encode` / `::decode` for the
//! runtime's `Envelope` type. These are trivial on the happy path but
//! centralise the error translation so callers don't sprinkle
//! `.expect(...)` or `prost::DecodeError` handling throughout.
//!
//! MVP-1 only exercises these through their own roundtrip tests — the
//! real IPC path currently inlines the prost calls. The helpers are
//! kept as the target for MVP-2 when we add IPC error-taxonomy
//! consolidation; remove the `#[allow(dead_code)]` then.

use crate::proto::Envelope;
use prost::Message;

#[allow(dead_code)]
pub(crate) fn encode_envelope(env: &Envelope) -> Vec<u8> {
    let mut buf = Vec::with_capacity(env.encoded_len());
    // prost's encoding cannot fail when the buffer has enough capacity,
    // and we just allocated `encoded_len()` bytes.
    env.encode(&mut buf).expect("Envelope encode cannot fail");
    buf
}

#[allow(dead_code)]
pub(crate) fn decode_envelope(bytes: &[u8]) -> Result<Envelope, prost::DecodeError> {
    Envelope::decode(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{envelope::Payload, Goodbye, Hello};

    #[test]
    fn test_hello_roundtrip() {
        let env = Envelope {
            payload: Some(Payload::Hello(Hello {
                protocol_version: 1,
                client_role: "test".into(),
                client_id: "id".into(),
            })),
        };
        let bytes = encode_envelope(&env);
        let decoded = decode_envelope(&bytes).unwrap();
        match decoded.payload {
            Some(Payload::Hello(h)) => {
                assert_eq!(h.protocol_version, 1);
                assert_eq!(h.client_role, "test");
                assert_eq!(h.client_id, "id");
            }
            other => panic!("expected Hello, got {:?}", other),
        }
    }

    #[test]
    fn test_goodbye_roundtrip() {
        let env = Envelope {
            payload: Some(Payload::Goodbye(Goodbye {
                reason: "bye".into(),
            })),
        };
        let bytes = encode_envelope(&env);
        let decoded = decode_envelope(&bytes).unwrap();
        match decoded.payload {
            Some(Payload::Goodbye(g)) => assert_eq!(g.reason, "bye"),
            other => panic!("expected Goodbye, got {:?}", other),
        }
    }
}
