use std::sync::Arc;

use bytes::{Bytes, BytesMut};
use log::{error, warn};
use prost::Message;
use station_iface::{
    StationEngine, iface_proto::drivers::QueueDataType
};
use normfs::NormFS;

use crate::{
    converters::{self, FourCCFormat},
    usbvideo_proto::{
        frame::{self, FrameFormatKind, FrameStamp, FramesPack},
        usbvideo::{
            Camera, RxEnvelope, RxEnvelopeType
        },
    },
};

pub struct StateTracker<T: StationEngine> {
    normfs: Arc<NormFS>,
    station_engine: Arc<T>,
    config: crate::USBVideoConfig,
    inference_states_queue_id: normfs::QueueId,
}

impl<T: StationEngine> StateTracker<T> {
    pub fn new(
        normfs: Arc<NormFS>,
        station_engine: Arc<T>,
        config: crate::USBVideoConfig,
    ) -> Self {
        let inference_states_queue_id = normfs.resolve("inference-states");
        Self {
            normfs,
            station_engine,
            config,
            inference_states_queue_id,
        }
    }

    pub fn resolve_queue_id(&self, queue_id: &str) -> normfs::QueueId {
        self.normfs.resolve(queue_id)
    }

    pub async fn handle_queue_start(&self, queue_id: &normfs::QueueId) {
        let _ = self.normfs.ensure_queue_exists_for_write(queue_id).await;
        self.station_engine.register_queue(
            queue_id,
            QueueDataType::QdtUsbVideoFrames,
            vec![],
        )
    }

    pub fn send_envelope(&self, queue_id: &normfs::QueueId, envelope: RxEnvelope) -> Result<(), normfs::Error> {
        let mut buf = BytesMut::new();
        envelope.encode(&mut buf).unwrap();
        self.normfs.enqueue(queue_id, buf.freeze())?;
        Ok(())
    }

    pub fn get_last_inference_id_bytes(&self) -> Bytes {
        match self.normfs.get_last_id(&self.inference_states_queue_id) {
            Ok(id) => {
                id.value_to_bytes()
            },
            Err(e) => {
                warn!("Failed to get last inference ID: {}", e);
                Bytes::new()
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn enqueue_frame(&self,
        queue_id: &normfs::QueueId,
        format: FourCCFormat,
        camera: &Camera,
        stamp: FrameStamp,
        width: u32, height: u32,
        frame_data: Bytes,
    ) {
        let converted = converters::convert_frame(
            width as u16,
            height as u16,
            format,
            frame_data,
            self.config.resize_target,
        );

        let converted = match converted {
            Ok(c) => c,
            Err(e) => {
                warn!("Failed to convert frame for camera {}: {}", camera.unique_id, e);
                return;
            }
        };

        let envelope = RxEnvelope {
            r#type: RxEnvelopeType::EtFrames as i32,
            camera: Some(camera.clone()),
            frames: Some(FramesPack {
                format: Some(frame::FrameFormat {
                    width: converted.width,
                    height: converted.height,
                    kind: FrameFormatKind::FfJpeg as i32,
                }),
                linear_data: Bytes::new(),
                frames_data: vec![converted.jpeg.clone()],
                stamps: vec![stamp.clone()],
            }),
            stamp: Some(stamp.clone()),
            formats: vec![],
            last_inference_queue_ptr: self.get_last_inference_id_bytes(),
            error: String::new(),
        };

        let mut buf = BytesMut::new();
        envelope.encode(&mut buf).unwrap();
        if let Err(e) = self.normfs.enqueue(queue_id, buf.freeze()) {
            error!("Failed to enqueue envelope for camera {}: {}", camera.unique_id, e);
        }
    }
}