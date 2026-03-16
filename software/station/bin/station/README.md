# 🤖 NormaCore.Dev Station

Physical operations platform for robotics - real-time data collection, inference integration, and robot control.

## 🚀 Quick Start

```bash
station --web --tcp
```

Open your browser at **http://localhost:8889** to see the web interface.

## ✨ Features

- 🤖 **Robot & sensor agnostic** - Works with any hardware through extensible drivers
- 💻 **Runs on any computer** - Low resource usage, works on Raspberry Pi out of the box
- 📦 **Zero-dependency** - Single binary without external libraries
- 📱 **Operate & monitor from any device** - Web-based interface accessible from phones, tablets, laptops
- 🌐 **Operate & monitor over any network** - Local or remote access via TCP/WebSocket
- 🕸️ **Sensor mesh** - Build distributed sensor networks using the API
- 🔌 **Plug & play** - Auto-detection and zero-configuration setup
- 🛡️ **Fail-safe by design** - Current limiting, automatic recovery, safe defaults
- 🔐 **Robotic data encryption** - AES-256 encryption, compression, signing with robot key and automatic key rotation
- 📜 **Full lifetime history** - Every sensor reading and command permanently stored
- 🗂️ **Automated dataset assembly** - Ready-to-use datasets for training ML models

## 🗂️ Platform & Feature Support

| Category | Feature | Status |
|----------|---------|--------|
| **Operating Systems** | macOS | ✅ Supported |
|  | Linux | ✅ Supported |
|  | Windows | 📋 Planned |
|  | FreeBSD | 📋 Planned |
| **Devices** | [UVC USB Cameras](../../../../drivers/usbvideo) | ✅ Done |
|  | [SO101](../../../../drivers/st3215) | ✅ Done |
|  | [ElRobot](../../../../drivers/st3215) | ✅ Done |
|  | OpenArm | 🚧 Work in Progress |
|  | Yahboom Dogzilla | 🚧 Work in Progress |
|  | IP Cameras | 🚧 Work in Progress |
|  | Waveshare RoArm-M2 | 📋 Planned |
|  | Yahboom ROSMASTER X3 | 📋 Planned |
| **Client Libraries** | Python | 🔜 Coming Soon |
|  | Golang | 🔜 Coming Soon |
|  | JavaScript | 📋 Planned |
|  | TypeScript | 📋 Planned |

**Want support for your robot?** [Open an issue](https://github.com/norma-core/norma-core/issues) with your device details!

## 📖 Usage

```bash
station --help
```

```bash
NormaCore.Dev station: physical operations platform

Usage: station [OPTIONS]

Options:
      --max-queue-disk-size <MAX_QUEUE_DISK_SIZE>
          Maximum queue disk size in bytes [default: 2147483648]
      --normfs-base-folder <NORMFS_BASE_FOLDER>
          Base folder for normfs storage [default: ./station_data]
  -c, --config <CONFIG>
          Path to configuration file [default: station.yaml]
  -t, --tcp [<TCP>]
          Addr to listen for normfs TCP server. If provided without a value, it will listen on 0.0.0.0:8888
      --web [<WEB>]
          Addr to listen for websocket server. If provided without a value, it will listen on 0.0.0.0:8889
  -h, --help
          Print help
  -V, --version
          Print version
```

### Examples

```bash
# Run with default settings
station

# With web interface
station --web

# With custom config
station --config my-robot.yaml

# Full example
station \
  --config robot.yaml \
  --normfs-base-folder ./data \
  --max-queue-disk-size 5368709120 \
  --tcp 0.0.0.0:8888 \
  --web 0.0.0.0:8889
```

## 📝 Configuration

Station uses YAML configuration. On first run, a default `station.yaml` is created:

```yaml
drivers:
  # ST3215 servo bus
  st3215:
    enabled: true
    current-threshold: 100     # Current limit for safety (mA)
    deadband: 20               # Minimum movement threshold
    motor-current-thresholds:  # Per-motor overrides
      8: 40
      5: 60

  # System resource monitoring
  system-info: true

  # USB video capture
  usb-video:
    enabled: true
    resize-target: 224  # Resize shortest dimension to 224px

# ML inference integration
inference:
  - queue-id: "inference/normvla"
    shm: "/tmp/normvla"
    shm-size-mb: 12
    format: "normvla"
    st3215-bus: "auto"  # Auto-detect or specify bus ID
    update-interval: "100ms"

# Optional: S3-compatible cloud offload
cloud-offload:
  bucket: "my-robot-data"  # leave empty to use env: AWS_S3_BUCKET
  region: "us-east-1"  # leave empty to use env: AWS_REGION
  access_key_id: "YOUR_KEY"  # leave empty to use env: AWS_ACCESS_KEY_ID
  secret_access_key: "YOUR_SECRET"  # leave empty to use env: AWS_SECRET_ACCESS_KEY
  endpoint: "https://s3.amazonaws.com"  # Optional for MinIO/R2, leave empty to use env: AWS_ENDPOINT_URL
```

## 📊 Data Storage

All data is stored in NormFS queues under `station-data/{unique-id}`.
Store data is:
- 🔒 **Encrypted** - AES-256 encryption
- 🗜️ **Compressed** - LZ4 compression
- ☁️ **Cloud-synced** - Optional automatic S3 upload (encrypted data only)

## 🌐 Web Interface

Access the web interface at `http://localhost:8889` (when `--web` is enabled):

- Real-time robot state visualization
- 3D URDF rendering
- Servo calibration tools
- Video feed monitoring
- Timeline navigation

See [station-viewer](../../clients/station-viewer) for details.

## 🔧 Building

```bash
# Build web client first
cd software/station/clients/station-viewer
yarn install
yarn build
cd -

# Build release binary
cargo build --release -p station

# Binary location
./target/release/station

# Cross-compile for Linux ARM64 (e.g., Raspberry Pi)
cargo zigbuild --target aarch64-unknown-linux-gnu --release -p station
```

## 📖 License

MIT - See [LICENSE](../../LICENSE)
