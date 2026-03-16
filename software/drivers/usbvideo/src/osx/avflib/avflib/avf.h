//
//  avcameras.h
//  avcameras
//
//  Created by Aleksandr Batsuev on 2/10/25.
//

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>

// C-compatible structure for device information
typedef struct {
    char uniqueID[256];
    char modelID[256];
    char localizedName[256];
    char manufacturer[256];
    int32_t position; // 0=unspecified, 1=back, 2=front
    char deviceType[128]; // String representation of device type
    bool hasFlash;
    bool hasTorch;
    bool isConnected;
    bool isSuspended;
} CameraDeviceInfo;

// C-compatible structure for format information
typedef struct {
    uint32_t index;
    int32_t width;
    int32_t height;
    double minFrameRate;
    double maxFrameRate;
    uint32_t pixelFormat;       // The raw pixel format value (e.g., 0x42475241 for 'BGRA')
    char pixelFormatString[5];  // FourCC string representation (e.g., "BGRA")
    bool isHighPhotoQualitySupported;
} CameraFormatInfo;

// Frame data structure
typedef struct {
    int32_t width;              // Frame width
    int32_t height;             // Frame height
    uint32_t pixelFormat;       // Pixel format (same as in CameraFormatInfo)
    char pixelFormatString[5];  // FourCC string
    uint64_t monotonicTimestampNs;      // Frame timestamp in nanoseconds (monotonic clock)
    uint64_t localTimestampNs; // System timestamp in nanoseconds (unix epoch)
    int64_t frameNumber;        // Sequential frame number
    size_t dataSize;            // Size of frame data in bytes
} CameraFrameInfo;

@interface avcameras : NSObject

// Request camera access permission (returns 0 if granted/already granted, -1 if denied, -2 if not determined)
// For async permission request, use requestCameraAccessAsync instead
+ (int32_t)requestCameraAccess;

// Request camera access permission asynchronously (non-blocking)
// The callback will be called with 1 if granted, 0 if denied
+ (void)requestCameraAccessAsync:(void (^)(BOOL granted))callback;

// Get the current camera authorization status
// Returns: 0=not determined, 1=restricted, 2=denied, 3=authorized
+ (int32_t)getCameraAuthorizationStatus;

// Get the number of available video devices
+ (int32_t)getVideoDeviceCount;

// Get device info by index (returns 0 on success, -1 on error)
+ (int32_t)getVideoDeviceInfo:(int32_t)index deviceInfo:(CameraDeviceInfo *)info;

// Get the number of formats for a specific device
+ (int32_t)getFormatCountForDevice:(const char *)deviceID;

// Get format info by index for a specific device (returns 0 on success, -1 on error)
+ (int32_t)getFormatInfo:(const char *)deviceID formatIndex:(int32_t)index formatInfo:(CameraFormatInfo *)info;

// Create a capture session for a device with specific format
// bufferCount: number of frame buffers to allocate (recommended: 3-5)
// Returns session ID (>= 0) on success, -1 on error
+ (int32_t)createCaptureSession:(const char *)deviceID
                           formatIndex:(int32_t)formatIndex
                           bufferCount:(int32_t)bufferCount;

// Start capturing frames
// Returns 0 on success, -1 on error
+ (int32_t)startCapture:(int32_t)sessionId;

// Stop capturing frames
// Returns 0 on success, -1 on error
+ (int32_t)stopCapture:(int32_t)sessionId;

// Check if session is capturing
// Returns 1 if capturing, 0 if not, -1 on error (invalid session)
+ (int32_t)isCapturing:(int32_t)sessionId;

// Get the next available frame (non-blocking)
// Returns 0 if frame available, -1 if no frame available, -2 on error
// frameInfo: will be filled with frame metadata
// buffer: pre-allocated buffer to receive frame data (must be at least bufferSize bytes)
// bufferSize: size of the provided buffer
// actualSize: will be set to actual frame data size
+ (int32_t)getNextFrame:(int32_t)sessionId
              frameInfo:(CameraFrameInfo *)frameInfo
                 buffer:(uint8_t *)buffer
             bufferSize:(size_t)bufferSize
             actualSize:(size_t *)actualSize;

// Get the number of frames currently available in the buffer
// Returns frame count (>= 0) on success, -1 on error
+ (int32_t)getAvailableFrameCount:(int32_t)sessionId;

// Get the number of dropped frames (due to buffer overflow)
// Returns dropped frame count (>= 0) on success, -1 on error
+ (int64_t)getDroppedFrameCount:(int32_t)sessionId;

// Destroy capture session and free resources
// Returns 0 on success, -1 on error
+ (int32_t)destroyCaptureSession:(int32_t)sessionId;

// Get the maximum number of concurrent capture sessions supported
+ (int32_t)getMaxConcurrentSessions;

// Process main run loop briefly to handle notifications
// Call this periodically (e.g. every 100ms) from your main thread
+ (void)processMainRunLoop;

// Helper to get device types for discovery session
+ (NSMutableArray<AVCaptureDeviceType> *)getDeviceTypes;

@end
