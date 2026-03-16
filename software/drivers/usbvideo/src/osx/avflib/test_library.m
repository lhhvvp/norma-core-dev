#import <Foundation/Foundation.h>
#import "avflib/avf.h"

void printDevices() {
    int32_t count = [avcameras getVideoDeviceCount];
    NSLog(@"=== Device count: %d ===", count);

    if (count > 0) {
        for (int32_t i = 0; i < count; i++) {
            CameraDeviceInfo info;
            int32_t result = [avcameras getVideoDeviceInfo:i deviceInfo:&info];
            if (result == 0) {
                NSLog(@"  [%d] %s", i, info.uniqueID);
                NSLog(@"      Model: %s", info.modelID);
                NSLog(@"      Name: %s", info.localizedName);
            } else {
                NSLog(@"  [%d] Failed to get info: %d", i, result);
            }
        }
    }
}

int main(int argc, const char * argv[]) {
    @autoreleasepool {
        NSLog(@"=== Testing avflib Library ===\n");

        // Request camera access
        NSLog(@"Requesting camera access...");
        int32_t accessResult = [avcameras requestCameraAccess];
        NSLog(@"Camera access result: %d\n", accessResult);

        if (accessResult != 0) {
            NSLog(@"ERROR: Camera access denied!");
            return 1;
        }

        // Give the AVFoundation thread time to start
        NSLog(@"Waiting for AVFoundation thread to initialize...");
        sleep(1);

        // Print initial devices
        NSLog(@"\nInitial device list:");
        printDevices();

        NSLog(@"\n=== Monitoring for device changes ===");
        NSLog(@"Plug or unplug a USB camera...");
        NSLog(@"Press Ctrl+C to exit\n");

        // Test periodic run loop processing (like Rust will do)
        NSLog(@"Using periodic processMainRunLoop ticks...");
        for (int i = 0; i < 120; i++) {  // Run for 2 minutes
            // Tick the main run loop to process notifications
            [avcameras processMainRunLoop];

            // Sleep a bit
            usleep(100000);  // 100ms

            // Print devices every 2 seconds
            if (i % 20 == 0) {
                NSLog(@"\n--- Poll %d ---", i / 20 + 1);
                printDevices();
            }
        }
    }
    return 0;
}
