#include <iostream>
#include "camera_interface.h"
#include "skeleton_processor.h"

void SkeletonDetectionCallback(const SkeletonData& skeletonData) {
    // Process the skeleton data received from the camera
    std::cout << "Skeleton detected: " << std::endl;
    for (const auto& joint : skeletonData.joints) {
        std::cout << "Joint ID: " << joint.id << " Position: (" 
                  << joint.position.x << ", " 
                  << joint.position.y << ", " 
                  << joint.position.z << ")" << std::endl;
    }
}

int main() {
    // Initialize camera
    if (!InitializeCamera()) {
        std::cerr << "Failed to initialize camera." << std::endl;
        return -1;
    }

    // Set the callback for skeleton detection
    SetSkeletonDetectionCallback(SkeletonDetectionCallback);

    // Start capturing frames and processing skeletons
    StartCapture();

    // Main loop
    while (true) {
        // Here you can add any additional logic or checks
        // For example, you could check for a termination condition
    }

    // Cleanup
    StopCapture();
    ShutdownCamera();

    return 0;
}