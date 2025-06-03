#include "skeleton_processor.h"
#include <k4a/k4a.h>
#include <k4abt.h>
#include <iostream>
#include <vector>
#include <functional>

class SkeletonProcessor {
public:
    SkeletonProcessor() : tracker(nullptr) {
        // Initialize the body tracker
        k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
        trackerConfig.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA; // or CPU based on your needs
        if (k4abt_tracker_create(&sensorCalibration, trackerConfig, &tracker) != K4A_RESULT_SUCCEEDED) {
            std::cerr << "Failed to create body tracker!" << std::endl;
        }
    }

    ~SkeletonProcessor() {
        if (tracker) {
            k4abt_tracker_shutdown(tracker);
            k4abt_tracker_destroy(tracker);
        }
    }

    void ProcessSkeletons(k4a_capture_t capture, std::function<void(const std::vector<k4abt_body_t>&)> callback) {
        k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);
        if (queueCaptureResult == K4A_WAIT_RESULT_FAILED) {
            std::cerr << "Failed to enqueue capture!" << std::endl;
            return;
        }

        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, K4A_WAIT_INFINITE);
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED) {
            uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
            std::vector<k4abt_body_t> bodies(numBodies);
            for (uint32_t i = 0; i < numBodies; i++) {
                k4abt_frame_get_body_skeleton(bodyFrame, i, &bodies[i].skeleton);
                bodies[i].id = k4abt_frame_get_body_id(bodyFrame, i);
            }
            callback(bodies);
            k4abt_frame_release(bodyFrame);
        } else {
            std::cerr << "Failed to pop body frame!" << std::endl;
        }
    }

private:
    k4abt_tracker_t tracker;
    k4a_calibration_t sensorCalibration; // This should be initialized properly in your camera interface
};