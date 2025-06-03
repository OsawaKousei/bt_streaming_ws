#include "camera_interface.h"
#include <k4a/k4a.h>
#include <iostream>

class CameraInterface {
public:
    CameraInterface() : device(nullptr) {}

    bool initialize() {
        if (k4a_device_open(0, &device) != K4A_RESULT_SUCCEEDED) {
            std::cerr << "Failed to open K4A device!" << std::endl;
            return false;
        }

        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

        if (k4a_device_start_cameras(device, &config) != K4A_RESULT_SUCCEEDED) {
            std::cerr << "Failed to start cameras!" << std::endl;
            k4a_device_close(device);
            return false;
        }

        return true;
    }

    void getSkeletonData(void (*callback)(const k4abt_body_t&)) {
        k4a_capture_t capture = nullptr;
        k4a_wait_result_t result = k4a_device_get_capture(device, &capture, 0);

        if (result == K4A_WAIT_RESULT_SUCCEEDED) {
            k4abt_frame_t bodyFrame = nullptr;
            // Assume skeleton processing is done here and bodyFrame is filled
            // Call the callback function with the detected skeleton data
            for (uint32_t i = 0; i < k4abt_frame_get_num_bodies(bodyFrame); i++) {
                k4abt_body_t body;
                k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton);
                callback(body);
            }
            k4a_capture_release(capture);
        }
    }

    ~CameraInterface() {
        if (device) {
            k4a_device_stop_cameras(device);
            k4a_device_close(device);
        }
    }

private:
    k4a_device_t device;
};