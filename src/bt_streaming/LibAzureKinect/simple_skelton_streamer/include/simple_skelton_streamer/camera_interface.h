#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

#include <k4a/k4a.h>
#include <k4abt.h>
#include <functional>

class CameraInterface
{
public:
    CameraInterface();
    ~CameraInterface();

    bool initialize();
    void startCapture();
    void stopCapture();

    using SkeletonCallback = std::function<void(const k4abt_body_t &body)>;
    void setSkeletonCallback(SkeletonCallback callback);

private:
    k4a_device_t device;
    k4abt_tracker_t tracker;
    SkeletonCallback skeletonCallback;

    void processSkeletons();
};

#endif // CAMERA_INTERFACE_H