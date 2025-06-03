#ifndef SKELETON_PROCESSOR_H
#define SKELETON_PROCESSOR_H

#include <vector>
#include <functional>
#include <k4abt.h>

class SkeletonProcessor
{
public:
    using SkeletonCallback = std::function<void(const std::vector<k4abt_body_t> &)>;

    SkeletonProcessor();
    ~SkeletonProcessor();

    bool initialize();
    void setSkeletonCallback(SkeletonCallback callback);
    void processSkeletons(k4abt_frame_t bodyFrame);

private:
    SkeletonCallback m_callback;
};

#endif // SKELETON_PROCESSOR_H