#include <iostream>
#include "camera_interface.h"
#include "skeleton_processor.h"
#include <vector>
#include <k4abt.h>
#include <thread>
#include <chrono>

// スケルトンデータを扱いやすい形式に変換するための構造体
struct Joint
{
    int id;
    struct
    {
        float x;
        float y;
        float z;
    } position;
};

struct SkeletonData
{
    std::vector<Joint> joints;
};

// Azure Kinectのスケルトンデータを変換する関数
SkeletonData ConvertBodyToSkeletonData(const k4abt_body_t &body)
{
    SkeletonData skeletonData;

    // K4ABTは全身で32の関節を追跡
    for (int i = 0; i < K4ABT_JOINT_COUNT; i++)
    {
        Joint joint;
        joint.id = i;
        joint.position.x = body.skeleton.joints[i].position.xyz.x;
        joint.position.y = body.skeleton.joints[i].position.xyz.y;
        joint.position.z = body.skeleton.joints[i].position.xyz.z;
        skeletonData.joints.push_back(joint);
    }

    return skeletonData;
}

void SkeletonDetectionCallback(const k4abt_body_t &body)
{
    // Kinectのボディデータを扱いやすい形式に変換
    SkeletonData skeletonData = ConvertBodyToSkeletonData(body);

    // Process the skeleton data received from the camera
    std::cout << "Skeleton detected: " << std::endl;
    for (const auto &joint : skeletonData.joints)
    {
        std::cout << "Joint ID: " << joint.id << " Position: ("
                  << joint.position.x << ", "
                  << joint.position.y << ", "
                  << joint.position.z << ")" << std::endl;
    }
}

int main()
{
    // カメラインターフェースのインスタンスを作成
    CameraInterface camera;

    // カメラの初期化
    if (!camera.initialize())
    {
        std::cerr << "Failed to initialize camera." << std::endl;
        return -1;
    }

    // スケルトン検出のコールバックを設定
    camera.setSkeletonCallback(SkeletonDetectionCallback);

    // キャプチャ開始
    camera.startCapture();

    std::cout << "Camera is running. Press Ctrl+C to exit." << std::endl;

    // メインループ
    // 実際のアプリケーションでは、終了条件やユーザー入力の処理が必要
    try
    {
        while (true)
        {
            // ここに追加のロジックやチェックを追加できます
            // 例えば、キーボード入力をチェックして終了条件を設定するなど
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    // クリーンアップ
    camera.stopCapture();

    return 0;
}