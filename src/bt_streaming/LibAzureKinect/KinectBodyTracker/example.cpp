#include "KinectBodyTracker.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>

// 骨格情報を表示する関数
void printBodyInfo(const kinect_tracker::BodyInfo &body)
{
  std::cout << "Body ID: " << body.id << std::endl;

  // 主要なジョイントのみ表示
  const int displayJoints[] = {
      K4ABT_JOINT_PELVIS,
      K4ABT_JOINT_SPINE_CHEST,
      K4ABT_JOINT_HEAD,
      K4ABT_JOINT_HAND_LEFT,
      K4ABT_JOINT_HAND_RIGHT,
      K4ABT_JOINT_FOOT_LEFT,
      K4ABT_JOINT_FOOT_RIGHT};

  for (int jointId : displayJoints)
  {
    const auto &joint = body.joints[jointId];
    std::cout << "  Joint " << jointId << ": ";
    std::cout << "Position (" << joint.position[0] << ", " << joint.position[1] << ", " << joint.position[2] << ") ";
    std::cout << "Confidence: " << joint.confidenceLevel << std::endl;
  }
}

// コールバック関数
void bodyTrackingCallback(const kinect_tracker::BodyTrackingResult &result)
{
  std::cout << "Timestamp: " << result.timestamp << " Number of bodies: " << result.bodies.size() << std::endl;

  for (const auto &body : result.bodies)
  {
    printBodyInfo(body);
  }
  std::cout << "------------------------" << std::endl;
}

int main()
{
  // トラッカーのインスタンス作成
  kinect_tracker::KinectBodyTracker tracker;

  // 設定
  kinect_tracker::KinectBodyTracker::DeviceConfig config;
  config.depthMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
  config.colorResolution = K4A_COLOR_RESOLUTION_720P;
  config.fps = K4A_FRAMES_PER_SECOND_30;

  // 初期化
  if (!tracker.initialize(config))
  {
    std::cerr << "Failed to initialize KinectBodyTracker" << std::endl;
    return -1;
  }

  std::cout << "KinectBodyTracker initialized successfully" << std::endl;

  // 同期モードの例
  std::cout << "Synchronous mode example:" << std::endl;
  for (int i = 0; i < 5; i++)
  {
    kinect_tracker::BodyTrackingResult result;
    if (tracker.getBodyTrackingResult(result))
    {
      std::cout << "Frame " << i << ", Number of bodies: " << result.bodies.size() << std::endl;
      for (const auto &body : result.bodies)
      {
        printBodyInfo(body);
      }
    }
    else
    {
      std::cerr << "Failed to get body tracking result" << std::endl;
    }
    std::cout << "------------------------" << std::endl;
  }

  // 非同期モードの例
  std::cout << "Asynchronous mode example (2Hz output):" << std::endl;

  // コールバック関数をセットする前にフラグを初期化
  bool connectionLost = false;

  // カメラの接続が切れたら処理を停止するためのコールバック関数
  tracker.setBodyTrackingCallback([&connectionLost](const kinect_tracker::BodyTrackingResult &result)
                                  {
    static auto lastPrintTime = std::chrono::steady_clock::now();
    auto currentTime = std::chrono::steady_clock::now();

    // 2Hz (0.5秒ごと) に制限して出力
    if (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastPrintTime).count() >= 500)
    {
      std::cout << "Timestamp: " << result.timestamp << " Number of bodies: " << result.bodies.size() << std::endl;

      for (const auto &body : result.bodies)
      {
        printBodyInfo(body);
      }
      std::cout << "------------------------" << std::endl;

      // 時間を更新
      lastPrintTime = currentTime;
    } });

  if (tracker.startTracking())
  {
    std::cout << "Tracking started. Press Enter to stop..." << std::endl;

    // 別スレッドでカメラの接続状態を監視
    std::atomic<bool> stopMonitoring{false};
    std::thread monitorThread([&]()
                              {
      while (!stopMonitoring)
      {
        if (!tracker.isConnected())
        {
          connectionLost = true;
          std::cout << "\nAlert: Camera connection lost!" << std::endl;
          break;
        }
        // 100ミリ秒ごとに接続状態をチェック
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } });

    // 入力待機またはカメラ切断までループ
    while (!connectionLost)
    {
      // 入力があるかチェック (非ブロッキング)
      if (std::cin.rdbuf()->in_avail() > 0)
      {
        break; // 入力があれば終了
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 監視スレッドを停止
    stopMonitoring = true;
    if (monitorThread.joinable())
    {
      monitorThread.join();
    }

    // トラッキングを停止
    tracker.stopTracking();

    if (connectionLost)
    {
      std::cout << "Tracking stopped due to camera disconnection." << std::endl;
    }
    else
    {
      std::cout << "Tracking stopped by user." << std::endl;
    }
  }
  else
  {
    std::cerr << "Failed to start tracking" << std::endl;
  }

  // 終了処理
  tracker.shutdown();
  std::cout << "KinectBodyTracker shutdown" << std::endl;

  return 0;
}
