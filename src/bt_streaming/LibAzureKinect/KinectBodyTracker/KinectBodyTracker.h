#pragma once

#include <k4a/k4a.h>
#include <k4abt.h>
#include <vector>
#include <string>
#include <memory>
#include <functional>

namespace kinect_tracker
{

  // 骨格のジョイント情報を格納する構造体
  struct JointInfo
  {
    k4abt_joint_id_t jointId;                       // ジョイントID
    float position[3];                              // 位置情報 (x, y, z)
    float orientation[4];                           // クォータニオン (x, y, z, w)
    k4abt_joint_confidence_level_t confidenceLevel; // 信頼度
  };

  // 1つの骨格情報を格納する構造体
  struct BodyInfo
  {
    uint32_t id;                   // ボディID
    std::vector<JointInfo> joints; // ジョイント情報の配列
  };

  // 検出結果を格納する構造体
  struct BodyTrackingResult
  {
    std::vector<BodyInfo> bodies; // 検出された骨格情報
    uint64_t timestamp;           // タイムスタンプ
  };

  // コールバック関数の型定義
  using BodyTrackingCallback = std::function<void(const BodyTrackingResult &)>;

  class KinectBodyTracker
  {
  public:
    // デバイス初期化設定
    struct DeviceConfig
    {
      k4a_depth_mode_t depthMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
      k4a_color_resolution_t colorResolution = K4A_COLOR_RESOLUTION_OFF;
      k4a_fps_t fps = K4A_FRAMES_PER_SECOND_30;
      k4abt_tracker_processing_mode_t processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
      std::string modelPath = "";
    };

    // コンストラクタ
    KinectBodyTracker();

    // デストラクタ
    ~KinectBodyTracker();

    // デバイスの初期化
    bool initialize(const DeviceConfig &config = DeviceConfig());

    // 終了処理
    void shutdown();

    // 1フレーム分の骨格検出結果を取得（同期処理）
    bool getBodyTrackingResult(BodyTrackingResult &result);

    // コールバック関数をセット（非同期処理用）
    void setBodyTrackingCallback(BodyTrackingCallback callback);

    // 非同期処理を開始
    bool startTracking();

    // 非同期処理を停止
    void stopTracking();

    // カメラが接続されているかチェック
    bool isConnected() const;

  private:
    // デバイスとトラッカーのポインタ
    k4a_device_t m_device;
    k4abt_tracker_t m_tracker;

    // デバイスの設定
    DeviceConfig m_config;

    // キャリブレーション情報
    k4a_calibration_t m_calibration;

    // 初期化フラグ
    bool m_initialized;

    // 非同期処理用
    bool m_isRunning;
    std::unique_ptr<std::thread> m_trackingThread;
    BodyTrackingCallback m_callback;

    // 非同期処理のスレッド関数
    void trackingThread();

    // 骨格情報を変換する
    void convertBodyFrame(k4abt_frame_t bodyFrame, BodyTrackingResult &result);
  };

} // namespace kinect_tracker
