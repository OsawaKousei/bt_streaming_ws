#include "KinectBodyTracker.h"
#include <iostream>
#include <thread>
#include <chrono>

namespace kinect_tracker
{

  // コンストラクタ
  KinectBodyTracker::KinectBodyTracker()
      : m_device(nullptr),
        m_tracker(nullptr),
        m_initialized(false),
        m_isRunning(false)
  {
  }

  // デストラクタ
  KinectBodyTracker::~KinectBodyTracker()
  {
    shutdown();
  }

  // デバイスの初期化
  bool KinectBodyTracker::initialize(const DeviceConfig &config)
  {
    if (m_initialized)
    {
      std::cerr << "KinectBodyTracker is already initialized" << std::endl;
      return false;
    }

    // 設定を保存
    m_config = config;

    // デバイスを開く
    if (k4a_device_open(0, &m_device) != K4A_RESULT_SUCCEEDED)
    {
      std::cerr << "Failed to open K4A device" << std::endl;
      return false;
    }

    // カメラ設定
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = m_config.depthMode;
    deviceConfig.color_resolution = m_config.colorResolution;
    deviceConfig.camera_fps = m_config.fps;

    // カメラ開始
    if (k4a_device_start_cameras(m_device, &deviceConfig) != K4A_RESULT_SUCCEEDED)
    {
      std::cerr << "Failed to start K4A cameras" << std::endl;
      k4a_device_close(m_device);
      m_device = nullptr;
      return false;
    }

    // キャリブレーション情報の取得
    if (k4a_device_get_calibration(m_device, deviceConfig.depth_mode, deviceConfig.color_resolution, &m_calibration) != K4A_RESULT_SUCCEEDED)
    {
      std::cerr << "Failed to get calibration" << std::endl;
      k4a_device_stop_cameras(m_device);
      k4a_device_close(m_device);
      m_device = nullptr;
      return false;
    }

    // ボディトラッカーの作成
    k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    trackerConfig.processing_mode = m_config.processingMode;

    // モデルパスが指定されている場合は設定
    if (!m_config.modelPath.empty())
    {
      trackerConfig.model_path = m_config.modelPath.c_str();
    }

    if (k4abt_tracker_create(&m_calibration, trackerConfig, &m_tracker) != K4A_RESULT_SUCCEEDED)
    {
      std::cerr << "Failed to create body tracker" << std::endl;
      k4a_device_stop_cameras(m_device);
      k4a_device_close(m_device);
      m_device = nullptr;
      return false;
    }

    m_initialized = true;
    return true;
  }

  // 終了処理
  void KinectBodyTracker::shutdown()
  {
    // 非同期処理を停止
    stopTracking();

    // トラッカーの解放
    if (m_tracker != nullptr)
    {
      k4abt_tracker_shutdown(m_tracker);
      k4abt_tracker_destroy(m_tracker);
      m_tracker = nullptr;
    }

    // デバイスの解放
    if (m_device != nullptr)
    {
      k4a_device_stop_cameras(m_device);
      k4a_device_close(m_device);
      m_device = nullptr;
    }

    m_initialized = false;
  }

  // カメラが接続されているかチェック
  bool KinectBodyTracker::isConnected() const
  {
    return m_initialized && m_device != nullptr;
  }

  // 1フレーム分の骨格検出結果を取得（同期処理）
  bool KinectBodyTracker::getBodyTrackingResult(BodyTrackingResult &result)
  {
    if (!m_initialized)
    {
      std::cerr << "KinectBodyTracker is not initialized" << std::endl;
      return false;
    }

    // キャプチャの取得
    k4a_capture_t capture = nullptr;
    k4a_wait_result_t captureResult = k4a_device_get_capture(m_device, &capture, 1000); // 1秒待つ

    if (captureResult != K4A_WAIT_RESULT_SUCCEEDED)
    {
      if (captureResult == K4A_WAIT_RESULT_TIMEOUT)
      {
        std::cerr << "Timed out waiting for capture" << std::endl;
      }
      else
      {
        std::cerr << "Failed to get capture" << std::endl;
      }
      return false;
    }

    // 深度画像の確認
    k4a_image_t depthImage = k4a_capture_get_depth_image(capture);
    if (depthImage == nullptr)
    {
      std::cerr << "Failed to get depth image" << std::endl;
      k4a_capture_release(capture);
      return false;
    }
    k4a_image_release(depthImage);

    // キャプチャをトラッカーにエンキュー
    if (k4abt_tracker_enqueue_capture(m_tracker, capture, K4A_WAIT_INFINITE) != K4A_WAIT_RESULT_SUCCEEDED)
    {
      std::cerr << "Failed to enqueue capture" << std::endl;
      k4a_capture_release(capture);
      return false;
    }

    // キャプチャを解放
    k4a_capture_release(capture);

    // 骨格フレームの取得
    k4abt_frame_t bodyFrame = nullptr;
    if (k4abt_tracker_pop_result(m_tracker, &bodyFrame, K4A_WAIT_INFINITE) != K4A_WAIT_RESULT_SUCCEEDED)
    {
      std::cerr << "Failed to pop body tracking result" << std::endl;
      return false;
    }

    // 結果の変換
    convertBodyFrame(bodyFrame, result);

    // 骨格フレームの解放
    k4abt_frame_release(bodyFrame);

    return true;
  }

  // 骨格フレームを結果構造体に変換
  void KinectBodyTracker::convertBodyFrame(k4abt_frame_t bodyFrame, BodyTrackingResult &result)
  {
    // ボディ数の取得
    uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);

    // タイムスタンプの取得
    result.timestamp = k4abt_frame_get_device_timestamp_usec(bodyFrame);
    result.bodies.clear();
    result.bodies.reserve(numBodies);

    // 各ボディの情報を取得
    for (uint32_t i = 0; i < numBodies; i++)
    {
      BodyInfo bodyInfo;
      k4abt_body_t body;

      // スケルトン情報の取得
      if (k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton) != K4A_RESULT_SUCCEEDED)
      {
        std::cerr << "Failed to get body skeleton" << std::endl;
        continue;
      }

      // ボディIDの取得
      bodyInfo.id = k4abt_frame_get_body_id(bodyFrame, i);

      // ジョイント情報の取得
      bodyInfo.joints.reserve(K4ABT_JOINT_COUNT);
      for (int j = 0; j < K4ABT_JOINT_COUNT; j++)
      {
        JointInfo jointInfo;
        jointInfo.jointId = static_cast<k4abt_joint_id_t>(j);
        jointInfo.confidenceLevel = body.skeleton.joints[j].confidence_level;

        // 位置情報
        jointInfo.position[0] = body.skeleton.joints[j].position.xyz.x;
        jointInfo.position[1] = body.skeleton.joints[j].position.xyz.y;
        jointInfo.position[2] = body.skeleton.joints[j].position.xyz.z;

        // 回転情報
        jointInfo.orientation[0] = body.skeleton.joints[j].orientation.wxyz.x;
        jointInfo.orientation[1] = body.skeleton.joints[j].orientation.wxyz.y;
        jointInfo.orientation[2] = body.skeleton.joints[j].orientation.wxyz.z;
        jointInfo.orientation[3] = body.skeleton.joints[j].orientation.wxyz.w;

        bodyInfo.joints.push_back(jointInfo);
      }

      result.bodies.push_back(bodyInfo);
    }
  }

  // コールバック関数をセット
  void KinectBodyTracker::setBodyTrackingCallback(BodyTrackingCallback callback)
  {
    m_callback = callback;
  }

  // 非同期処理を開始
  bool KinectBodyTracker::startTracking()
  {
    if (!m_initialized)
    {
      std::cerr << "KinectBodyTracker is not initialized" << std::endl;
      return false;
    }

    if (m_isRunning)
    {
      std::cerr << "Tracking thread is already running" << std::endl;
      return false;
    }

    if (!m_callback)
    {
      std::cerr << "No callback function set" << std::endl;
      return false;
    }

    m_isRunning = true;
    m_trackingThread = std::make_unique<std::thread>(&KinectBodyTracker::trackingThread, this);
    return true;
  }

  // 非同期処理を停止
  void KinectBodyTracker::stopTracking()
  {
    if (m_isRunning)
    {
      m_isRunning = false;
      if (m_trackingThread && m_trackingThread->joinable())
      {
        m_trackingThread->join();
      }
      m_trackingThread.reset();
    }
  }

  // 非同期処理のスレッド関数
  void KinectBodyTracker::trackingThread()
  {
    while (m_isRunning)
    {
      BodyTrackingResult result;
      if (getBodyTrackingResult(result))
      {
        // コールバック関数を呼び出し
        if (m_callback)
        {
          m_callback(result);
        }
      }
      else
      {
        // エラーが発生した場合は少し待機
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

} // namespace kinect_tracker
