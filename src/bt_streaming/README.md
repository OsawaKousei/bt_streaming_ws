# bt_streaming

Azure Kinect DKを使用したリアルタイムボディトラッキングのROS2パッケージです。人体の骨格情報を検出し、ROS2メッセージとして配信します。

## 機能

- Azure Kinect DKからのリアルタイムボディトラッキング
- 同期モードと非同期モードの選択可能
- 複数人同時検出対応
- 32個の関節点の位置・姿勢・信頼度情報を取得
- ROS2 JointStateメッセージでの配信
- 設定可能なフレームレートとフレームID

## 必要条件

### システム要件
- Ubuntu 22.04
- ROS2 Humble
- NVIDIA GPU (CUDA対応)
- 別添の手順書に従い、Kinectの依存関係をインストールしていること

## インストール

```bash
# ワークスペースに移動
cd /home/kousei/bt_streaming_ws

# 依存関係のインストール
rosdep install --from-paths src --ignore-src -r -y

# ビルド
colcon build --packages-select bt_streaming

# 環境設定
source install/setup.bash
```

## 使用方法

```bash
# config/body_tracking_params.yamlの設定を読み込みます
ros2 launch bt_streaming body_tracking.launch.py
```

### トピックの確認

```bash
# 利用可能なトピックを表示
ros2 topic list

# ボディトラッキング結果を表示
ros2 topic echo /body_tracking/joint_states
```

## 設定パラメータ

### body_tracking_params.yaml

```yaml
body_tracking:
  ros__parameters:
    # パブリッシュレート (Hz)
    publish_rate: 30.0
    
    # フレームID
    frame_id: "kinect_link"
    
    # 非同期モードの使用 (true: 非同期, false: 同期)
    use_async_mode: true
```

### パラメータ詳細

| パラメータ | 型 | デフォルト値 | 説明 |
|-----------|-----|-------------|------|
| `publish_rate` | double | 30.0 | データ配信頻度 (Hz) |
| `frame_id` | string | "kinect_link" | 座標系のフレームID |
| `use_async_mode` | bool | true | 非同期モードの使用 |

## 動作モード

### 同期モード (use_async_mode: false)
- ROS2タイマーによる定期取得
- 指定されたpublish_rateで動作

### 非同期モード (use_async_mode: true)
- 専用スレッドでの連続取得
- Kinectのネイティブ頻度での動作

## 出力データ

### JointStateメッセージ形式

```yaml
header:
  stamp: # タイムスタンプ
  frame_id: "kinect_link"
name: 
  - "body_0_pelvis"
  - "body_0_spine_navel"
  # ... 32個の関節名
position: [x, y, z, ...] # 位置 (メートル)
velocity: [0.0, 0.0, 0.0, ...] # 未使用
effort: [confidence, confidence, confidence, ...] # 信頼度
```

### 関節点一覧

1. pelvis (骨盤)
2. spine_navel (脊椎・へそ)
3. spine_chest (脊椎・胸部)
4. neck (首)
5. clavicle_left/right (鎖骨)
6. shoulder_left/right (肩)
7. elbow_left/right (肘)
8. wrist_left/right (手首)
9. hand_left/right (手)
10. handtip_left/right (指先)
11. thumb_left/right (親指)
12. hip_left/right (腰)
13. knee_left/right (膝)
14. ankle_left/right (足首)
15. foot_left/right (足)
16. head (頭)
17. nose (鼻)
18. eye_left/right (目)
19. ear_left/right (耳)

## トラブルシューティング
- Kinectの依存関係が適切にインストールされていることを確認してください
- OrbbrcSDKのLD_LIBRARY_PATHが正しく環境変数に追加されていることを確認してください
- SDKのに同梱されているsimple_3dViewerなどで、カメラの動作確認を行ってください

## API仕様

主要クラス: `kinect_tracker::KinectBodyTracker`

```cpp
// 初期化
bool initialize(const DeviceConfig &config);

// 同期データ取得
bool getBodyTrackingResult(BodyTrackingResult &result);

// 非同期処理開始
bool startTracking();
void setBodyTrackingCallback(BodyTrackingCallback callback);
```

## 作成者

Kousei (jwithelis@gmail.com)