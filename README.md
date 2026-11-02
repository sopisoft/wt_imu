# wt_imu

WitMotion IMUを読み取るROS 2ドライバー。

```sh
colcon build --packages-select wt_imu --symlink-install
source install/setup.bash
ros2 launch wt_imu wt_imu.launch.xml
```

設定は`config/bwt901bcl.yaml`。接続先`device`、通信速度`baud_rate`、`frame_id`を指定する。
`auto_reconnect`は再接続、`sync_sensor_time`はセンサー時刻の同期を制御する。
`configure_sensor`がtrueなら出力設定をセンサーへ書き込む。設定値-1は現在値を維持する。

主な出力は`imu`（Imu）、`magnetometer`（MagneticField）、`temperature`（Temperature）。
気圧・高度・Quaternion・GPS・速度・デジタル入力も配信でき、トピック名は設定で変更する。
