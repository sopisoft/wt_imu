# wt_imu

WitMotion IMUをROS 2で読むドライバです。

## Launch

```bash
source install/setup.bash
ros2 launch wt_imu wt_imu.launch.xml
```

設定ファイルは `config/bwt901bcl.yaml` です。主な設定は次のとおりです。

```yaml
device: "/dev/witmotion"
baud_rate: 115200
frame_id: "imu"
auto_reconnect: true
sync_sensor_time: true
configure_sensor: true
settings:
  output_rate: 6
  content: 2047
```

`configure_sensor` を `true` にすると，`settings.output_rate` と `settings.content` をセンサーへ書き込みます．値 `-1` はその項目のレジスタを書き換えず，センサーの現在値を維持します．
`content: 2047`（`0x7ff`）は，公式プロトコルで定義された全出力データを有効にします．

## Topics

| トピック | メッセージ |
| --- | --- |
| `imu` | `sensor_msgs/msg/Imu` |
| `magnetometer` | `sensor_msgs/msg/MagneticField` |
| `temperature` | `sensor_msgs/msg/Temperature` |
| `pressure` | `sensor_msgs/msg/FluidPressure` |
| `altitude` | `std_msgs/msg/Float32` |
| `quaternion` | `geometry_msgs/msg/QuaternionStamped` |
| `gps` | `sensor_msgs/msg/NavSatFix` |
| `velocity` | `geometry_msgs/msg/TwistStamped` |
| `digital_port` | `std_msgs/msg/UInt16MultiArray` |

トピック名と `frame_id` は設定ファイルで変更できます。

## Commands

```bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install --base-paths . --packages-select wt_imu
source install/setup.bash

ctest --test-dir build/wt_imu --output-on-failure -L gtest
ctest --test-dir build/wt_imu --output-on-failure -L linter

find include src test -type f \( -name "*.cpp" -o -name "*.hpp" \) -print0 \
  | xargs -0 clang-format -i
```
