# gz_dvs_plugin — DVS Event Camera Plugin for Gazebo Harmonic

Gazebo Harmonic (gz-sim8) 용 DVS (Dynamic Vision Sensor) 이벤트 카메라 에뮬레이션 시스템 플러그인.
[v2e](https://github.com/SensorsINI/v2e) 알고리즘 기반으로 기존 카메라 센서의 렌더링 이미지에서 이벤트를 생성합니다.

기본 파라미터는 **iniVation DAVIS346** 실제 스펙에 맞추어 설정되어 있습니다.

---

## 특징

- **v2e 알고리즘**: Lin-log 변환, IIR 저역통과 필터, 서브프레임 타임스탬프 보간
- **Per-pixel threshold mismatch**: 가우시안 분포 (sigma=0.03)로 실제 센서 특성 모사
- **노이즈 모델**: Shot noise (포아송), Leak current (ON 편향)
- **Refractory period**: 불응기 구현 (기본 1ms)
- **이중 출력**: ROS 2 토픽 (`DvsEventArray`) + Gazebo transport (GUI ImageDisplay용)
- **시각화**: ON=흰색, OFF=검정, 배경=회색 이벤트 영상

---

## DAVIS346 기본 파라미터

| 파라미터 | 값 | 실제 스펙 |
|----------|-----|-----------|
| 해상도 | 346 x 260 | 346 x 260 |
| ON threshold | 0.13 (ln) | ~14.3% |
| OFF threshold | 0.20 (ln) | ~22.5% |
| Sigma threshold | 0.03 | 제조 편차 |
| Refractory period | 1ms | 1us (시뮬레이션 제한) |
| Dynamic range | 48dB | 120dB (8bit 소스 제한) |

---

## 의존성

- ROS 2 Jazzy
- Gazebo Harmonic (gz-sim8, gz-rendering8, gz-plugin2, gz-transport13, gz-msgs10)
- OpenCV (core, imgproc)

---

## 빌드

```bash
cd ~/space_ros_ws
colcon build --symlink-install --packages-select gz_dvs_plugin
source install/setup.bash
```

---

## 사용법

### 1. World SDF에 플러그인 추가

```xml
<plugin filename="libgz-dvs-system.so" name="gz_dvs_plugin::DvsSystem">
  <camera_sensor_name>nasa_satellite3::nasa_satellite_link::satellite_camera</camera_sensor_name>
  <ros_topic>/dvs/events</ros_topic>
  <frame_id>nasa_satellite3/dvs_camera</frame_id>
  <pos_threshold>0.13</pos_threshold>
  <neg_threshold>0.20</neg_threshold>
  <sigma_threshold>0.03</sigma_threshold>
  <lpf_tau>0.01</lpf_tau>
  <shot_noise_rate>0.0</shot_noise_rate>
  <leak_rate>0.0</leak_rate>
  <refractory_period>0.001</refractory_period>
  <publish_visualization>true</publish_visualization>
  <viz_topic>/dvs/image</viz_topic>
</plugin>
```

### 2. Gazebo GUI에서 DVS 영상 표시

World SDF의 `<gui>` 섹션에 추가:
```xml
<plugin filename="ImageDisplay" name="DVS Event View">
  <gz-gui>
    <title>DVS Event Camera</title>
    <property type="bool" key="showTitleBar">true</property>
    <property type="string" key="state">docked</property>
  </gz-gui>
  <topic>dvs/viz</topic>
  <refresh_rate_hz>30</refresh_rate_hz>
</plugin>
```

### 3. 환경 변수

Launch 파일에서 플러그인 라이브러리 경로를 설정해야 합니다:
```python
env = {
    'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join(filter(None, [
        environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        os.path.join(get_package_share_directory('gz_dvs_plugin'), '..', '..', 'lib'),
    ])),
}
```

### 4. 검증

```bash
ros2 topic list | grep dvs
ros2 topic hz /dvs/events
ros2 topic echo /dvs/events --once
```

---

## SDF 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `camera_sensor_name` | (필수) | 대상 카메라 센서 이름 (substring 매칭) |
| `ros_topic` | `/dvs/events` | ROS 2 이벤트 토픽 |
| `frame_id` | `dvs_camera` | TF 프레임 ID |
| `pos_threshold` | 0.2 | ON 이벤트 임계값 (ln 단위) |
| `neg_threshold` | 0.2 | OFF 이벤트 임계값 (ln 단위) |
| `sigma_threshold` | 0.03 | Per-pixel 임계값 표준편차 |
| `lin_log_threshold` | 20.0 | Lin-log 전환 경계 |
| `lpf_tau` | 0.0 | IIR 저역통과 시정수 (0=비활성) |
| `shot_noise_rate` | 0.0 | Shot noise 비율 |
| `leak_rate` | 0.0 | Leak current 비율 |
| `refractory_period` | 0.0 | 불응기 (초) |
| `publish_visualization` | false | 시각화 영상 발행 여부 |
| `viz_topic` | `/dvs/image` | ROS 2 시각화 토픽 |
| `gz_viz_topic` | `dvs/viz` | Gazebo transport 시각화 토픽 |

---

## 메시지 정의

### DvsEvent.msg

```
uint16 x
uint16 y
builtin_interfaces/Time ts
bool polarity
```

### DvsEventArray.msg

```
std_msgs/Header header
uint32 width
uint32 height
DvsEvent[] events
```

---

## 아키텍처

```
[Rendering Thread — PostRender]          [Sim Thread — PostUpdate]
 Scene → Camera → Copy(image)            simTime 업데이트 (atomic)
 RGB → Gray → float → lin_log            eventBuffer swap (mutex)
 IIR lowpass (optional)                   DvsEventArray 빌드 → ROS 2 publish
 diff = filtered - base                   viz image → ROS 2 + gz transport
 n = floor(diff / threshold)
 서브프레임 타임스탬프 보간
 base += n * threshold
 events → buffer (mutex)
```

---

## 알려진 제한사항

| 항목 | 시뮬레이션 | 실제 DAVIS346 |
|------|------------|---------------|
| Dynamic range | 48 dB (8-bit 소스) | 120 dB |
| 시간 분해능 | ~16.7ms (60Hz 카메라) | 1 us |
| 대역폭 | 카메라 fps 제한 | 12M events/s |

이 제한은 Gazebo의 8-bit 렌더링과 카메라 프레임레이트에 기인하며, DVS 알고리즘 자체의 한계가 아닙니다.

---

## 관련 프로젝트

- [orbit_sim](https://github.com/ndh8205/mysrc_sprs_backup) — 이 플러그인을 사용하는 우주 시뮬레이션 패키지
- [v2e](https://github.com/SensorsINI/v2e) — 원본 비디오→이벤트 변환 알고리즘 (Python)
- [ESIM](https://rpg.ifi.uzh.ch/esim.html) — ETH Zurich 이벤트 카메라 시뮬레이터

---

*최종 업데이트: 2026-03-06*
