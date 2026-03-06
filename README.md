# gz_dvs_plugin — DVS Event Camera Plugin for Gazebo Harmonic

A Gazebo Harmonic (gz-sim8) system plugin that emulates a DVS (Dynamic Vision Sensor) event camera.
It generates asynchronous brightness-change events from an existing camera sensor's rendered frames using the [v2e](https://github.com/SensorsINI/v2e) algorithm.

Default parameters are tuned to match the **iniVation DAVIS346** hardware specifications.

---

## Features

- **v2e algorithm**: Lin-log intensity transform, IIR lowpass filter, sub-frame timestamp interpolation
- **Per-pixel threshold mismatch**: Gaussian distribution (sigma=0.03) to simulate real sensor manufacturing variance
- **Noise models**: Shot noise (Poisson), leak current (ON-biased)
- **Refractory period**: Per-pixel dead time after each event (default 1ms)
- **Dual output**: ROS 2 topic (`DvsEventArray`) + Gazebo transport (for GUI ImageDisplay)
- **Visualization**: ON=white, OFF=black, background=gray event image

---

## DAVIS346 Default Parameters

| Parameter | Value | Real Spec |
|-----------|-------|-----------|
| Resolution | 346 x 260 | 346 x 260 |
| ON threshold | 0.13 (ln) | ~14.3% |
| OFF threshold | 0.20 (ln) | ~22.5% |
| Sigma threshold | 0.03 | Manufacturing mismatch |
| Refractory period | 1ms | 1us (simulation limited) |
| Dynamic range | 48dB | 120dB (8-bit source limited) |

---

## Dependencies

- ROS 2 Jazzy
- Gazebo Harmonic (gz-sim8, gz-rendering8, gz-plugin2, gz-transport13, gz-msgs10)
- OpenCV (core, imgproc)

---

## Build

```bash
cd ~/space_ros_ws
colcon build --symlink-install --packages-select gz_dvs_plugin
source install/setup.bash
```

---

## Usage

### 1. Add the plugin to your World SDF

```xml
<plugin filename="libgz-dvs-system.so" name="gz_dvs_plugin::DvsSystem">
  <camera_sensor_name>model_name::link_name::camera_sensor</camera_sensor_name>
  <ros_topic>/dvs/events</ros_topic>
  <frame_id>dvs_camera</frame_id>
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

> **Note:** `camera_sensor_name` uses substring matching against the gz-rendering sensor name.
> Use the full qualified path (e.g., `model::link::sensor`) to avoid matching the wrong camera when multiple cameras exist in the scene.

### 2. Display DVS output in Gazebo GUI

Add to the `<gui>` section of your World SDF:
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

### 3. Environment variables

The plugin library path must be set so Gazebo can find it. In a ROS 2 launch file:
```python
env = {
    'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join(filter(None, [
        environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        os.path.join(get_package_share_directory('gz_dvs_plugin'), '..', '..', 'lib'),
    ])),
}
```

### 4. Verify

```bash
ros2 topic list | grep dvs
ros2 topic hz /dvs/events
ros2 topic echo /dvs/events --once
```

---

## SDF Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_sensor_name` | (required) | Target camera sensor name (substring match) |
| `ros_topic` | `/dvs/events` | ROS 2 event array topic |
| `frame_id` | `dvs_camera` | TF frame ID |
| `pos_threshold` | 0.2 | ON event threshold (ln units) |
| `neg_threshold` | 0.2 | OFF event threshold (ln units) |
| `sigma_threshold` | 0.03 | Per-pixel threshold standard deviation |
| `lin_log_threshold` | 20.0 | Lin-log crossover intensity |
| `lpf_tau` | 0.0 | IIR lowpass time constant in seconds (0 = disabled) |
| `shot_noise_rate` | 0.0 | Shot noise rate (events/pixel/sec) |
| `leak_rate` | 0.0 | Leak current rate (events/pixel/sec, ON-biased) |
| `refractory_period` | 0.0 | Dead time after each event (seconds) |
| `publish_visualization` | false | Enable visualization image publishing |
| `viz_topic` | `/dvs/image` | ROS 2 visualization image topic |
| `gz_viz_topic` | `dvs/viz` | Gazebo transport visualization topic |

---

## Message Definitions

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

## Architecture

```
[Rendering Thread — PostRender]          [Sim Thread — PostUpdate]
 Scene -> Camera -> Copy(image)           Update simTime (atomic)
 RGB -> Gray -> float -> lin_log          Swap event buffer (mutex)
 IIR lowpass (optional)                   Build DvsEventArray -> ROS 2 publish
 diff = filtered - base                   Viz image -> ROS 2 + gz transport
 n = floor(diff / threshold)
 Sub-frame timestamp interpolation
 base += n * threshold
 events -> buffer (mutex)
```

Thread safety is ensured via `std::mutex` for the event buffer, `std::atomic<double>` for simulation time, and `std::atomic<bool>` for camera discovery state.

---

## Known Limitations

| Aspect | Simulation | Real DAVIS346 |
|--------|------------|---------------|
| Dynamic range | 48 dB (8-bit source) | 120 dB |
| Temporal resolution | ~16.7ms (60Hz camera) | 1 us |
| Bandwidth | Limited by camera fps | 12M events/s |

These limitations stem from Gazebo's 8-bit rendering pipeline and camera frame rate, not from the DVS algorithm itself. Higher camera fps yields finer temporal resolution at the cost of increased rendering load.

---

## Related Projects

- [orbit_sim](https://github.com/ndh8205/mysrc_sprs_backup) — Space ROS orbital simulation package that uses this plugin
- [v2e](https://github.com/SensorsINI/v2e) — Original video-to-events conversion algorithm (Python)
- [ESIM](https://rpg.ifi.uzh.ch/esim.html) — ETH Zurich event camera simulator

---

## License

Apache-2.0

---

*Last updated: 2026-03-06*
