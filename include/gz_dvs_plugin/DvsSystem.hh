#ifndef GZ_DVS_PLUGIN__DVS_SYSTEM_HH_
#define GZ_DVS_PLUGIN__DVS_SYSTEM_HH_

#include <atomic>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/EventManager.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Image.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/common/Event.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gz_dvs_plugin/msg/dvs_event.hpp"
#include "gz_dvs_plugin/msg/dvs_event_array.hpp"

namespace gz_dvs_plugin
{

struct RawEvent
{
  uint16_t x;
  uint16_t y;
  double timestamp;
  bool polarity;  // true=ON, false=OFF
};

class DvsSystem
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPostUpdate
{
public:
  DvsSystem();
  ~DvsSystem() override;

  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

private:
  void OnPostRender();
  static float LinLog(float x, float threshold = 20.0f);
  void InitializeState(const cv::Mat &firstFrame, double simTime);
  void ProcessFrame(const cv::Mat &currentGray, double simTime);

  // SDF params
  std::string cameraSensorName_;
  std::string rosTopic_{"/dvs/events"};
  std::string frameId_{"dvs_camera"};
  float posThreshold_{0.2f};
  float negThreshold_{0.2f};
  float sigmaThreshold_{0.03f};
  float linLogThreshold_{20.0f};
  float lpfTau_{0.0f};
  float shotNoiseRate_{0.0f};
  float leakRate_{0.0f};
  float refractoryPeriod_{0.0f};
  bool publishVisualization_{false};
  std::string vizTopic_{"/dvs/image"};
  std::string gzVizTopic_{"dvs/viz"};

  // Rendering state
  std::atomic<bool> cameraFound_{false};
  bool stateInitialized_{false};
  gz::rendering::CameraPtr camera_{nullptr};
  gz::rendering::Image renderImage_;
  unsigned int imgWidth_{0};
  unsigned int imgHeight_{0};

  // DVS per-pixel state
  cv::Mat baseLogFrame_;
  cv::Mat lpfLogFrame_;
  cv::Mat perPixelPosThresh_;
  cv::Mat perPixelNegThresh_;
  cv::Mat lastEventTime_;
  double prevSimTime_{0.0};

  // Thread-safe event buffer
  std::mutex eventMutex_;
  std::vector<RawEvent> eventBufferRender_;
  std::vector<RawEvent> eventBufferPublish_;
  std::atomic<double> currentSimTime_{0.0};

  // ROS 2
  std::shared_ptr<rclcpp::Node> rosNode_{nullptr};
  rclcpp::Publisher<gz_dvs_plugin::msg::DvsEventArray>::SharedPtr eventPub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vizPub_;

  // Gazebo
  gz::common::ConnectionPtr postRenderConn_;
  std::mt19937 rng_;

  // Gazebo transport (for GUI ImageDisplay)
  gz::transport::Node gzNode_;
  gz::transport::Node::Publisher gzVizPub_;
};

}  // namespace gz_dvs_plugin

#endif
