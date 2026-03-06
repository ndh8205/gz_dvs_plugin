#include "gz_dvs_plugin/DvsSystem.hh"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <gz/plugin/Register.hh>
#include <gz/sim/rendering/Events.hh>

#include <opencv2/imgproc.hpp>

using namespace gz_dvs_plugin;

// ============================================================
// Constructor / Destructor
// ============================================================

DvsSystem::DvsSystem()
    : rng_(std::random_device{}())
{
}

DvsSystem::~DvsSystem()
{
  rosNode_.reset();
}

// ============================================================
// Configure
// ============================================================

void DvsSystem::Configure(
    const gz::sim::Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &_eventMgr)
{
  // Parse SDF parameters
  auto sdf = const_cast<sdf::Element *>(_sdf.get());

  if (sdf->HasElement("camera_sensor_name"))
    cameraSensorName_ = sdf->Get<std::string>("camera_sensor_name");
  else
    gzerr << "[DvsSystem] Missing <camera_sensor_name>!\n";

  if (sdf->HasElement("ros_topic"))
    rosTopic_ = sdf->Get<std::string>("ros_topic");
  if (sdf->HasElement("frame_id"))
    frameId_ = sdf->Get<std::string>("frame_id");
  if (sdf->HasElement("pos_threshold"))
    posThreshold_ = sdf->Get<float>("pos_threshold");
  if (sdf->HasElement("neg_threshold"))
    negThreshold_ = sdf->Get<float>("neg_threshold");
  if (sdf->HasElement("sigma_threshold"))
    sigmaThreshold_ = sdf->Get<float>("sigma_threshold");
  if (sdf->HasElement("lin_log_threshold"))
    linLogThreshold_ = sdf->Get<float>("lin_log_threshold");
  if (sdf->HasElement("lpf_tau"))
    lpfTau_ = sdf->Get<float>("lpf_tau");
  if (sdf->HasElement("shot_noise_rate"))
    shotNoiseRate_ = sdf->Get<float>("shot_noise_rate");
  if (sdf->HasElement("leak_rate"))
    leakRate_ = sdf->Get<float>("leak_rate");
  if (sdf->HasElement("refractory_period"))
    refractoryPeriod_ = sdf->Get<float>("refractory_period");
  if (sdf->HasElement("publish_visualization"))
    publishVisualization_ = sdf->Get<bool>("publish_visualization");
  if (sdf->HasElement("viz_topic"))
    vizTopic_ = sdf->Get<std::string>("viz_topic");
  if (sdf->HasElement("gz_viz_topic"))
    gzVizTopic_ = sdf->Get<std::string>("gz_viz_topic");

  // Connect to PostRender event
  postRenderConn_ = _eventMgr.Connect<gz::sim::events::PostRender>(
      std::bind(&DvsSystem::OnPostRender, this));

  // Initialize ROS 2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rosNode_ = std::make_shared<rclcpp::Node>("gz_dvs_emulator");
  eventPub_ = rosNode_->create_publisher<gz_dvs_plugin::msg::DvsEventArray>(
      rosTopic_, rclcpp::QoS(10));

  if (publishVisualization_) {
    vizPub_ = rosNode_->create_publisher<sensor_msgs::msg::Image>(
        vizTopic_, rclcpp::QoS(1));
    gzVizPub_ = gzNode_.Advertise<gz::msgs::Image>(gzVizTopic_);
  }

  gzmsg << "[DvsSystem] Configured: camera=" << cameraSensorName_
        << " topic=" << rosTopic_
        << " pos_thresh=" << posThreshold_
        << " neg_thresh=" << negThreshold_ << "\n";
}

// ============================================================
// PostUpdate (Simulation thread)
// ============================================================

void DvsSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &/*_ecm*/)
{
  double simSec = std::chrono::duration<double>(_info.simTime).count();
  currentSimTime_.store(simSec);

  if (_info.paused) return;
  if (!cameraFound_.load()) return;

  // Drain event buffer
  {
    std::lock_guard<std::mutex> lock(eventMutex_);
    eventBufferPublish_.swap(eventBufferRender_);
    eventBufferRender_.clear();
  }

  if (eventBufferPublish_.empty()) return;

  // Build ROS 2 message
  auto msg = std::make_unique<gz_dvs_plugin::msg::DvsEventArray>();
  msg->header.stamp.sec = static_cast<int32_t>(simSec);
  msg->header.stamp.nanosec =
      static_cast<uint32_t>((simSec - msg->header.stamp.sec) * 1e9);
  msg->header.frame_id = frameId_;
  msg->width = imgWidth_;
  msg->height = imgHeight_;

  msg->events.reserve(eventBufferPublish_.size());
  for (const auto &evt : eventBufferPublish_) {
    gz_dvs_plugin::msg::DvsEvent e;
    e.x = evt.x;
    e.y = evt.y;
    e.ts.sec = static_cast<int32_t>(evt.timestamp);
    e.ts.nanosec =
        static_cast<uint32_t>((evt.timestamp - e.ts.sec) * 1e9);
    e.polarity = evt.polarity;
    msg->events.push_back(e);
  }

  eventPub_->publish(std::move(msg));

  // Visualization image
  if (publishVisualization_ && vizPub_) {
    // Build pixel data (shared between ROS and GZ)
    // Draw each event as a 3x3 box for visibility
    const int radius = 1;
    std::vector<uint8_t> pixels(imgWidth_ * imgHeight_ * 3, 128);
    for (const auto &evt : eventBufferPublish_) {
      uint8_t r, g, b;
      if (evt.polarity) {
        b = 255; g = 255; r = 255;  // white = ON
      } else {
        b = 0; g = 0; r = 0;        // black = OFF
      }
      int x0 = std::max(0, static_cast<int>(evt.x) - radius);
      int y0 = std::max(0, static_cast<int>(evt.y) - radius);
      int x1 = std::min(static_cast<int>(imgWidth_) - 1, static_cast<int>(evt.x) + radius);
      int y1 = std::min(static_cast<int>(imgHeight_) - 1, static_cast<int>(evt.y) + radius);
      for (int py = y0; py <= y1; ++py) {
        for (int px = x0; px <= x1; ++px) {
          size_t idx = (py * imgWidth_ + px) * 3;
          pixels[idx] = b; pixels[idx+1] = g; pixels[idx+2] = r;
        }
      }
    }

    // ROS 2 publish
    auto vizMsg = std::make_unique<sensor_msgs::msg::Image>();
    vizMsg->header.stamp.sec = static_cast<int32_t>(simSec);
    vizMsg->header.stamp.nanosec =
        static_cast<uint32_t>((simSec - vizMsg->header.stamp.sec) * 1e9);
    vizMsg->header.frame_id = frameId_;
    vizMsg->height = imgHeight_;
    vizMsg->width = imgWidth_;
    vizMsg->encoding = "bgr8";
    vizMsg->step = imgWidth_ * 3;
    vizMsg->data = pixels;
    vizPub_->publish(std::move(vizMsg));

    // Gazebo transport publish (for GUI ImageDisplay)
    if (gzVizPub_.Valid()) {
      gz::msgs::Image gzImg;
      gzImg.set_width(imgWidth_);
      gzImg.set_height(imgHeight_);
      gzImg.set_pixel_format_type(gz::msgs::PixelFormatType::RGB_INT8);
      gzImg.set_step(imgWidth_ * 3);
      // Convert BGR→RGB for gz
      std::vector<uint8_t> rgbPixels(pixels.size());
      for (size_t i = 0; i < pixels.size(); i += 3) {
        rgbPixels[i]   = pixels[i+2];  // R
        rgbPixels[i+1] = pixels[i+1];  // G
        rgbPixels[i+2] = pixels[i];    // B
      }
      gzImg.set_data(rgbPixels.data(), rgbPixels.size());
      gzVizPub_.Publish(gzImg);
    }
  }

  eventBufferPublish_.clear();
}

// ============================================================
// OnPostRender (Rendering thread)
// ============================================================

void DvsSystem::OnPostRender()
{
  // Lazy camera lookup
  if (!cameraFound_.load()) {
    auto scene = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene) return;

    for (unsigned int i = 0; i < scene->SensorCount(); ++i) {
      auto sensor = scene->SensorByIndex(i);
      if (!sensor) continue;
      if (sensor->Name().find(cameraSensorName_) != std::string::npos) {
        camera_ = std::dynamic_pointer_cast<gz::rendering::Camera>(sensor);
        if (camera_) {
          imgWidth_ = camera_->ImageWidth();
          imgHeight_ = camera_->ImageHeight();
          renderImage_ = camera_->CreateImage();
          cameraFound_.store(true);
          gzmsg << "[DvsSystem] Found camera: " << sensor->Name()
                << " (" << imgWidth_ << "x" << imgHeight_ << ")\n";
          break;
        }
      }
    }
    if (!cameraFound_.load()) return;
  }

  // Capture frame
  camera_->Copy(renderImage_);
  const auto *data = renderImage_.Data<unsigned char>();
  if (!data) return;

  // RGB → Grayscale float32
  cv::Mat rgb(imgHeight_, imgWidth_, CV_8UC3,
              const_cast<unsigned char *>(data));
  cv::Mat gray8;
  cv::cvtColor(rgb, gray8, cv::COLOR_RGB2GRAY);
  cv::Mat grayFloat;
  gray8.convertTo(grayFloat, CV_32F);

  double simTime = currentSimTime_.load();
  ProcessFrame(grayFloat, simTime);
}

// ============================================================
// LinLog
// ============================================================

float DvsSystem::LinLog(float x, float threshold)
{
  if (x <= threshold) {
    return x;
  }
  return threshold * (1.0f + std::log(x / threshold));
}

// ============================================================
// InitializeState
// ============================================================

void DvsSystem::InitializeState(const cv::Mat &firstFrame, double simTime)
{
  int rows = firstFrame.rows;
  int cols = firstFrame.cols;

  baseLogFrame_ = cv::Mat(rows, cols, CV_32F);
  lpfLogFrame_ = cv::Mat(rows, cols, CV_32F);

  for (int r = 0; r < rows; ++r) {
    const float *src = firstFrame.ptr<float>(r);
    float *baseDst = baseLogFrame_.ptr<float>(r);
    float *lpfDst = lpfLogFrame_.ptr<float>(r);
    for (int c = 0; c < cols; ++c) {
      float val = LinLog(src[c], linLogThreshold_);
      baseDst[c] = val;
      lpfDst[c] = val;
    }
  }

  // Per-pixel thresholds with Gaussian mismatch
  perPixelPosThresh_ = cv::Mat(rows, cols, CV_32F, cv::Scalar(posThreshold_));
  perPixelNegThresh_ = cv::Mat(rows, cols, CV_32F, cv::Scalar(negThreshold_));

  if (sigmaThreshold_ > 0.0f) {
    std::normal_distribution<float> dist(0.0f, sigmaThreshold_);
    for (int r = 0; r < rows; ++r) {
      float *posPtr = perPixelPosThresh_.ptr<float>(r);
      float *negPtr = perPixelNegThresh_.ptr<float>(r);
      for (int c = 0; c < cols; ++c) {
        float noise = dist(rng_);
        posPtr[c] = std::max(0.01f, posThreshold_ + noise);
        negPtr[c] = std::max(0.01f, negThreshold_ + noise);
      }
    }
  }

  lastEventTime_ = cv::Mat(rows, cols, CV_64F, cv::Scalar(simTime));
  prevSimTime_ = simTime;
  stateInitialized_ = true;

  gzmsg << "[DvsSystem] State initialized: "
        << cols << "x" << rows << " pixels\n";
}

// ============================================================
// ProcessFrame — Core DVS Algorithm
// ============================================================

void DvsSystem::ProcessFrame(const cv::Mat &currentGray, double simTime)
{
  if (!stateInitialized_) {
    InitializeState(currentGray, simTime);
    return;
  }

  double dt = simTime - prevSimTime_;
  if (dt <= 1e-6) return;

  int rows = currentGray.rows;
  int cols = currentGray.cols;

  // 1. Lin-log transform
  cv::Mat currentLog(rows, cols, CV_32F);
  for (int r = 0; r < rows; ++r) {
    const float *src = currentGray.ptr<float>(r);
    float *dst = currentLog.ptr<float>(r);
    for (int c = 0; c < cols; ++c) {
      dst[c] = LinLog(src[c], linLogThreshold_);
    }
  }

  // 2. IIR lowpass filter (optional)
  cv::Mat filteredLog;
  if (lpfTau_ > 0.0f) {
    float alpha = static_cast<float>(dt) / (lpfTau_ + static_cast<float>(dt));
    filteredLog = lpfLogFrame_ + alpha * (currentLog - lpfLogFrame_);
    filteredLog.copyTo(lpfLogFrame_);
  } else {
    filteredLog = currentLog;
    currentLog.copyTo(lpfLogFrame_);
  }

  // 3. Diff from base reference
  cv::Mat diff = filteredLog - baseLogFrame_;

  // 4. Per-pixel event generation
  std::vector<RawEvent> localEvents;
  localEvents.reserve(2000);

  for (int r = 0; r < rows; ++r) {
    const float *diffPtr = diff.ptr<float>(r);
    const float *posThPtr = perPixelPosThresh_.ptr<float>(r);
    const float *negThPtr = perPixelNegThresh_.ptr<float>(r);
    float *basePtr = baseLogFrame_.ptr<float>(r);
    double *lastTPtr = lastEventTime_.ptr<double>(r);

    for (int c = 0; c < cols; ++c) {
      float d = diffPtr[c];
      float posT = posThPtr[c];
      float negT = negThPtr[c];

      int nEvents = 0;
      bool polarity = true;

      if (d >= posT) {
        nEvents = static_cast<int>(d / posT);
        polarity = true;
      } else if (d <= -negT) {
        nEvents = static_cast<int>(-d / negT);
        polarity = false;
      }

      if (nEvents == 0) continue;

      // Refractory period check
      if (refractoryPeriod_ > 0.0f &&
          (simTime - lastTPtr[c]) < refractoryPeriod_) {
        continue;
      }

      // Sub-frame timestamp interpolation
      float threshold = polarity ? posT : negT;
      float absD = std::abs(d);
      for (int e = 0; e < nEvents; ++e) {
        float fraction = (static_cast<float>(e + 1) * threshold) / absD;
        fraction = std::min(fraction, 1.0f);
        double eventTime = prevSimTime_ + fraction * dt;

        localEvents.push_back({
            static_cast<uint16_t>(c),
            static_cast<uint16_t>(r),
            eventTime,
            polarity
        });
      }

      // Update base reference (v2e pattern: advance by n*threshold)
      float advance = static_cast<float>(nEvents) * threshold;
      if (polarity) {
        basePtr[c] += advance;
      } else {
        basePtr[c] -= advance;
      }

      lastTPtr[c] = simTime;
    }
  }

  // 5. Shot noise (optional)
  if (shotNoiseRate_ > 0.0f) {
    float expectedNoise = shotNoiseRate_ *
        static_cast<float>(rows * cols) * static_cast<float>(dt);
    std::poisson_distribution<int> poissonDist(
        std::max(expectedNoise, 0.001f));
    int nNoise = poissonDist(rng_);
    std::uniform_int_distribution<int> xDist(0, cols - 1);
    std::uniform_int_distribution<int> yDist(0, rows - 1);
    std::uniform_real_distribution<double> tDist(prevSimTime_, simTime);
    std::bernoulli_distribution polDist(0.5);

    for (int i = 0; i < nNoise; ++i) {
      localEvents.push_back({
          static_cast<uint16_t>(xDist(rng_)),
          static_cast<uint16_t>(yDist(rng_)),
          tDist(rng_),
          polDist(rng_)
      });
    }
  }

  // 6. Leak current (optional, ON bias)
  if (leakRate_ > 0.0f) {
    float expectedLeak = leakRate_ *
        static_cast<float>(rows * cols) * static_cast<float>(dt);
    std::poisson_distribution<int> poissonDist(
        std::max(expectedLeak, 0.001f));
    int nLeak = poissonDist(rng_);
    std::uniform_int_distribution<int> xDist(0, cols - 1);
    std::uniform_int_distribution<int> yDist(0, rows - 1);
    std::uniform_real_distribution<double> tDist(prevSimTime_, simTime);

    for (int i = 0; i < nLeak; ++i) {
      localEvents.push_back({
          static_cast<uint16_t>(xDist(rng_)),
          static_cast<uint16_t>(yDist(rng_)),
          tDist(rng_),
          true  // leak = ON
      });
    }
  }

  // 7. Sort by timestamp
  std::sort(localEvents.begin(), localEvents.end(),
      [](const RawEvent &a, const RawEvent &b) {
        return a.timestamp < b.timestamp;
      });

  // 8. Push to shared buffer
  if (!localEvents.empty()) {
    std::lock_guard<std::mutex> lock(eventMutex_);
    eventBufferRender_.insert(eventBufferRender_.end(),
        localEvents.begin(), localEvents.end());
  }

  prevSimTime_ = simTime;
}

// ============================================================
// Plugin Registration
// ============================================================

GZ_ADD_PLUGIN(
    gz_dvs_plugin::DvsSystem,
    gz::sim::System,
    gz_dvs_plugin::DvsSystem::ISystemConfigure,
    gz_dvs_plugin::DvsSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz_dvs_plugin::DvsSystem,
    "gz_dvs_plugin::DvsSystem")
