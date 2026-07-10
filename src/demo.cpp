/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * sold by Shenzhen LDROBOT Co., LTD    
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros2_api.h"
#include "ldlidar_driver.h"
#include "scan_time_reconstruct.h"

void  ToLaserscanMessagePublish(ldlidar::LaserScan& src, double lidar_spin_freq, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub);

uint64_t GetSystemTimeStamp(void);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ldlidar_published"); // create a ROS2 Node
  std::string product_name;
  std::string topic_name;
  std::string port_name;
  int serial_port_baudrate = 0;
  ldlidar::LDType type_name;
  LaserScanSetting setting;
  setting.frame_id = "base_laser";
  setting.laser_scan_dir = true;
  setting.enable_angle_crop_func = false;
  setting.angle_crop_min = 0.0;
  setting.angle_crop_max = 0.0;
  setting.bins = 0;
  setting.enable_box_crop_func = false;
  setting.x_crop_min = 0.0;
  setting.x_crop_max = 0.0;
  setting.y_crop_min = 0.0;
  setting.y_crop_max = 0.0;
  setting.range_min = 0.03;
  setting.range_max = 25.0;
  
  // declare ros2 param
  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("topic_name", topic_name);
  node->declare_parameter<std::string>("frame_id", setting.frame_id);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<int>("port_baudrate", serial_port_baudrate);
  node->declare_parameter<bool>("laser_scan_dir", setting.laser_scan_dir);
  node->declare_parameter<bool>("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->declare_parameter<double>("angle_crop_min", setting.angle_crop_min);
  node->declare_parameter<double>("angle_crop_max", setting.angle_crop_max);
  node->declare_parameter<int>("bins", setting.bins);
  node->declare_parameter<bool>("enable_box_crop_func", setting.enable_box_crop_func);
  node->declare_parameter<double>("x_crop_min", setting.x_crop_min);
  node->declare_parameter<double>("x_crop_max", setting.x_crop_max);
  node->declare_parameter<double>("y_crop_min", setting.y_crop_min);
  node->declare_parameter<double>("y_crop_max", setting.y_crop_max);
  node->declare_parameter<double>("range_min", setting.range_min);
  node->declare_parameter<double>("range_max", setting.range_max);

  // get ros2 param
  node->get_parameter("product_name", product_name);
  node->get_parameter("topic_name", topic_name);
  node->get_parameter("frame_id", setting.frame_id);
  node->get_parameter("port_name", port_name);
  node->get_parameter("port_baudrate", serial_port_baudrate);
  node->get_parameter("laser_scan_dir", setting.laser_scan_dir);
  node->get_parameter("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->get_parameter("angle_crop_min", setting.angle_crop_min);
  node->get_parameter("angle_crop_max", setting.angle_crop_max);
  node->get_parameter("bins", setting.bins);
  node->get_parameter("enable_box_crop_func", setting.enable_box_crop_func);
  node->get_parameter("x_crop_min", setting.x_crop_min);
  node->get_parameter("x_crop_max", setting.x_crop_max);
  node->get_parameter("y_crop_min", setting.y_crop_min);
  node->get_parameter("y_crop_max", setting.y_crop_max);
  node->get_parameter("range_min", setting.range_min);
  node->get_parameter("range_max", setting.range_max);

  ldlidar::LDLidarDriver* ldlidarnode = new ldlidar::LDLidarDriver();

  RCLCPP_INFO(node->get_logger(), "LDLiDAR SDK Pack Version is: %s", ldlidarnode->GetLidarSdkVersionNumber().c_str());
  RCLCPP_INFO(node->get_logger(), "<product_name>: %s", product_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<topic_name>: %s", topic_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<frame_id>: %s", setting.frame_id.c_str());
  RCLCPP_INFO(node->get_logger(), "<port_name>: %s", port_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<port_baudrate>: %d", serial_port_baudrate);
  RCLCPP_INFO(node->get_logger(), "<laser_scan_dir>: %s", (setting.laser_scan_dir?"Counterclockwise":"Clockwise"));
  RCLCPP_INFO(node->get_logger(), "<enable_angle_crop_func>: %s", (setting.enable_angle_crop_func?"true":"false"));
  RCLCPP_INFO(node->get_logger(), "<angle_crop_min>: %f", setting.angle_crop_min);
  RCLCPP_INFO(node->get_logger(), "<angle_crop_max>: %f", setting.angle_crop_max);
  RCLCPP_INFO(node->get_logger(), "<bins>: %d", setting.bins);
  RCLCPP_INFO(node->get_logger(), "<enable_box_crop_func>: %s", (setting.enable_box_crop_func?"true":"false"));
  RCLCPP_INFO(node->get_logger(), "<x_crop_min>: %f", setting.x_crop_min);
  RCLCPP_INFO(node->get_logger(), "<x_crop_max>: %f", setting.x_crop_max);
  RCLCPP_INFO(node->get_logger(), "<y_crop_min>: %f", setting.y_crop_min);
  RCLCPP_INFO(node->get_logger(), "<y_crop_max>: %f", setting.y_crop_max);

  if (setting.bins > 0 && setting.bins < 10) {
    RCLCPP_INFO(node->get_logger(), "recommend increasing bin number");
  }

  if (product_name == "LDLiDAR_LD06") {
    type_name = ldlidar::LDType::LD_06;
  } else if (product_name == "LDLiDAR_LD19") {
    type_name = ldlidar::LDType::LD_19;
  } else if (product_name == "LDLiDAR_STL27L") {
    type_name = ldlidar::LDType::STL_27L;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Error, input <product_name> is illegal.");
    exit(EXIT_FAILURE);
  }

  ldlidarnode->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); 

  ldlidarnode->EnableFilterAlgorithnmProcess(true);

  if (ldlidarnode->Start(type_name, port_name, serial_port_baudrate, ldlidar::COMM_SERIAL_MODE)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar node start is success");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar node start is fail");
    exit(EXIT_FAILURE);
  }

  if (ldlidarnode->WaitLidarCommConnect(3000)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar communication is normal.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }

  // create ldlidar data topic and publisher
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher = 
      node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
  
  rclcpp::WallRate r(60); // 60hz to drain buffer faster (LiDAR is 10Hz)

  ldlidar::LaserScan laser_scan;
  double lidar_scan_freq;
  RCLCPP_INFO(node->get_logger(), "Publish topic message:ldlidar scan data.");
  while (rclcpp::ok()) {
    switch (ldlidarnode->GetLaserScanData(laser_scan, 1500)){
      case ldlidar::LidarStatus::NORMAL: 
        ldlidarnode->GetLidarScanFreq(lidar_scan_freq);
        ToLaserscanMessagePublish(laser_scan, lidar_scan_freq, setting, node, publisher);
        break;
      case ldlidar::LidarStatus::DATA_TIME_OUT:
        RCLCPP_ERROR(node->get_logger(), "get ldlidar data is time out, please check your lidar device.");
        break;
      case ldlidar::LidarStatus::DATA_WAIT:
        break;
      default:
        break;
    }

    r.sleep();
  }

  ldlidarnode->Stop();

  delete ldlidarnode;
  ldlidarnode = nullptr;

  RCLCPP_INFO(node->get_logger(), "ldlidar_published is end");
  rclcpp::shutdown();

  return 0;
}

void  ToLaserscanMessagePublish(ldlidar::LaserScan& src,  double lidar_spin_freq, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub) {
  // Rotation-model measurement-time reconstruction (BUG-0050 / Story 059.3): the
  // frame stamp and per-point timing derive from the LiDAR's reported spin rate,
  // not serial-arrival time, so a batched read() that collapses per-packet arrival
  // stamps cannot reach the published scan. State persists across frames.
  static ldlidar::ScanStartReconstructor scan_start_recon;

  const int beam_size = (setting.bins > 0)
    ? setting.bins
    : static_cast<int>(src.points.size());

  // NFF: no plausible rotation model for this frame -> log and skip, never present
  // a serial-arrival stamp as measurement time.
  if (beam_size <= 1 || lidar_spin_freq <= 0.0) {
    RCLCPP_WARN(node->get_logger(),
      "ldlidar: no rotation model (beam_size=%d, spin=%.3f Hz); skipping frame",
      beam_size, lidar_spin_freq);
    return;
  }
  const double scan_time = 1.0 / lidar_spin_freq;  // one rotation period (s)
  if (scan_time < 0.02 || scan_time > 0.5) {        // implausible outside ~2..50 Hz
    RCLCPP_WARN(node->get_logger(),
      "ldlidar: implausible spin %.2f Hz (period %.3f s); skipping frame",
      lidar_spin_freq, scan_time);
    return;
  }

  // Frame stamp: a fresh CLOCK_REALTIME sample at publish, reconstructed against the
  // rotation cadence (immune to batched-read collapse; see scan_time_reconstruct.h).
  const int64_t period_ns = static_cast<int64_t>(scan_time * 1e9);
  const int64_t stamp_ns = scan_start_recon.Update(
    static_cast<int64_t>(GetSystemTimeStamp()), period_ns);

  // Encoding (b): forward measurement order + non-negative time_increment; the
  // laser_scan_dir counterclockwise reflection is carried by the signed angle
  // fields (a consistent negative-increment triple for rtabmap's guard).
  const ldlidar::ScanAngleTiming timing =
    ldlidar::ComputeScanAngleTiming(beam_size, scan_time, setting.laser_scan_dir);

  sensor_msgs::msg::LaserScan output;
  output.header.stamp = rclcpp::Time(stamp_ns);
  output.header.frame_id = setting.frame_id;
  output.angle_min = timing.angle_min;
  output.angle_max = timing.angle_max;
  output.angle_increment = timing.angle_increment;
  output.time_increment = timing.time_increment;
  output.scan_time = timing.scan_time;
  output.range_min = setting.range_min;
  output.range_max = setting.range_max;
  // First fill all the data with Nan
  output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
  output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

  // Bin index from the real (unsigned) angular step, decoupled from the published
  // (possibly negative) angle_increment: output[index] holds the index-th-measured
  // point -> array order == measurement/sweep order (encoding (b)).
  const float bin_increment = static_cast<float>(2.0 * M_PI) / static_cast<float>(beam_size - 1);

  for (auto point : src.points) {
    float range = point.distance / 1000.f;  // distance unit transform to meters
    float intensity = point.intensity;      // laser receive intensity
    float dir_angle = point.angle;

    if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
      range = std::numeric_limits<float>::quiet_NaN();
      intensity = std::numeric_limits<float>::quiet_NaN();
    }

    if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
      if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
        range = std::numeric_limits<float>::quiet_NaN();
        intensity = std::numeric_limits<float>::quiet_NaN();
      }
    }

    if (setting.enable_box_crop_func) { // Box crop setting, Mask data within the box
      double x = range*cos(dir_angle*M_PI/180);
      double y = -range*sin(dir_angle*M_PI/180);
      if (x > setting.x_crop_min && x < setting.x_crop_max &&
          y > setting.y_crop_min && y < setting.y_crop_max) {
        range = std::numeric_limits<float>::quiet_NaN();
        intensity = std::numeric_limits<float>::quiet_NaN();
      }
    }

    float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
    int index = static_cast<int>(ceil(angle / bin_increment));
    if (index >= 0 && index < beam_size) {
      // Keep the nearest return when multiple points bin to one index.
      if (std::isnan(output.ranges[index]) || range < output.ranges[index]) {
        output.ranges[index] = range;
      }
      output.intensities[index] = intensity;
    } else if (index < 0) {
      RCLCPP_ERROR(node->get_logger(),
        "error index: %d, beam_size: %d, angle: %f, bin_increment: %f",
        index, beam_size, angle, bin_increment);
    }
  }
  lidarpub->publish(output);
}

uint64_t GetSystemTimeStamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
