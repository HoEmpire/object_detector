#include <string.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16.h>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

#include "object_detector/util.h"
#include <platform_driver/command.h>
#include "usfs_common/frame/object_type.h"
#include "usfs_common/InsSensorData.h"

#define DEFINE_WAMP 0

#include "usfs_example/lidar_camera_adapter.h"
#include "usfs_bridge/driver/ins_driver.h"

using namespace std;
using namespace Eigen;

class LCDetector
{
public:
  LCDetector(ros::NodeHandle *nh);
#if DEFINE_WAMP
  void detection_callback(const sensor_msgs::PointCloud2ConstPtr &msg_pc, const sensor_msgs::ImageConstPtr &msg_img);
#else
  void detection_callback(const sensor_msgs::PointCloud2ConstPtr &msg_pc, const sensor_msgs::CompressedImageConstPtr &msg_img);
#endif
  void platform_callback(const std_msgs::Float32MultiArray &rotation);
  void ins_callback(const usfs_common::InsSensorData &msg);
  bool WampMsgCallback(wampsdk::wsession &ws, const std::string &topic,
                       const wampcc::json_object &data);
  float platform_roll,
      platform_pitch, platform_yaw;
  float ins_yaw;

private:
  trackCommandType track_info;
  platform_driver::command cmd_msg;
  ros::NodeHandle *nh_;
  usfs::inference::YoloObjectDetector detector;
  ros::Publisher marker_pub;
  ros::Publisher platform_command_pub;
  ros::Subscriber platform_yaw_sub;
  ros::Subscriber ins_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
#if DEFINE_WAMP
  message_filters::Subscriber<sensor_msgs::Image> camera_sub;
#else
  message_filters::Subscriber<sensor_msgs::CompressedImage> camera_sub;
#endif

  ros::Publisher pcl_filter_debug;
  ros::Publisher cam_detection_debug;

#if DEFINE_WAMP
  usfs::bridge::LidarCameraAdapter *adapter;
  usfs::bridge::InsDriver *driver_ins;
#endif
};

LCDetector::LCDetector(ros::NodeHandle *nh)
{
  nh_ = nh;
  // detector
  Config config_cam;
  std::string package_path = ros::package::getPath("usfs_inference");
  config_cam.net_type = config.net_type_table[config.cam_net_type];
  config_cam.file_model_cfg = package_path + config.cam_file_model_cfg;
  config_cam.file_model_weights = package_path + config.cam_file_model_weights;
  config_cam.inference_precison = config.precision_table[config.cam_inference_precison]; // use FP16 for Jetson Xavier NX
  config_cam.n_max_batch = config.cam_n_max_batch;
  config_cam.detect_thresh = config.cam_prob_threshold;
  config_cam.min_width = config.cam_min_width;
  config_cam.max_width = config.cam_max_width;
  config_cam.min_height = config.cam_min_height;
  config_cam.max_height = config.cam_max_height;
  detector.Init(config_cam);

  platform_roll = 0.0;
  platform_pitch = 0.0;
  platform_yaw = 0.0;
  ins_yaw = 0.0;

  marker_pub = nh_->advertise<visualization_msgs::Marker>("visualization_marker", 1);
  platform_command_pub = nh_->advertise<platform_driver::command>("write", 1);
  pcl_filter_debug = nh_->advertise<sensor_msgs::PointCloud2>("pc_filtered", 1);
  cam_detection_debug = nh_->advertise<sensor_msgs::Image>("cam_detection", 1);
  platform_yaw_sub = nh_->subscribe("/platform_driver/platform_rotation", 1, &LCDetector::platform_callback, this);
  ins_sub = nh_->subscribe("/usfs/sensor/ins", 1, &LCDetector::ins_callback, this);
  cloud_sub.subscribe(*nh_, config.lidar_topic, 1);
  camera_sub.subscribe(*nh_, config.camera_topic, 1);

#if DEFINE_WAMP
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
#else
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CompressedImage> MySyncPolicy;
#endif
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, camera_sub);
  sync.registerCallback(boost::bind(&LCDetector::detection_callback, this, _1, _2));

#if DEFINE_WAMP
  adapter = new usfs::bridge::LidarCameraAdapter(nh_);
  driver_ins = new usfs::bridge::InsDriver(nh_);
  // ROS_INFO("FUCK0");
  adapter->Init();
  driver_ins->Init();
  driver_ins->Start();
  // ROS_INFO("FUCK1");
  adapter->subscribe(boost::bind(&LCDetector::WampMsgCallback, this, _1, _2, _3));
  // ROS_INFO("FUCK2");
#endif

  ros::Rate r(20);
  while (ros::ok())
  {
    if (track_info.timeout.toc() > config.track_timeout)
    {
      if (track_info.have_sent_scan_command == false)
        ROS_INFO("Sending scan command!!!");
      cmd_msg.mode = 1; //scanning
      cmd_msg.scan_cycle_time = config.scan_cycle_time;
      cmd_msg.scan_range = config.scan_range;
      platform_command_pub.publish(cmd_msg);
      track_info.have_sent_scan_command = true;
    }
    ros::spinOnce();
    r.sleep();
  }
}

#if DEFINE_WAMP
void LCDetector::detection_callback(const sensor_msgs::PointCloud2ConstPtr &msg_pc, const sensor_msgs::ImageConstPtr &msg_img)
#else
void LCDetector::detection_callback(const sensor_msgs::PointCloud2ConstPtr &msg_pc, const sensor_msgs::CompressedImageConstPtr &msg_img)
#endif
{
  // ROS_INFO("into callback");

  int id = 0;
  timer a;
  a.tic();
  pcl::PointCloud<pcl::PointXYZI> point_cloud_livox, pc_filtered;
  pcl::fromROSMsg(*msg_pc, point_cloud_livox);
  cv_bridge::CvImagePtr cv_ptr, image_detection_result_ros;
  cv::Mat image_detection_result;
  ros::Time time = msg_pc->header.stamp;

  cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
  // undistortImage(cv_ptr->image, image_undistorted);
  image_detection_result = cv_ptr->image.clone();
  // ROS_INFO_STREAM("Data pre process takes " << a.toc() << " seconds");

  std::vector<usfs::inference::ObjectDetectionResult> results;
  std::vector<int> points_count;
  std::vector<visualization_msgs::Marker> markers;
  std::vector<cv::Rect> boxs;
  // object detection
  if (config.detection_mode == "camera_first")
  {
    detector.Detect(image_detection_result, results);
    for (const auto &result : results)
    {
      if (result.type == 0)
      {
        std::cout << result.type << ": " << result.prob << std::endl;
        boxs.push_back(result.bbox);
        std::cout << "x: " << result.bbox.x << ", y: " << result.bbox.x << std::endl;
        std::cout << "width: " << result.bbox.width << ", y: " << result.bbox.height << std::endl;
      }
    }
    calPointsNum(point_cloud_livox, boxs, points_count, markers);
    for (uint i = 0; i < points_count.size(); i++)
    {
      std::cout << "number of 3d points: " << points_count[i] << std::endl;
      std::cout << "positon:" << std::endl;
      std::cout << "x: " << markers[i].points[0].x << std::endl;
      std::cout << "y: " << markers[i].points[0].y << std::endl;
      std::cout << "z: " << markers[i].points[0].z << std::endl;
      marker_pub.publish(markers[i]);
    }
  }
  else if (config.detection_mode == "lidar_first")
  {
    //point cloud clustering
    std::vector<int> points_count_tmp;
    std::vector<visualization_msgs::Marker> markers_tmp;
    std::vector<std::vector<float>> lidar_box;
    a.tic();
    pointCloudClustering(point_cloud_livox, points_count, markers, lidar_box, pc_filtered);
    // ROS_INFO_STREAM("Filtering and Clustering takes " << a.toc() << " seconds");

    pcl::PointCloud<pcl::PointXYZI> pc_filtered_boat_frame;

    Eigen::Affine3f T = Eigen::Affine3f::Identity();
    T.rotate(Eigen::AngleAxisf(platform_yaw / 180 * M_PI, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(pc_filtered, pc_filtered_boat_frame, T);

    pc_filtered_boat_frame.header.frame_id = "livox_frame";
    pcl_filter_debug.publish(pc_filtered_boat_frame);

    for (uint i = 0; i < points_count.size(); i++)
    {
      float r =
          sqrt((markers[i].points[0].x * markers[i].points[0].x + markers[i].points[0].y * markers[i].points[0].y +
                markers[i].points[0].z + markers[i].points[0].z));
      float pts_num_min = config.points_number_min / (r * r / (config.points_number_distance_coeff * config.points_number_distance_coeff));
      float pts_num_max = config.points_number_max / (r / config.points_number_distance_coeff);
      if (points_count[i] > pts_num_min && points_count[i] < pts_num_max)
      {
        points_count_tmp.push_back(points_count[i]);
        markers_tmp.push_back(markers[i]);
        ROS_INFO_STREAM("Found an interesting 3d target!");
        ROS_INFO_STREAM("Points number: " << points_count[i]);
        ROS_INFO_STREAM("Target distance: " << r);
        ROS_INFO_STREAM("Target position: " << markers[i].points[0].x << ", " << markers[i].points[0].y << ", "
                                            << markers[i].points[0].z);
      }
    }

    if (points_count_tmp.size() == 0)
      return;
    points_count = points_count_tmp;
    markers = markers_tmp;

    // image detection
    ROS_INFO_STREAM("Checking with image...");
    detector.Detect(cv_ptr->image, results);

    if (results.size() == 0)
    {
      ROS_INFO_STREAM("Nothing detected!");
#if DEFINE_WAMP
      if (track_info.have_sent_scan_command == false)
      {
        ROS_INFO_STREAM("Sending lidar detection informations in Targeting mode!");
        for (int i = 0; i < points_count.size(); i++)
        {
          visualization_msgs::Marker marker_boat_frame;
          transformMarkerCoordinate(markers[i], platform_roll, platform_pitch, platform_yaw, marker_boat_frame);
          marker_pub.publish(marker_boat_frame);

          objectType object_info;
          object_info.id = id++;
          object_info.target_pcl_num = points_count[i];
          object_info.x = marker_boat_frame.points[0].x;
          object_info.y = marker_boat_frame.points[0].y;
          object_info.z = marker_boat_frame.points[0].z;
          object_info.set_offset(config.extrinsic_offset);
          object_info.coordinate_transform();
          object_info.lidar_box.assign(lidar_box[i].begin(), lidar_box[i].end());
          object_info.type = static_cast<int>(usfs::common::ObjectType::UNKNOWN);
          object_info.type_reliability = 0.0;
          object_info.color = static_cast<int>(usfs::common::BuoyType::UNKNOWN);
          object_info.color_reliability = 0.0;
          object_info.print();
          adapter->publish(time, object_info.id, object_info.distance, object_info.angle,
                           object_info.type,
                           object_info.type_reliability, object_info.target_pcl_num, object_info.color,
                           object_info.color_reliability, object_info.lidar_box, !track_info.have_sent_scan_command);
        }
      }
#endif
      cout << endl;
    }
    else
    {
      for (const auto &result : results)
      {
        if (result.type == static_cast<int>(usfs::common::ObjectType::UNKNOWN) || result.type == static_cast<int>(usfs::common::ObjectType::SHIP) && result.bbox.height > config.boat_pixel_height || result.type == static_cast<int>(usfs::common::ObjectType::BUOY))
        {
          //draw result
          ROS_INFO_STREAM("type: " << result.type << " detected!");
          cv::rectangle(image_detection_result, result.bbox, cv::Scalar(255, 0, 0));
          string prob_str = to_string(result.prob);
          string text;
          cv::Scalar draw_color;
          if (result.type == static_cast<int>(usfs::common::ObjectType::UNKNOWN))
          {
            text = "unknown: " + prob_str;
            draw_color = cv::Scalar(255, 0, 0);
          }
          else if (result.type == static_cast<int>(usfs::common::ObjectType::SHIP))
          {
            text = "ship: " + prob_str;
            draw_color = cv::Scalar(0, 255, 0);
          }
          else if (result.type == static_cast<int>(usfs::common::ObjectType::BUOY))
          {
            text = "buoy: " + prob_str;
            draw_color = cv::Scalar(0, 0, 255);
          }
          cv::rectangle(image_detection_result, result.bbox, draw_color, 5.0);
          cv::putText(image_detection_result, text, cv::Point(result.bbox.x, result.bbox.y), cv::FONT_HERSHEY_COMPLEX, 1.5, draw_color, 6.0);
          float y_center = -(result.bbox.x + 0.5 * result.bbox.width - 0.5 * config.cam_max_width);
          float z_center = -(result.bbox.y + 0.5 * result.bbox.height - 0.5 * config.cam_max_height);
          Vector2f object_img_direction_vector;
          Vector2f object_pc_direction_vector;
          object_img_direction_vector << y_center, z_center;
          object_img_direction_vector.normalize();
          float distance_min = 1000000;
          int index = -1;
          for (uint i = 0; i < markers.size(); i++)
          {
            object_pc_direction_vector << markers[i].points[0].y, markers[i].points[0].y;
            if (object_pc_direction_vector.norm() < distance_min)
            {
              distance_min = object_pc_direction_vector.norm();
              index = i;
            }
          }

          if (index == -1)
            ROS_INFO("Error in matching points with image object");
          else
          {
            ROS_INFO("Succeed in matching points with image object");
            ROS_INFO_STREAM("Bbox size: x = " << result.bbox.width << ", y = " << result.bbox.height);
            visualization_msgs::Marker marker_boat_frame;
            transformMarkerCoordinate(markers[index], platform_roll, platform_pitch, platform_yaw, marker_boat_frame);
            marker_pub.publish(marker_boat_frame);

            objectType object_info;
            object_info.id = id++;
            object_info.target_pcl_num = points_count[index];
            object_info.x = marker_boat_frame.points[0].x;
            object_info.y = marker_boat_frame.points[0].y;
            object_info.z = marker_boat_frame.points[0].z;
            object_info.set_offset(config.extrinsic_offset);
            object_info.coordinate_transform();

            object_info.lidar_box.assign(lidar_box[index].begin(), lidar_box[index].end());
            object_info.type = result.type;
            object_info.type_reliability = result.prob;

            if (result.type == static_cast<int>(usfs::common::ObjectType::BUOY))
            {
              usfs::inference::ColorDetectionResult color_result;
              std::shared_ptr<usfs::inference::HsvColorClassifier> color_classifier;
              color_classifier->Classify(cv_ptr->image, color_result, result);
              object_info.color = color_result.color;
              object_info.color_reliability = color_result.prob;
            }
            else
            {
              object_info.color = static_cast<int>(usfs::common::BuoyType::UNKNOWN);
              object_info.color_reliability = 0.0;
            }
            object_info.print();
            sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_detection_result).toImageMsg();
            cam_detection_debug.publish(image_msg);

#if DEFINE_WAMP
            adapter->publish(time, object_info.id, object_info.distance, object_info.angle,
                             object_info.type,
                             object_info.type_reliability, object_info.target_pcl_num, object_info.color,
                             object_info.color_reliability, object_info.lidar_box, !track_info.have_sent_scan_command);
#endif
            markers.erase(markers.begin() + index); //remove the  point cloud that have been matched
          }
        }
      }
      cout << endl;
    }
  }
}

void LCDetector::platform_callback(const std_msgs::Float32MultiArray &rotation)
{
  platform_roll = rotation.data[0];
  platform_pitch = rotation.data[1];
  platform_yaw = rotation.data[2];
  static tf::TransformBroadcaster br;
  tf::Transform transform_world, transform_lidar;
  transform_world.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q_world, q_lidar;
  q_world.setRPY(0, 0, 0);
  transform_world.setRotation(q_world);
  br.sendTransform(tf::StampedTransform(transform_world, ros::Time::now(), "world", "livox_frame"));
}

void LCDetector::ins_callback(const usfs_common::InsSensorData &msg)
{
  if (msg.bow_angle < 180.0)
    ins_yaw = msg.bow_angle;
  else
    ins_yaw = msg.bow_angle - 360;
}

bool LCDetector::WampMsgCallback(wampsdk::wsession &ws, const std::string &topic,
                                 const wampcc::json_object &data)
{
  wampcc::json_object content;
  usfs::common::UnpackWampMsg(data, content);
  // auto distance = usfs::common::get_value<float>(content, "distance");
  track_info.command_angle = usfs::common::get_value<float>(content, "angle");
  if (track_info.command_angle > 180)
    track_info.command_angle -= 360;
  ROS_INFO_STREAM("Recevie tracking command, yaw = " << track_info.command_angle);
  track_info.timeout.tic();
  track_info.command_queue++;
  // ROS_INFO_STREAM(" Command queue = " << track_info.command_queue);
  if (track_info.command_queue != 0)
  {
    ROS_INFO("Sending track command!!!");
    track_info.command_queue--;
    cmd_msg.mode = 2; //tracking
    cmd_msg.track_yaw = -(track_info.command_angle - ins_yaw);
    if (abs(cmd_msg.track_yaw) < 85.0)
      platform_command_pub.publish(cmd_msg);
    else
    {
      cmd_msg.track_yaw = -(track_info.command_angle - ins_yaw) / abs(track_info.command_angle - ins_yaw) * 85.0;
      platform_command_pub.publish(cmd_msg);
    }
    track_info.have_sent_scan_command = false;
  }
  return true;
}

int main(int argc, char **argv)
{
  readConfig();
  ros::init(argc, argv, "pointcloud_fusion");

  ros::NodeHandle n;
  loadConfig(n);
  LCDetector detector(&n);
}
