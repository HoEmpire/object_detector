#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt16.h>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "object_detector/include/util.h"
#include "usfs_inference/detector/yolo_object_detector.h"

using namespace std;
using namespace Eigen;

usfs::inference::YoloObjectDetector detector;

void callback(const sensor_msgs::PointCloud2ConstPtr &msg_pc, const sensor_msgs::ImageConstPtr &msg_img)
{
  pcl::PointCloud<pcl::PointXYZI> point_cloud_livox;
  pcl::fromROSMsg(*msg_pc, point_cloud_livox);

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);

  cv::Mat image_undistorted;
  undistortImage(cv_ptr->image, image_undistorted);

  std::vector<usfs::inference::ObjectDetectionResult> results;
  std::vector<int> points_count;
  std::vector<cv::Rect> boxs;
  detector.Detect(image_undistorted, results);
  for (const auto &result : results)
  {
    std::cout << result.type << ": " << result.prob << std::endl;
    boxs.push_back(result.bbox);
    std::cout << "x: " << result.bbox.x << ", y: " << result.bbox.x << std::endl;
    std::cout << "width: " << result.bbox.width << ", y: " << result.bbox.height << std::endl;
  }
  calPointsNum(point_cloud_livox, boxs, points_count);
  for (int i = 0; i < boxs.size(); i++)
    std::cout << "Object id: " << i << ", number of 3d points: " << boxs[i] << std::endl;
}

int main(int argc, char **argv)
{
  readConfig();
  ros::init(argc, argv, "pointcloud_fusion");

  ros::NodeHandle n;
  string lidar_topic, camera_topic;
  n.getParam("/pointcloud_fusion/lidar_topic", lidar_topic);
  n.getParam("/pointcloud_fusion/camera_topic", camera_topic);
  loadConfig(n);

  // detector
  Config config;
  std::string package_path = ros::package::getPath("usfs_inference");
  config.net_type = YOLOV4_TINY;
  config.file_model_cfg = package_path + "/asset/yolov4-tiny.cfg";
  config.file_model_weights = package_path + "/asset/yolov4-tiny.weights";
  config.inference_precison = FP32;  // use FP16 for Jetson Xavier NX
  config.n_max_batch = 1;
  config.detect_thresh = 0.4;
  config.min_width = 50;
  config.max_width = 1920;
  config.min_height = 50;
  config.max_height = 1920;
  detector.Init(config);

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, lidar_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> camera_sub(n, camera_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, camera_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();

  return EXIT_SUCCESS;
}
