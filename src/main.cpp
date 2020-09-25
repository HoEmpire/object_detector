#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "usfs_bridge/reporter/lidar_camera_reporter.h"
#include "usfs_inference/component/object_detection_component.h"
#include "usfs_inference/detector/yolo_object_detector.h"
#include "util.h"

using namespace std;
using namespace Eigen;

usfs::inference::YoloObjectDetector detector;
float platform_yaw = 0.0;
// class LCDetector
// {

// };

// void callback(const sensor_msgs::PointCloud2ConstPtr &msg_pc, const sensor_msgs::ImageConstPtr &msg_img,
//               ros::Publisher marker_pub, usfs::bridge::LidarCameraReporter reporter)
void callback(const sensor_msgs::PointCloud2ConstPtr &msg_pc, const sensor_msgs::ImageConstPtr &msg_img,
              ros::Publisher marker_pub)
{
  static int id = 0;
  pcl::PointCloud<pcl::PointXYZI> point_cloud_livox;
  pcl::fromROSMsg(*msg_pc, point_cloud_livox);
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
  cv::Mat image_undistorted;
  undistortImage(cv_ptr->image, image_undistorted);

  std::vector<usfs::inference::ObjectDetectionResult> results;
  std::vector<int> points_count;
  std::vector<visualization_msgs::Marker> markers;
  std::vector<cv::Rect> boxs;
  // object detection
  if (config.detection_mode == "camera_first")
  {
    detector.Detect(image_undistorted, results);
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
    std::vector<int> points_count_tmp;
    std::vector<visualization_msgs::Marker> markers_tmp;
    std::vector<std::vector<float>> lidar_box;
    pointCloudClustering(point_cloud_livox, points_count, markers, lidar_box);
    for (uint i = 0; i < points_count.size(); i++)
    {
      float r =
          sqrt((markers[i].points[0].x * markers[i].points[0].x + markers[i].points[0].y * markers[i].points[0].y +
                markers[i].points[0].z + markers[i].points[0].z));
      float pts_num_min = config.points_number_min / (r / config.points_number_distance_coeff);
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
    detector.Detect(image_undistorted, results);

    if (results.size() == 0)
    {
      ROS_INFO_STREAM("Nothing detected!");
    }

    for (const auto &result : results)
    {
      if (result.type == 0)
      {
        // std::cout << result.type << ": " << result.prob << std::endl;
        // boxs.push_back(result.bbox);
        // std::cout << "x: " << result.bbox.x << ", y: " << result.bbox.x << std::endl;
        // std::cout << "width: " << result.bbox.width << ", y: " << result.bbox.height << std::endl;
        // std::cout << "x_center:" << result.bbox.x + 0.5 * result.bbox.width
        //           << ", y_center:" << result.bbox.y + 0.5 * result.bbox.height << std::endl;
        float y_center = -(result.bbox.x + 0.5 * result.bbox.width - 0.5 * 1440);
        float z_center = -(result.bbox.y + 0.5 * result.bbox.height - 0.5 * 1080);
        Vector2f object_img_direction_vector;
        Vector2f object_pc_direction_vector;
        object_img_direction_vector << y_center, z_center;
        object_img_direction_vector.normalize();  // TODO might have bugs in here

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
          transformMarkerCoordinate(markers[index], platform_yaw);
          marker_pub.publish(markers[index]);

          objectType object_info;
          object_info.id = id++;
          object_info.target_pcl_num = points_count[index];
          object_info.x = markers[index].points[0].x;
          object_info.y = markers[index].points[0].y;
          object_info.z = markers[index].points[0].z;
          object_info.coordinate_transform();

          object_info.lidar_box.assign(lidar_box[index].begin(), lidar_box[index].end());
          // TODO add info from vision
          if (result.type == 0)  // TODO update here later
          {
            object_info.type = result.type;
            object_info.type_reliability = result.prob;
            usfs::inference::ColorDetectionResult color_result;
            std::shared_ptr<usfs::inference::HsvColorClassifier> color_classifier;
            color_classifier->Classify(image_undistorted, color_result, result);
            object_info.color = color_result.color;
            object_info.color_reliability = color_result.prob;
          }
          else
          {
            object_info.color = 0;  // TODO hardcode in here
            object_info.color_reliability = 0.0;
          }
          object_info.print();
          // reporter.publish(ros::Time::now(), object_info.id, object_info.distance, object_info.angle,
          // object_info.type,
          //                  object_info.type_reliability, object_info.target_pcl_num, object_info.color,
          //                  object_info.color_reliability, object_info.lidar_box);
        }
      }
    }
  }
}

void platformCallBack(const std_msgs::Float32 &yaw)
{
  platform_yaw = yaw.data;
}

int main(int argc, char **argv)
{
  readConfig();
  ros::init(argc, argv, "pointcloud_fusion");

  ros::NodeHandle n;
  string lidar_topic, camera_topic;
  n.getParam("/data_topic/lidar_topic", lidar_topic);
  n.getParam("/data_topic/camera_topic", camera_topic);
  loadConfig(n);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  // detector
  Config config_cam;
  std::string package_path = ros::package::getPath("usfs_inference");
  config_cam.net_type = YOLOV4_TINY;
  config_cam.file_model_cfg = package_path + "/asset/yolov4-tiny.cfg";
  config_cam.file_model_weights = package_path + "/asset/yolov4-tiny.weights";
  config_cam.inference_precison = FP16;  // use FP16 for Jetson Xavier NX
  config_cam.n_max_batch = 1;
  config_cam.detect_thresh = config.prob_threshold;
  config_cam.min_width = 0;
  config_cam.max_width = 1440;
  config_cam.min_height = 0;
  config_cam.max_height = 1080;
  detector.Init(config_cam);

  // usfs::bridge::LidarCameraReporter reporter(&n);
  // reporter.Init();

  ros::Subscriber platform_yaw_sub = n.subscribe("/platform_driver/platform_yaw", 10, &platformCallBack);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, lidar_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> camera_sub(n, camera_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, camera_sub);
  // sync.registerCallback(boost::bind(&callback, _1, _2, marker_pub, reporter));
  sync.registerCallback(boost::bind(&callback, _1, _2, marker_pub));
  ros::spin();

  return EXIT_SUCCESS;
}
