#pragma once
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>

#include <visualization_msgs/Marker.h>

#include "usfs_inference/component/object_detection_component.h"
#include "usfs_inference/detector/yolo_object_detector.h"

const float PI = 3.1415926535;
using namespace Eigen;
using namespace std;
using namespace std::chrono;

enum class platformCommand
{
  SCANNING = 1,
  TRACKING = 2,
};

Vector3f rotationMatrixToEulerAngles(Matrix3f R)
{
  float sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

  bool singular = sy < 1e-6; // If

  float x, y, z;
  if (!singular)
  {
    x = atan2(R(2, 1), R(2, 2));
    y = atan2(-R(2, 0), sy);
    z = atan2(R(1, 0), R(0, 0));
  }
  else
  {
    x = atan2(-R(1, 2), R(1, 1));
    y = atan2(-R(2, 0), sy);
    z = 0;
  }
  Vector3f result;
  result << x, y, z;
  return result;
}

// config util
struct ConfigSetting
{
  std::unordered_map<std::string, ModelType> net_type_table = {{"YOLOV3", ModelType::YOLOV3}, {"YOLOV3_TINY", ModelType::YOLOV3_TINY}, {"YOLOV4", ModelType::YOLOV4}, {"YOLOV4_TINY", ModelType::YOLOV4_TINY}, {"YOLOV4", ModelType::YOLOV4}, {"YOLOV5", ModelType::YOLOV5}};
  std::unordered_map<std::string, Precision> precision_table = {{"INT8", Precision::INT8}, {"FP16", Precision::FP16}, {"FP32", Precision::FP32}};

  Eigen::Matrix4f extrinsic_matrix;
  Eigen::Matrix3f camera_matrix;
  double k1, k2, k3, p1, p2;
  string pkg_loc;
  float cam_prob_threshold;
  int filter_pt_num;
  float filter_std;
  string detection_mode;
  float filter_distance_min, filter_distance_max;
  float cluster_tolerance, cluster_size_min, cluster_size_max;
  float points_number_max, points_number_min, points_number_distance_coeff;

  string lidar_topic, camera_topic;
  string cam_net_type = "YOLOV4_TINY";
  string cam_file_model_cfg;
  string cam_file_model_weights;
  string cam_inference_precison;
  int cam_n_max_batch = 1;
  float cam_min_width = 0;
  float cam_max_width = 1440;
  float cam_min_height = 0;
  float cam_max_height = 1080;

  float scan_cycle_time = 20.0;
  float scan_range = 80.0;
  float track_timeout = 5.0;

  Matrix4f extrinsic_offset;

  void print()
  {
    cout << "Extrinsic matrix: \n"
         << extrinsic_matrix << endl;
    cout << "Camera matrix: \n"
         << camera_matrix << endl;
    cout << "Distortion coeff: \n"
         << k1 << " " << k2 << " " << k3 << " " << p1 << " " << p2 << endl;
    cout << "filter_pt_num: " << filter_pt_num << endl;
    cout << "filter_std: " << filter_std << endl;
    cout << "detection_mode: " << detection_mode << endl;
    cout << "filter_distance_min: " << filter_distance_min << endl;
    cout << "filter_distance_max: " << filter_distance_max << endl;
    cout << "cluster_tolerance: " << cluster_tolerance << endl;
    cout << "cluster_size_min: " << cluster_size_min << endl;
    cout << "cluster_size_max: " << cluster_size_max << endl;
    cout << "points_number_max: " << points_number_max << endl;
    cout << "points_number_min: " << points_number_min << endl;
    cout << "points_number_distance_coeff: " << points_number_distance_coeff << endl;
    cout << "lidar_topic: " << lidar_topic << endl;
    cout << "camera_topic: " << camera_topic << endl;
    cout << "cam_net_type: " << cam_net_type << endl;
    cout << "cam_file_model_cfg: " << cam_file_model_cfg << endl;
    cout << "cam_file_model_weights: " << cam_file_model_weights << endl;
    cout << "cam_inference_precison: " << cam_inference_precison << endl;
    cout << "cam_n_max_batch: " << cam_n_max_batch << endl;
    cout << "cam_prob_threshold: " << cam_prob_threshold << endl;
    cout << "cam_min_width: " << cam_min_width << endl;
    cout << "cam_max_width: " << cam_max_width << endl;
    cout << "cam_min_height: " << cam_min_height << endl;
    cout << "cam_max_height: " << cam_max_height << endl;

    cout << "scan_cycle_time: " << scan_cycle_time << endl;
    cout << "scan_range: " << scan_range << endl;
    cout << "track_timeout: " << track_timeout << endl;
  }
} config;

struct objectType
{
  int id = -1;
  float x, y, z;
  float angle, distance;
  int type = -1;
  float type_reliability;
  int target_pcl_num = -1;
  int color = -1;
  float color_reliability;
  vector<float> lidar_box;

  void coordinate_transform()
  {
    distance = sqrt(x * x + y * y);
    angle = float(atan2(y, x)) * 180 / M_PI; // in degree
  }

  void set_offset(Matrix4f T)
  {
    Vector4f p;
    p << x, y, z, 1.0;
    p = T * p;
    x = p(0);
    y = p(1);
    z = p(2);
  }

  void print()
  {
    cout << "id " << id << endl;
    cout << "x: " << x << ", y: " << y << ", z: " << z << endl;
    cout << "angle: " << angle << ", distance: " << distance << endl;
    cout << "type: " << type << endl;
    cout << "type_reliability: " << type_reliability << endl;
    cout << "target_pcl_num: " << target_pcl_num << endl;
    cout << "color: " << color << endl;
    cout << "color_reliability: " << color_reliability << endl;
    cout << "lidar_box_x: " << lidar_box[0] << ", lidar_box_y: " << lidar_box[1] << ", lidar_box_z: " << lidar_box[2]
         << endl;
  }
};

struct timer
{
  timer()
  {
    t_start = steady_clock::now();
  }
  steady_clock::time_point t_start, t_end;
  void tic()
  {
    t_start = steady_clock::now();
  }

  double toc()
  {
    t_end = steady_clock::now();
    return duration_cast<duration<double>>(t_end - t_start).count();
  }
};

struct trackCommandType
{
  int command_queue = 0;
  float command_angle = 0.0;
  timer timeout;
  bool have_sent_scan_command;
};

void readConfig()
{
  config.pkg_loc = ros::package::getPath("object_detector");
  ifstream infile(config.pkg_loc + "/config/config.txt");
  config.extrinsic_matrix.setIdentity(4, 4);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 4; j++)
      infile >> config.extrinsic_matrix(i, j);

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      infile >> config.camera_matrix(i, j);

  infile >> config.k1;
  infile >> config.k2;
  infile >> config.p1;
  infile >> config.p2;
  infile >> config.k3;

  infile.close();
}

void loadConfig(ros::NodeHandle n)
{
  n.getParam("/data_topic/lidar_topic", config.lidar_topic);
  n.getParam("/data_topic/camera_topic", config.camera_topic);

  n.getParam("/detection/mode", config.detection_mode);
  n.getParam("/detection/points_number_max", config.points_number_max);
  n.getParam("/detection/points_number_min", config.points_number_min);
  n.getParam("/detection/points_number_distance_coeff", config.points_number_distance_coeff);

  n.getParam("/filter/filter_pt_num", config.filter_pt_num);
  n.getParam("/filter/filter_std", config.filter_std);
  n.getParam("/filter/filter_distance_max", config.filter_distance_max);
  n.getParam("/filter/filter_distance_min", config.filter_distance_min);

  n.getParam("/cluster/cluster_tolerance", config.cluster_tolerance);
  n.getParam("/cluster/cluster_size_min", config.cluster_size_min);
  n.getParam("/cluster/cluster_size_max", config.cluster_size_max);

  n.getParam("/cam/cam_net_type", config.cam_net_type);
  n.getParam("/cam/cam_file_model_cfg", config.cam_file_model_cfg);
  n.getParam("/cam/cam_file_model_weights", config.cam_file_model_weights);
  n.getParam("/cam/cam_inference_precison", config.cam_inference_precison);
  n.getParam("/cam/cam_n_max_batch", config.cam_file_model_cfg);
  n.getParam("/cam/cam_prob_threshold", config.cam_prob_threshold);
  n.getParam("/cam/cam_min_width", config.cam_min_width);
  n.getParam("/cam/cam_max_width", config.cam_max_width);
  n.getParam("/cam/cam_min_height", config.cam_min_height);
  n.getParam("/cam/cam_max_height", config.cam_max_height);

  vector<float> translation;
  vector<float> rotation;
  n.getParam("/extrinsic_parameter/translation", translation);
  n.getParam("/extrinsic_parameter/rotation", rotation);
  AngleAxisf rollAngle(AngleAxisf(rotation[0], Vector3f::UnitX()));
  AngleAxisf pitchAngle(AngleAxisf(rotation[1], Vector3f::UnitY()));
  AngleAxisf yawAngle(AngleAxisf(rotation[2], Vector3f::UnitZ()));
  Matrix3f rotation_matrix;
  rotation_matrix = yawAngle * pitchAngle * rollAngle;
  config.extrinsic_offset.topLeftCorner(3, 3) = rotation_matrix;
  config.extrinsic_offset(3, 0) = translation[0];
  config.extrinsic_offset(3, 1) = translation[1];
  config.extrinsic_offset(3, 2) = translation[2];

  n.getParam("/platform/scan_cycle_time", config.scan_cycle_time);
  n.getParam("/platform/scan_range", config.scan_range);
  n.getParam("/platform/track_timeout", config.track_timeout);

  config.print();
}

void calPointsNum(pcl::PointCloud<pcl::PointXYZI> point_cloud, const vector<cv::Rect> boxs, vector<int> &points_count,
                  vector<visualization_msgs::Marker> &markers)
{
  Eigen::Vector4f p;
  float fx, fy, cx, cy;
  fx = config.camera_matrix(0, 0);
  fy = config.camera_matrix(1, 1);
  cx = config.camera_matrix(0, 2);
  cy = config.camera_matrix(1, 2);

  points_count.resize(boxs.size());
  // initialize counts

  vector<pcl::PointCloud<pcl::PointXYZI>> point_cloud_each_object;
  point_cloud_each_object.resize(boxs.size());
  for (auto &psc : points_count)
    psc = 0;

  for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
  {
    p << 0, 0, 0, 1;
    p(0) = pt->x;
    p(1) = pt->y;
    p(2) = pt->z;
    p = config.extrinsic_matrix * p;
    // ROS_INFO_STREAM("x:" << p(0));
    // ROS_INFO_STREAM("y:" << p(1));
    // ROS_INFO_STREAM("z:" << p(2));
    if (p(2) == 0)
      continue;

    int x = int(p(0) / p(2) * fx + cx);
    int y = int(p(1) / p(2) * fy + cy);
    for (uint i = 0; i < boxs.size(); i++)
    {
      // ROS_INFO_STREAM("x:" << x);
      // ROS_INFO_STREAM("y:" << y);
      if (x > boxs[i].x && x < (boxs[i].x + boxs[i].width) && y > boxs[i].y && y < (boxs[i].y + boxs[i].height))
      {
        points_count[i]++;
        point_cloud_each_object[i].push_back(*pt);
      }
    }
  }

  if (point_cloud_each_object.size() == 0)
    return;
  // filter
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statistical_filter;
  statistical_filter.setStddevMulThresh(config.filter_std);
  statistical_filter.setKeepOrganized(true);
  ROS_INFO("Start point cloud filtering!");
  for (uint i = 0; i < point_cloud_each_object.size(); i++)
  {
    statistical_filter.setInputCloud(point_cloud_each_object[i].makeShared());
    statistical_filter.filter(point_cloud_each_object[i]);
  }

  markers.resize(point_cloud_each_object.size());
  ROS_INFO("Start point cloud averaging!");
  for (uint i = 0; i < point_cloud_each_object.size(); i++)
  {
    float x, y, z;
    x = 0;
    y = 0;
    z = 0;

    markers[i].header.frame_id = "/livox_frame";
    markers[i].header.stamp = ros::Time::now();
    markers[i].ns = "points_and_lines";
    markers[i].action = visualization_msgs::Marker::ADD;
    markers[i].pose.orientation.w = 1.0;
    markers[i].scale.x = 0.2;
    markers[i].scale.y = 0.2;
    markers[i].color.r = 0.0;
    markers[i].color.a = 1.0;
    markers[i].color.g = 1.0;
    markers[i].color.b = 0.0;
    markers[i].type = visualization_msgs::Marker::POINTS;

    int pt_num = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
    {
      x += pt->x;
      y += pt->y;
      z += pt->z;
      pt_num++;
    }
    geometry_msgs::Point p;
    p.x = x / pt_num;
    p.y = y / pt_num;
    p.z = z / pt_num;
    markers[i].points.push_back(p);
  }
}

void pointCloudClustering(pcl::PointCloud<pcl::PointXYZI> point_cloud, vector<int> &points_count,
                          vector<visualization_msgs::Marker> &markers, vector<vector<float>> &lidar_box, pcl::PointCloud<pcl::PointXYZI> &cloud_filtered)
{
  // filter
  pcl::PassThrough<pcl::PointXYZI> pass;
  pcl::PointCloud<pcl::PointXYZI> cloud_filtered_pre;
  pass.setInputCloud(point_cloud.makeShared());                                 //设置输入点云
  pass.setFilterFieldName("x");                                                 //设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits(config.filter_distance_min, config.filter_distance_max); //设置在过滤字段的范围
  pass.setFilterLimitsNegative(false);                                          //设置保留范围内还是过滤掉范围内
  pass.filter(cloud_filtered_pre);                                              //执行滤波，保存过滤结果在cloud_filtered
  if (cloud_filtered_pre.size() == 0)
    return;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statistical_filter;
  statistical_filter.setStddevMulThresh(config.filter_std);
  statistical_filter.setMeanK(config.filter_pt_num);
  // statistical_filter.setKeepOrganized(true);
  statistical_filter.setInputCloud(cloud_filtered_pre.makeShared());
  statistical_filter.filter(cloud_filtered);

  if (cloud_filtered.size() == 0)
    return;
  // clustering
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_filtered.makeShared()); //创建点云索引向量，用于存储实际的点云信息
  vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(config.cluster_tolerance); //设置近邻搜索的搜索半径为2cm
  ec.setMinClusterSize(config.cluster_size_min);    //设置一个聚类需要的最少点数目为100
  ec.setMaxClusterSize(config.cluster_size_max);    //设置一个聚类需要的最大点数目为25000
  ec.setSearchMethod(tree);                         //设置点云的搜索机制
  ec.setInputCloud(cloud_filtered.makeShared());
  ec.extract(cluster_indices); //从点云中提取聚类，并将点云索引保存在cluster_indices中

  vector<pcl::PointCloud<pcl::PointXYZI>> cloud_clustered;
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud_cluster;
    //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cloud_cluster.points.push_back(cloud_filtered.points[*pit]);
      cloud_cluster.width = cloud_cluster.size();
      cloud_cluster.height = 1;
      cloud_cluster.is_dense = true;
    }
    cloud_clustered.push_back(cloud_cluster);
    points_count.push_back(cloud_cluster.size());
  }

  markers.resize(cloud_clustered.size());
  for (uint i = 0; i < cloud_clustered.size(); i++)
  {
    markers[i].header.frame_id = "/livox_frame";
    markers[i].header.stamp = ros::Time::now();
    markers[i].ns = "points_and_lines";
    markers[i].action = visualization_msgs::Marker::ADD;
    markers[i].pose.orientation.w = 1.0;
    markers[i].scale.x = 0.2;
    markers[i].scale.y = 0.2;
    markers[i].color.r = 1.0;
    markers[i].color.a = 1.0;
    markers[i].color.g = 0.0;
    markers[i].color.b = 0.0;
    markers[i].type = visualization_msgs::Marker::POINTS;

    int pt_num = 0;
    float x, y, z;
    x = 0;
    y = 0;
    z = 0;
    float x_min, x_max, y_min, y_max, z_min, z_max;
    for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = cloud_clustered[i].points.begin();
         pt < cloud_clustered[i].points.end(); pt++)
    {
      if (pt_num == 0)
      {
        x_min = pt->x;
        x_max = pt->x;
        y_min = pt->y;
        y_max = pt->y;
        z_min = pt->z;
        z_max = pt->z;
      }
      else
      {
        if (pt->x < x_min)
          x_min = pt->x;
        if (pt->x > x_max)
          x_max = pt->x;
        if (pt->y < y_min)
          y_min = pt->y;
        if (pt->y > y_max)
          y_max = pt->y;
        if (pt->z < z_min)
          z_min = pt->z;
        if (pt->z > z_max)
          z_max = pt->z;
      }
      x += pt->x;
      y += pt->y;
      z += pt->z;
      pt_num++;
    }
    geometry_msgs::Point p;
    p.x = x / pt_num;
    p.y = y / pt_num;
    p.z = z / pt_num;
    vector<float> lidar_box_tmp;
    lidar_box_tmp.push_back(x_max - x_min);
    lidar_box_tmp.push_back(y_max - y_min);
    lidar_box_tmp.push_back(z_max - z_min);
    lidar_box.push_back(lidar_box_tmp);
    markers[i].points.push_back(p);
  }
}

void undistortImage(const cv::Mat image_distorted, cv::Mat &image_undistorted)
{
  double fx, cx, fy, cy;
  fx = config.camera_matrix(0, 0);
  cx = config.camera_matrix(0, 2);
  fy = config.camera_matrix(1, 1);
  cy = config.camera_matrix(1, 2);
  double k1, k2, k3, p1, p2;
  k1 = config.k1;
  k2 = config.k2;
  k3 = config.k3;
  p1 = config.p1;
  p2 = config.p2;

  image_undistorted = cv::Mat::zeros(image_distorted.rows, image_distorted.cols, image_distorted.type());
  for (int v = 0; v < image_undistorted.rows; v++)
  {
    for (int u = 0; u < image_undistorted.cols; u++)
    {
      double x, y, x_distorted, y_distorted;
      x = (u - cx) / fx;
      y = (v - cy) / fy;

      double r2 = x * x + y * y;
      x_distorted = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2) + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
      y_distorted = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2) + 2 * p2 * x * y + p1 * (r2 + 2 * y * y);

      double u_distorted = fx * x_distorted + cx;
      double v_distorted = fy * y_distorted + cy;

      if (u_distorted >= 0 && u_distorted < image_distorted.cols && v_distorted >= 0 &&
          v_distorted < image_distorted.rows)
      {
        image_undistorted.at<uchar>(v, 3 * u) = image_distorted.at<uchar>(int(v_distorted), 3 * int(u_distorted));
        image_undistorted.at<uchar>(v, 3 * u + 1) =
            image_distorted.at<uchar>(int(v_distorted), 3 * int(u_distorted) + 1);
        image_undistorted.at<uchar>(v, 3 * u + 2) =
            image_distorted.at<uchar>(int(v_distorted), 3 * int(u_distorted) + 2);
      }
    }
  }
}

void transformMarkerCoordinate(visualization_msgs::Marker &marker, float yaw)
{
  marker.points[0].x = marker.points[0].x * cos(yaw / 180 * M_PI) - marker.points[0].y * sin(yaw / 180 * M_PI);
  marker.points[0].y = marker.points[0].x * sin(yaw / 180 * M_PI) + marker.points[0].y * cos(yaw / 180 * M_PI);
}