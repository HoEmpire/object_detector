#pragma once
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>

const float PI = 3.1415926535;
using namespace Eigen;
using namespace std;
using namespace std::chrono;

Vector3f rotationMatrixToEulerAngles(Matrix3f R)
{
  float sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

  bool singular = sy < 1e-6;  // If

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
  Eigen::Matrix4f extrinsic_matrix;
  Eigen::Matrix3f camera_matrix;
  double k1, k2, k3, p1, p2;
  string pkg_loc;
  float prob_threshold;

  void print()
  {
    cout << "Extrinsic matrix: \n" << extrinsic_matrix << endl;
    cout << "Camera matrix: \n" << camera_matrix << endl;
    cout << "Distortion coeff: \n" << k1 << " " << k2 << " " << k3 << " " << p1 << " " << p2 << endl;
  }
} config;

struct timer
{
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
  config.print();
}

void loadConfig(ros::NodeHandle n)
{
  n.getParam("/prob_threshold", config.prob_threshold);
}

void calPointsNum(pcl::PointCloud<pcl::PointXYZI> point_cloud, const vector<cv::Rect> boxs, vector<int> &points_count)
{
  Eigen::Vector4f p;
  float fx, fy, cx, cy;
  fx = config.camera_matrix(0, 0);
  fy = config.camera_matrix(1, 1);
  cx = config.camera_matrix(0, 2);
  cy = config.camera_matrix(1, 2);

  // initialize counts
  for (auto &psc : points_count)
    psc = 0;

  for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
  {
    p << 0, 0, 0, 1;
    p(0) = pt->x;
    p(1) = pt->y;
    p(2) = pt->z;
    if (p(2) == 0)
      continue;

    int x = int(p(0) / p(2) * fx + cx);
    int y = int(p(1) / p(2) * fy + cy);
    for (int i = 0; i < boxs.size(); i++)
    {
      if (x > boxs[i].x && x < (boxs[i].x + boxs[i].width) && y > boxs[i].y && y < (boxs[i].y + boxs[i].height))
        points_count[i]++;
    }
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
