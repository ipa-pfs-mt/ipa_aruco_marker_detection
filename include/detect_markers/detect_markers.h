#ifndef DETECT_MARKERS_H
#define DETECT_MARKERS_H

// ros includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//opencv includes
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

//io
#include <iostream>
#include <sstream>
#include <fstream>

#include <librealsense2/rs.hpp>

//tf includes
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <math.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

class MarkerDetection
{

  ros::NodeHandle node_handle;
  image_transport::ImageTransport image_transport;
  image_transport::Subscriber image_sub;
  image_transport::Subscriber image_sub1;
  image_transport::Publisher image_pub;

 public:
  Mat camera_matrix = Mat::eye(3, 3, CV_64F);
  Mat distance_coefficients;

  //to be used with parameter server
  std::string topic_name, parent_joint_to_camera, camera_joint_name;
  int aruco_grid_board_markers_x, aruco_grid_board_markers_y, aruco_grid_board_first_marker;
  float aruco_square_size, aruco_grid_board_marker_lenght, aruco_grid_board_marker_separation;
  float transform_camera_holder_x, transform_camera_holder_y, transform_camera_holder_z,
  rotation_vector_camera_holder_x, rotation_vector_camera_holder_y, rotation_vector_camera_holder_z;

  MarkerDetection();
 ~MarkerDetection();

  void imageCb(const sensor_msgs::ImageConstPtr& msg);

  int estimatePoseBoard(Mat frame, const Mat& camera_matrix,
                        const Mat& distance_coefficients,
                        float aruco_square_dimensions, string window_name);

  void RemoveMarkersOutsideBoard(vector<int> marker_ids, vector<vector<Point2f>> marker_corners);

  void PoseCallback(Vec3d rotation_vectors, Vec3d translation_vectors);

  Vec3f rotationMatrixtoEulerAngles(Mat &rotation_matrix);


  bool loadCameraCalibration(string name, Mat& camera_matrix,
                             Mat& distance_coefficients);


  void getMarkerCoordinates(vector<Vec3d> rotation_vectors,
                            vector<Vec3d> translation_vectors,
                            vector<int> marker_ids);
  };





#endif // DETECT_MARKERS_H
