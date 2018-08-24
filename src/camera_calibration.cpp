#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

const float CALIBRATION_SQUARE_DIMENSION = 0.03f; // meters
const Size CHESSBOARD_DIMENSIONS =
    Size(7, 10); // always one less in x and y direction than actual board
static const std::string OPENCV_WINDOW = "Image window";

class CameraCalibration
{

  ros::NodeHandle node_handle;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

public:
  Mat camera_matrix = Mat::eye(3, 3, CV_64F);
  Mat distance_coefficients;
  vector<Mat> saved_images;
  int frames_per_second = 20;

  CameraCalibration() : it(node_handle)
  {

    image_sub = it.subscribe("/camera/color/image_raw", 1,
                             &CameraCalibration::imageCb, this);
    loadCameraCalibration("Camera Calib File", camera_matrix,
                          distance_coefficients);
  }
  ~CameraCalibration() {}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(
          msg, enc::BGR8); // cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    searchForCorners(cv_ptr->image);
  }
  void searchForCorners(Mat frame)
  {
    // Mat frame;

    Mat draw_to_frame;

    namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
    // cv_ptr->image.copyTo(frame);

    vector<Vec2f> found_points;
    bool found = false;

    found = findChessboardCorners(frame, CHESSBOARD_DIMENSIONS, found_points,
                                  CV_CALIB_CB_ADAPTIVE_THRESH |
                                      CV_CALIB_CB_NORMALIZE_IMAGE);
    frame.copyTo(draw_to_frame);
    drawChessboardCorners(draw_to_frame, CHESSBOARD_DIMENSIONS, found_points,
                          found);
    if (found)
      imshow("Webcam", draw_to_frame);
    else
      imshow("Webcam", frame);
    char character = waitKey(1000 / frames_per_second);

    switch (character)
    {
    case ' ':
      // saving image
      if (found)
      {
        cout << "found" << endl;
        Mat temp;
        frame.copyTo(temp);
        saved_images.push_back(temp);
        cout << saved_images.size() << endl;
      }

      break;
    case 13: // Enter
      // start calibration
      cout << "start calibration" << endl;
      if (saved_images.size() > 15)
      {
        CameraCalibration::cameraCalibration(
            saved_images, CHESSBOARD_DIMENSIONS, CALIBRATION_SQUARE_DIMENSION,
            camera_matrix, distance_coefficients);
        CameraCalibration::saveCameraCalibration(
            "Camera Calib File", camera_matrix, distance_coefficients);
        cout << "camera calibration saved" << endl;
      }
      break;
    case 27: // Esc
      // exit

      break;
    }
  }

  void createKnownBoardPosition(Size board_size, float square_edge_length,
                                vector<Point3f>& corners)
  {
    for (int i = 0; i < board_size.height; i++)
    {
      for (int j = 0; j < board_size.width; j++)
      {
        corners.push_back(
            Point3f(j * square_edge_length, i * square_edge_length, 0.0f));
      }
    }
  }
  void getChessboardCorners(vector<Mat> images,
                            vector<vector<Point2f>>& all_found_corners,
                            bool show_results = false)
  {
    cout << "getChessboardCorners function called" << endl;
    for (vector<Mat>::iterator iter = images.begin(); iter != images.end();
         iter++)
    {
      vector<Point2f> point_buf;
      bool found = findChessboardCorners(*iter, CHESSBOARD_DIMENSIONS, point_buf,
                                         CV_CALIB_CB_ADAPTIVE_THRESH |
                                             CV_CALIB_CB_NORMALIZE_IMAGE);

      if (found)
      {
        all_found_corners.push_back(point_buf);
      }
      if (show_results)
      {
        drawChessboardCorners(*iter, CHESSBOARD_DIMENSIONS, point_buf, found);
        imshow("Looking for Corners", *iter);
        waitKey(0);
      }
    }
    cout << "getChessboardCorners function succeded" << endl;
  }
  void cameraCalibration(vector<Mat> calibration_images, Size board_size,
                         float square_edge_length, Mat& camera_matrix,
                         Mat& distance_coefficients)
  {
    cout << "CameraCalibration function called" << endl;
    vector<vector<Point2f>> checkerboard_image_space_points;
    getChessboardCorners(calibration_images, checkerboard_image_space_points,
                         false);

    vector<vector<Point3f>> world_space_corner_points(1);

    createKnownBoardPosition(board_size, square_edge_length,
                             world_space_corner_points[0]);
    world_space_corner_points.resize(checkerboard_image_space_points.size(),
                                  world_space_corner_points[0]);

    vector<Mat> r_vectors, t_vectors;
    distance_coefficients = Mat::zeros(8, 1, CV_64F);

    calibrateCamera(world_space_corner_points, checkerboard_image_space_points,
                    board_size, camera_matrix, distance_coefficients, r_vectors,
                    t_vectors);
    cout << "CameraCalibration function succeded" << endl;
  }

  bool saveCameraCalibration(string name, Mat camera_matrix,
                             Mat distance_coefficients)
  {
    cout << "saveCameraCalibration function called" << endl;
    ofstream outStream(name);
    if (outStream)
    {
      uint16_t rows = camera_matrix.rows;
      uint16_t columns = camera_matrix.cols;

      outStream << rows << endl;
      outStream << columns << endl;

      for (int r = 0; r < rows; r++)
      {
        for (int c = 0; c < columns; c++)
        {
          double value = camera_matrix.at<double>(r, c);
          outStream << value << endl;
        }
      }

      rows = distance_coefficients.rows;
      columns = distance_coefficients.cols;

      outStream << rows << endl;
      outStream << columns << endl;

      for (int r = 0; r < rows; r++)
      {
        for (int c = 0; c < columns; c++)
        {
          double value = distance_coefficients.at<double>(r, c);
          outStream << value << endl;
        }
      }

      outStream.close();
      return true;
    }
    return false;
  }
  bool loadCameraCalibration(string name, Mat& camera_matrix,
                             Mat& distance_coefficients)
  {
    cout << "loadCameraCalibration function called" << endl;
    ifstream inStream(name);
    if (inStream)
    {
      uint16_t rows;
      uint16_t columns;

      inStream >> rows;
      inStream >> columns;

      camera_matrix = Mat(Size(columns, rows), CV_64F);

      for (int r = 0; r < rows; r++)
      {
        for (int c = 0; c < columns; c++)
        {
          double read = 0.0f;
          inStream >> read;
          camera_matrix.at<double>(r, c) = read;
         }
      }
      // Distance Coefficients
      inStream >> rows;
      inStream >> columns;

      distance_coefficients = Mat::zeros(rows, columns, CV_64F);

      for (int r = 0; r < rows; r++)
      {
        for (int c = 0; c < columns; c++)
        {
          double read = 0.0f;
          inStream >> read;
          distance_coefficients.at<double>(r, c) = read;
        }
      }
      inStream.close();
      return true;
      cout << "saveCameraCalibration function success" << endl;
    }
    return false;
  }
};
  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "CameraCalibration");
    CameraCalibration ic;

    ros::spin();
    return 0;
  }
