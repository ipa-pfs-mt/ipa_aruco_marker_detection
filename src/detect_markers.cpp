#include "detect_markers/detect_markers.h"
#include <ros/console.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

MarkerDetection::MarkerDetection() : image_transport(node_handle)
{

  image_sub = image_transport.subscribe("/camera/color/image_raw", 1,
                                        &MarkerDetection::imageCb, this);

  loadCameraCalibration("Camera Calib File", camera_matrix,
                        distance_coefficients);
  ROS_DEBUG_STREAM_THROTTLE_NAMED(5, "test_only", camera_matrix);
}
MarkerDetection::~MarkerDetection() {}

void MarkerDetection::imageCbIr(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(
        msg, enc::TYPE_8UC1); // cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  estimatePoseBoard(cv_ptr->image, camera_matrix, distance_coefficients, 0.05f,
                    "Infrared");
}
void MarkerDetection::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  estimatePoseBoard(cv_ptr->image, camera_matrix, distance_coefficients, 0.03f,
                    "Color");
}
int MarkerDetection::estimatePoseBoard(Mat frame, const Mat& camera_matrix,
                                       const Mat& distance_coefficients,
                                       float aruco_square_dimensions,
                                       string window_name)
{

  vector<int> marker_ids, recovered_ids;
  vector<vector<Point2f>> marker_corners, rejected_candidates;
  Ptr<aruco::DetectorParameters> parameters =
      aruco::DetectorParameters::create();
  parameters->cornerRefinementMethod =
      2; // 1- CORNER_REFINE_SUBPIX, 2-CORNER_LINES
  // parameters->perspectiveRemoveIgnoredMarginPerCell = 0.30;

  Ptr<aruco::Dictionary> marker_dictionary = aruco::getPredefinedDictionary(
      aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

  namedWindow(window_name, CV_WINDOW_AUTOSIZE);

  cv::Ptr<cv::aruco::GridBoard> board =
      cv::aruco::GridBoard::create(7, 2, 0.05, 0.01, marker_dictionary, 0);

  aruco::detectMarkers(frame, marker_dictionary, marker_corners, marker_ids,
                       parameters, rejected_candidates);
  aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
  // aruco::refineDetectedMarkers(frame, board, markerCorners, markerIds,
  // rejectedCandidates,
  //                            cameraMatrix, distanceCoefficients, 10.f, 3.f,
  //                            true, recoveredIds, parameters );

  if (marker_ids.size() > 1)
  {
    Vec3d rotation_vectors, translation_vectors;
    int valid = aruco::estimatePoseBoard(marker_corners, marker_ids, board,
                                         camera_matrix, distance_coefficients,
                                         rotation_vectors, translation_vectors);

    if (valid > 0)
    {
      aruco::drawAxis(frame, camera_matrix, distance_coefficients,
                      rotation_vectors, translation_vectors, 0.1);

      PoseCallback(rotation_vectors, translation_vectors);
    }
  }
  imshow(window_name, frame);

  waitKey(5);
  return 1;
}

void MarkerDetection::PoseCallback(Vec3d rotation_vectors,
                                   Vec3d translation_vectors)
{
  // Camera Holder Frame
  static tf::TransformBroadcaster broadcaster_camera_holder;
  tf::Transform transform_camera_holder;
  tf::Quaternion quaternion_camera_holder;
  transform_camera_holder.setOrigin(tf::Vector3(0.574, 0.0, 0.250));

  Vec3f euler_angles_camera_holder{-0.925, 0, -1.5707}; // radians!
  quaternion_camera_holder.setRPY(euler_angles_camera_holder[0],
                                  euler_angles_camera_holder[1],
                                  euler_angles_camera_holder[2]);

  transform_camera_holder.setRotation(quaternion_camera_holder);

  broadcaster_camera_holder.sendTransform(tf::StampedTransform(
      transform_camera_holder, ros::Time::now(), "stick", "camera_holder"));

  // Marker frame
  static tf::TransformBroadcaster broadcaster_marker;
  tf::Transform transform_marker;
  transform_marker.setOrigin(tf::Vector3(
      translation_vectors[0], translation_vectors[1], translation_vectors[2]));
  tf::Quaternion quaternion_marker;
  Mat rotation_matrix;

  cv::Rodrigues(rotation_vectors, rotation_matrix);
  Vec3f euler_angles_marker = rotationMatrixtoEulerAngles(rotation_matrix);

  quaternion_marker.setRPY(euler_angles_marker[0], euler_angles_marker[1],
                           euler_angles_marker[2]);
  transform_marker.setRotation(quaternion_marker);

  broadcaster_marker.sendTransform(tf::StampedTransform(
      transform_marker, ros::Time::now(), "camera_holder", "marker"));
  geometry_msgs::Pose marker_pose;
}
Vec3f MarkerDetection::rotationMatrixtoEulerAngles(Mat& rotation_matrix)
{
  float sy =
      sqrt(rotation_matrix.at<double>(0, 0) * rotation_matrix.at<double>(0, 0) +
           rotation_matrix.at<double>(1, 0) * rotation_matrix.at<double>(1, 0));

  bool singular = sy < 1e-6;

  float x, y, z;
  if (!singular)
  {
    x = atan2(rotation_matrix.at<double>(2, 1),
              rotation_matrix.at<double>(2, 2));
    y = atan2(-rotation_matrix.at<double>(2, 0), sy);
    z = atan2(rotation_matrix.at<double>(1, 0),
              rotation_matrix.at<double>(0, 0));
  }
  else
  {
    x = atan2(-rotation_matrix.at<double>(1, 2),
              rotation_matrix.at<double>(1, 1));
    y = atan2(-rotation_matrix.at<double>(2, 0), sy);
    z = 0;
  }
  return Vec3f(x, y, z);
}

Vec3d MarkerDetection::RunningAverage(Vec3d vector)
{
  std::list<Vec3d> buffer;
  buffer.push_back(vector);
  if (buffer.size() > 10)
    buffer.pop_front();
  Vec3d sum = vector;
  for (list<Vec3d>::iterator it = buffer.begin(); it != buffer.end(); ++it)
    sum += (Vec3d)*it;
  Vec3d mean = sum * (int)(1 / buffer.size());
  return mean;
}

bool MarkerDetection::loadCameraCalibration(string name, Mat& camera_matrix,
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

void MarkerDetection::getMarkerCoordinates(vector<Vec3d> rotation_vectors,
                                           vector<Vec3d> translation_vectors,
                                           vector<int> marker_ids)
{
  for (int i = 0; i < marker_ids.size(); i++)
  {
    cout << " X: " << translation_vectors[i][0]
         << " Y: " << translation_vectors[i][1]
         << " Z: " << translation_vectors[i][2] << endl;
  }
}
