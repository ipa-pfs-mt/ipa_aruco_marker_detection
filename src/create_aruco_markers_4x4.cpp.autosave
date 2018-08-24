//ROS
#include <ros/ros.h>

//OPENCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

//IO
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace cv;

class CreateMarkers
{
  ros::NodeHandle nh;

public:

  CreateMarkers(){createArucoMarkers();}
  ~CreateMarkers() {}

  void createArucoMarkers()
  {
    Mat output_marker;

    Ptr<aruco::Dictionary> marker_dictionary = aruco::getPredefinedDictionary(
        aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    for (int i = 0; i < 10; i++)
    {
      aruco::drawMarker(marker_dictionary, i, 500, output_marker, 1);
      ostringstream convert;
      string image_name = "4x4Marker_";
      convert << image_name << i << ".jpg";
      imwrite(convert.str(), output_marker);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "create_aruco_markers");
  CreateMarkers markers;

  ros::spin();
  return 0;
}
