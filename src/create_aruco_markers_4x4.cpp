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
    Mat outputMarker;

    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(
        aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    for (int i = 0; i < 10; i++)
    {
      aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
      ostringstream convert;
      string imageName = "4x4Marker_";
      convert << imageName << i << ".jpg";
      imwrite(convert.str(), outputMarker);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "create_aruco_markers");
  CreateMarkers cm;

  ros::spin();
  return 0;
}
