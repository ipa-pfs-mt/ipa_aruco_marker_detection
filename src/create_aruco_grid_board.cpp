#include <ros/ros.h>

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

using namespace std;
using namespace cv;

class ArucoGridBoardCreator
{
  ros::NodeHandle nh;

public:
  ArucoGridBoardCreator() { createArucoBoardGrid(7, 2, 500, 100, 0, 0, 1); }
  ~ArucoGridBoardCreator() {}

  void createArucoBoardGrid(int markers_x, int markers_y, int marker_length,
                            int marker_separation, int dictionary_id, int margins,
                            int border_bits)
  {
    Size image_size;
    image_size.width = markers_x * (marker_length + marker_separation) -
                      marker_separation + 2 * margins;
    image_size.height = markers_y * (marker_length + marker_separation) -
                       marker_separation + 2 * margins;

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(
        aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

    Ptr<aruco::GridBoard> board =
        aruco::GridBoard::create(markers_x, markers_y, float(marker_length),
                                 float(marker_separation), dictionary);

    // show created board
    Mat board_image;
    board->draw(image_size, board_image, margins, border_bits);
    ostringstream convert;
    string image_name = "4x4Board1_";
    convert << image_name << ".jpg";
    imwrite(convert.str(), board_image);

    imshow("board", board_image);
    waitKey(0);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ArucoGridBoardCreator");
  ArucoGridBoardCreator board;

  ros::spin();
  return 0;
}
