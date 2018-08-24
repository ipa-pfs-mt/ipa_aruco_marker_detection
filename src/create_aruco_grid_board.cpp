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

  void createArucoBoardGrid(int markersX, int markersY, int markerLength,
                            int markerSeparation, int dictionaryId, int margins,
                            int borderBits)
  {
    Size imageSize;
    imageSize.width = markersX * (markerLength + markerSeparation) -
                      markerSeparation + 2 * margins;
    imageSize.height = markersY * (markerLength + markerSeparation) -
                       markerSeparation + 2 * margins;

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(
        aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    Ptr<aruco::GridBoard> board =
        aruco::GridBoard::create(markersX, markersY, float(markerLength),
                                 float(markerSeparation), dictionary);

    // show created board
    Mat boardImage;
    board->draw(imageSize, boardImage, margins, borderBits);
    ostringstream convert;
    string imageName = "4x4Board1_";
    convert << imageName << ".jpg";
    imwrite(convert.str(), boardImage);

    imshow("board", boardImage);
    waitKey(0);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ArucoGridBoardCreator");
  ArucoGridBoardCreator ic;

  ros::spin();
  return 0;
}
