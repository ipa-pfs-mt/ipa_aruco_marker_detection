#include "detect_markers/detect_markers.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "marker_detection");
  MarkerDetection md;

  ros::spin();
  return 0;
}
