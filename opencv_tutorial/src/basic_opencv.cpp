#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_opencv");
  ros::NodeHandle nh;

  cout << "OpenCV version : " << CV_VERSION << endl;
  cout << "Major version : " << CV_MAJOR_VERSION << endl;
  ROS_INFO("Hello world!");

  double width;
  double height;
  double exposure;
  double brightness;

  Mat img;

  VideoCapture vc(0);

  width = vc.get(CAP_PROP_FRAME_WIDTH);
  height = vc.get(CAP_PROP_FRAME_HEIGHT);
  exposure = vc.get(CAP_PROP_EXPOSURE);
  brightness = vc.get(CAP_PROP_BRIGHTNESS);

  cout << "width : " << width << endl;
  cout << "height : " << height << endl;
  cout << "exposure : " << exposure << endl;
  cout << "brightness : " << brightness << endl;
  namedWindow("video", 1);
  ros::Rate loop_rate(60);

  while(ros::ok()){
    vc >> img;

    flip(img, img, 1);

    imshow("video", img);

    if(waitKey(10) == 27) break;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;


}
