#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_realsense_opencv_tutorial");
  ros::NodeHandle nh;

  cout << "OpenCV version : " << CV_VERSION << endl;
  cout << "Major version : "  << CV_MAJOR_VERSION << endl;

  rs2::pipeline pipe;
  rs2::config cfg;
  rs2::frameset frames;
  rs2::frame color_frame;


  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  pipe.start(cfg);

  for(int i=0; i < 30; i ++)
  {
    frames = pipe.wait_for_frames();
  }

  namedWindow("Display Imagee", WINDOW_AUTOSIZE);

  ros::Rate loop_rate(30);

  while(ros::ok())
  {
    frames = pipe.wait_for_frames();
    color_frame = frames.get_color_frame();
    Mat color(Size(640,480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

    imshow("Display Image", color);

    if(waitKey(10)==27) break;
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
