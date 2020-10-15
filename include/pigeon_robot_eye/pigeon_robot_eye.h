#ifndef PIGEON_ROBOT_EYE_H
#define PIGEON_ROBOT_EYE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>

#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"

#include <termios.h>
#include <unistd.h>

/*struct DATA_NAME_1
{
  double liner_x = 0;
};
struct DATA_NAME_2
{
  double angular_z = 0;
};

enum packet_size : unsigned int
{
    SIZE_IN = sizeof(DATA_NAME_1),
    SIZE_OUT = sizeof(DATA_NAME_2),

};*/

// 120 100

class Pigeon_robot_eye
{
public:
    Pigeon_robot_eye(ros::NodeHandle &n)
           : loop_rate(120)
       {
          // open run
          ROS_INFO("PIGEON_ROBOT_EYE_NODE OPNE");
       }
       ~Pigeon_robot_eye()
       {
          // close run
          ROS_INFO("PIGEON_ROBOT_EYE_NODE CLOSE");
       }

    int input_return_key();

    void set_camera();

    void wait_camera();

    cv::Mat Draw_RockOn(cv::Mat Matrix);

    cv::Mat color_to_gray(cv::Mat Matrix);

    cv::Mat Histogram(cv::Mat Matrix);

    cv::Mat Binary(cv::Mat Matrix);

    cv::Mat reversal(cv::Mat Matrix);

    cv::Mat labeling(cv::Mat Matrix, cv::Mat color);

    cv::Mat detect_faces(cv::Mat Matrix);

    cv::Mat detect(cv::Mat Matrix);

    void video_cap(cv::Mat Matrix);

    void display_camera();

    void update();

private:
    ros::Subscriber sub;
    ros::Rate loop_rate;

    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset frames;
    rs2::frame color_frame;

    cv::CascadeClassifier classifier;

//    cv::VideoCapture cap("black_box.avi");

    int Width = 640;
    int Hight = 480;
    int camera_param = 0;

};


#endif // PIGEON_ROBOT_EYE_H
