#include <ros/ros.h>
#include <pigeon_robot_eye/pigeon_robot_eye.h>
#include <math.h>

#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <fcntl.h>

#include <stdlib.h>
#include <errno.h>

#define PI 3.14159265

#define INIT_BUFSIZE 1024

int Pigeon_robot_eye::input_return_key()
{

}

cv::Mat Pigeon_robot_eye::Draw_RockOn(cv::Mat Matrix)
{
  double sum = 0;

  for(int i=0; i < 7; i++){
    sum += 0.166667;
    cv::line(Matrix, cv::Point(Width*sum,0), cv::Point(Width*sum, Hight), cv::Scalar(255,255,255,50),1, cv::LINE_8);
    cv::line(Matrix, cv::Point(0,Hight*sum), cv::Point(Width, Hight*sum), cv::Scalar(255,255,255,50),1, cv::LINE_8);
//    std::cout << sum << std::endl;

  }

  cv::line(Matrix, cv::Point(Width/2, Hight/2 -20), cv::Point(Width/2, Hight/2 +20), cv::Scalar(255,255,255,50), 4, cv::LINE_8);
  cv::line(Matrix, cv::Point(Width/2 -20, Hight/2), cv::Point(Width/2+20, Hight/2), cv::Scalar(255,255,255,50), 4, cv::LINE_8);


  cv::line(Matrix, cv::Point(Width/2 + Hight/2.5 - 20 , Hight/2), cv::Point(Width/2, Hight/2 + Hight/2.5 - 20), cv::Scalar(255,255,255,50), 4, cv::LINE_8);
  cv::line(Matrix, cv::Point(Width/2 - Hight/2.5 + 20 , Hight/2), cv::Point(Width/2, Hight/2 + Hight/2.5 - 20), cv::Scalar(255,255,255,50), 4, cv::LINE_8);
  cv::line(Matrix, cv::Point(Width/2 - Hight/2.5 + 20 , Hight/2), cv::Point(Width/2, Hight/2 - Hight/2.5 + 20), cv::Scalar(255,255,255,50), 4, cv::LINE_8);
  cv::line(Matrix, cv::Point(Width/2 + Hight/2.5 - 20 , Hight/2), cv::Point(Width/2, Hight/2 - Hight/2.5 + 20), cv::Scalar(255,255,255,50), 4, cv::LINE_8);


  cv::circle(Matrix, cv::Point(Width/2, Hight/2), Hight/2.5, cv::Scalar(0, 165, 255, 50), 4);
  cv::circle(Matrix, cv::Point(Width/2, Hight/2), Hight/2.5 - 20, cv::Scalar(0, 165, 255, 50), 4);
  cv::circle(Matrix, cv::Point(Width/2, Hight/2), 30, cv::Scalar(0, 165, 255, 50), 2);

//  cv::line(Matrix, cv::Point(Width/2 - Hight/2.5 + 40, Hight/2 - Hight/2.5 + 40), cv::Point(Width/2, Hight/2), cv::Scalar(255,255,255,50), 4, cv::LINE_8);

  return Matrix;
}

void Pigeon_robot_eye::set_camera()
{
  std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  std::cout << "Major version : "  << CV_MAJOR_VERSION << std::endl;

  cfg.enable_stream(RS2_STREAM_COLOR, Width, Hight, RS2_FORMAT_BGR8, 60);
  pipe.start(cfg);

}

void Pigeon_robot_eye::wait_camera()
{
  for(int i=0; i < 30; i ++)
  {
    frames = pipe.wait_for_frames();
  }
}

cv::Mat Pigeon_robot_eye::color_to_gray(cv::Mat Matrix)
{
  cv::Mat gray;
  cv::cvtColor(Matrix,gray,cv::COLOR_BGR2GRAY);
  return gray;
}

cv::Mat Pigeon_robot_eye::Histogram(cv::Mat Matrix)
{
  double gmax, gmin;
  cv::minMaxLoc(Matrix, &gmin, &gmax);

  cv::Mat histogram;

  cv::equalizeHist(Matrix,histogram);

//  return (Matrix - gmin) * 255 / (gmax - gmin);
  return histogram;
}

cv::Mat Pigeon_robot_eye::Binary(cv::Mat Matrix)
{

//  cv::adaptiveThreshold(Matrix, Matrix, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,21, 9);
  cv::threshold(Matrix, Matrix, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  cv::morphologyEx(Matrix, Matrix, cv::MORPH_CLOSE, cv::Mat()); // 닫기

//  cv::morphologyEx(Matrix, Matrix, cv::MORPH_DILATE , cv::Mat()); //팽창
//  cv::morphologyEx(Matrix, Matrix, cv::MORPH_OPEN , cv::Mat());
//  cv::morphologyEx(Matrix, Matrix, cv::MORPH_GRADIENT , cv::Mat());
//  cv::adaptiveThreshold(Matrix, Matrix, 200, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY,3, 5);

  reversal(Matrix);

  return Matrix;
}

cv::Mat Pigeon_robot_eye::reversal(cv::Mat Matrix)
{
//  cv::Mat Mask(480, 640, CV_8U, cv::Scalar(0, 0, 0));

  cv::bitwise_not(Matrix,Matrix);

  return Matrix;
}

cv::Mat Pigeon_robot_eye::labeling(cv::Mat Matrix, cv::Mat color) // (2진 영상 입력, 출력 컬러 영상) //개발중
{
  cv::Mat labels, stats, centroids;
  int cnt = cv::connectedComponentsWithStats(Matrix, labels, stats, centroids);

//  cv::cvtColor(Matrix, Matrix, cv::COLOR_GRAY2BGR);

  for(int i = 1; i < cnt; i++)
  {
    int* p = stats.ptr<int>(i);

    if(p[4] < 700){
      if(p[4] > 1000) continue;
    }

    cv::rectangle(color,cv::Rect(p[0],p[1],p[2],p[3]), cv::Scalar(0, 255, 255), 2);
  }

  return color;

}

cv::Mat Pigeon_robot_eye::detect_faces(cv::Mat Matrix) //개발중
{
  cv::CascadeClassifier a("/home/pigeon/ros_camera/src/ROS_realsense_opencv_tutorial/haarcascade_frontalface_default.xml");

  std::vector<cv::Rect> faces;
  a.detectMultiScale(Matrix, faces,1.1,3,0, cv::Size(),cv::Size());

  for (cv::Rect rc : faces) {
    cv::rectangle(Matrix, rc, cv::Scalar(255,255,255), 2);
  }

  return Matrix;
}

cv::Mat Pigeon_robot_eye::detect(cv::Mat Matrix)
{
  cv::HOGDescriptor hog;
  hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

  std::vector<cv::Rect> detected;
  std::vector<double> weight;
  hog.detectMultiScale(Matrix, detected, weight);

//  std::cout << "x :" << detected[0].x << std::endl;
//  std::cout << "y : " << detected[0].y << std::endl;
//  std::cout << "width : " << detected[0].width << std::endl;
//  std::cout << "height: " << detected[0].height << std::endl;


  for(cv::Rect r : detected){
    cv::Scalar c = cv::Scalar(rand() % 256, rand() % 256, rand() & 256 );
    cv::rectangle(Matrix,r,c,3);
    std::cout << "x :" << r.x << std::endl;
    std::cout << "y : " << r.y << std::endl;
    std::cout << "width : " << r.width << std::endl;
    std::cout << "height: " << r.height << std::endl;

    cv::circle(Matrix, cv::Point(r.x + r.width/2 , r.y + r.height/2), 30, cv::Scalar(0, 165, 255, 80), 4,cv::FILLED);
//    std::cout << "vec[0] : " << weight[0] << std::endl;
//    std::cout << "vec[1] : " << weight[1] << std::endl;
//    std::cout << "vec[2] : " << weight[2] << std::endl;
  }

  return Matrix;

}

void Pigeon_robot_eye::video_cap(cv::Mat Matrix) // 개발중
{
  int fourcc = cv::VideoWriter::fourcc('X','2','6','4');
  cv::VideoWriter outputVideo("black_box.avi", fourcc, 30, cv::Size(Width,Hight));

  outputVideo << Matrix;

}

void Pigeon_robot_eye::display_camera()
{
  frames = pipe.wait_for_frames();
  color_frame = frames.get_color_frame();
  cv::Mat color(cv::Size(Width,Hight), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

//  cv::imshow("Color Image", color);
//  cv::imshow("Gray Image", color_to_gray(color));
//  cv::imshow("Histogram Image", Histogram(color_to_gray(color)));
//  cv::imshow("Threshole Image", Binary(Histogram(color_to_gray(color))));

//  cv::imshow("labeling Image", labeling(Binary(Histogram(color_to_gray(color))), color));

//  cv::imshow("Detect face", detect_faces(color));

  cv::imshow("Detect human", detect(color));

  cv::imshow("Draw Image", Draw_RockOn(color));

//  video_cap(color);

//  cv::imshow("Display Image", color);
}

void Pigeon_robot_eye::update()
{
  set_camera();
  wait_camera();
}

int dir(char *arg)
{
  DIR *pdir;
  struct dirent *dirt;
//  struct stat statBuf;
  int i = 0, count = 0;
  char buf[255];

//  memset(&)

  pdir = opendir(arg);
  if (!pdir){
//  if(() <= 0) {
    perror("opendir");
    return -1;
  }

  while(dirt = readdir(pdir)){
    printf("%s\n", dirt->d_name);
//    printf("%s\n", dirt->);
  }
  closedir(pdir);

//  chdir
}

char* pwd(void)
{
//  char *buf, *tmp;
  char buf[INIT_BUFSIZE];
  size_t size = INIT_BUFSIZE;

//  buf = malloc(size);
  getcwd(buf,size);

  std::cout << buf << " " << size << std::endl;

//  if(!buf)

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pigeon_robot_eye_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(60);

  Pigeon_robot_eye Pigeon_robot_eye(n);

  Pigeon_robot_eye.update();

//  dir(argv[1]);

//  pwd();

  while (ros::ok())
  {
    Pigeon_robot_eye.display_camera();
    ros::spinOnce();
    loop_rate.sleep();
    if(cv::waitKey(10)==27) return 0;
  }

  return 0;

}
