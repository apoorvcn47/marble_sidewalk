//hough tf: https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughLines_Demo.cpp
//cv bridge: http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
//image_transport use
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "Image window";

class sw_detector
{
    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;

public:
    sw_detector(): it_(n)
    {
        sub = it_.subscribe("/camera/color/image_raw", 1, &sw_detector::chatterCallback, this);
        pub = it_.advertise("/sidewalk_detector/color/image_raw", 1);
        cv::namedWindow("OPENCV_WINDOW");
    }

    ~sw_detector()
    {
        cv::destroyWindow("OPENCV_WINDOW");
    }

    void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)//std_msgs::String::ConstPtr& msg)
    {
      ROS_INFO("I heard: ");

      cv_bridge::CvImagePtr cv_ptr;
      try
      {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }

      /*Modify image
      // Draw an example circle on the video stream
      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      */

      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);

      //outputting modified image stream
      pub.publish(cv_ptr->toImageMsg());
    }

};



int main(int argc, char **argv)
{

  ros::init(argc, argv, "sidewalk_detection");
  sw_detector sw;






  ros::spin();

  return 0;
}
