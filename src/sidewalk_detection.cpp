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
#include <vector>
#include <iostream>
#include <stdio.h>

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
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~sw_detector()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)//std_msgs::String::ConstPtr& msg)
    {
      ROS_INFO("I heard: ");

      cv_bridge::CvImagePtr cv_ptr, cv_ptr_flip, canny_edge, hough_tf;
      try
      {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          cv_ptr_flip = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          canny_edge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          hough_tf = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
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

      //Received ROSbag is upside down. Hence I flipped the image
      cv::flip(cv_ptr->image, cv_ptr_flip->image, -1);

      //Using canny edge detector (parameters are determined by trial and error)
      cv::Canny( cv_ptr_flip->image, canny_edge->image, 2, 100, 3 );

      //Using Hough transform
      cv::vector<cv::Vec2f> hough_lines;
      int left_sw_index;
      int right_sw_index;
      double left_angle_threshold = 0;
      double left_rho_threshold = 500;
      double right_angle_threshold = 7;
      double right_rho_threshold = 0;
      cv::Point left_p1, left_p2, right_p1, right_p2;


      cvtColor( canny_edge->image, hough_tf->image, CV_GRAY2BGR );


      HoughLines( canny_edge->image, hough_lines, 1, CV_PI/180, 100, 0, 0 );


      for( size_t i = 0; i < hough_lines.size(); i++ )
         {
          float r = hough_lines[i][0], t = hough_lines[i][1];

          //ROS_INFO("r = %f and t = %f", r,t);
          double cos_t = cos(t), sin_t = sin(t);
          double x0 = r*cos_t, y0 = r*sin_t;
          double alpha = 1000;


          cv::Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
          cv::Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );
          //cv::line( hough_tf->image, pt1, pt2, cv::Scalar(255,0,0), 3, CV_AA);

          //classifying lines in left and right quadrant
          //double x;
          //x = 240*(pt1.x-pt2.x)/(pt1.y-pt2.y);

          if ( /*x>0 && x<320 &&*/ t<1.39 && t>0.17 && t>left_angle_threshold /*&& r<left_rho_threshold*/)
          {
              //left_sw_index = i;
              left_angle_threshold = t;
              left_p1 = pt1;
              left_p2 = pt2;

          }
          else if( /*x>320 && x<640 &&*/ t>1.65 && t<3.14 && t<right_angle_threshold /*&& r>right_rho_threshold*/)
          {
              //right_sw_index = i;
              right_angle_threshold = t;
              right_p1 = pt1;
              right_p2 = pt2;
              ROS_INFO("r = %f and t = %f", r,t);
          }



         }
      cv::line( hough_tf->image, left_p1, left_p2, cv::Scalar(0,0,255), 3, CV_AA);
      cv::line( hough_tf->image, right_p1, right_p2, cv::Scalar(0,255,0), 3, CV_AA);
      //ROS_INFO("lx = %d, ly = %d and rx = %d, ry = %d", left_p1.x,left_p1.y,left_p2.x, left_p2.y);
      //ROS_INFO("lx = %d, ly = %d and rx = %d, ry = %d", right_p1.x,right_p1.y,right_p2.x, right_p2.y);




      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, hough_tf->image);
      cv::waitKey(3);
      //cv_ptr_flip = &flipped_image;
      //outputting modified image stream
      pub.publish(cv_ptr_flip->toImageMsg());
    }

};



int main(int argc, char **argv)
{

  ros::init(argc, argv, "sidewalk_detection");
  sw_detector sw;






  ros::spin();

  return 0;
}
