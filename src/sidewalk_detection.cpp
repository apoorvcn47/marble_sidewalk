/*This program detects sidewalk borders
 * Left border is detected by red line and right border is detected by green line
 * These lines continue indefinitely
 * line segment after intersection of these lines can be ignored
 */


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
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

static const std::string OPENCV_WINDOW = "Image window";

class sw_detector
{
    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;
    ros::Publisher pc_in_pub;
    ros::Publisher pc_out_pub;
    ros::Subscriber pc_sub;

    //Points to draw two lines corresponding to left and right border of sidewalk respectively
    cv::Point left_p1, left_p2, right_p1, right_p2;

public:
    sw_detector(): it_(n)
    {
        sub = it_.subscribe("/camera/color/image_raw", 1, &sw_detector::chatterCallback, this);
        pub = it_.advertise("/sidewalk_detector/color/image_raw", 1);
        cv::namedWindow(OPENCV_WINDOW);
        pc_sub = n.subscribe("/camera/depth/points",1, &sw_detector::pc_callback, this);
        pc_in_pub = n.advertise<sensor_msgs::PointCloud2> ("/sidewalk_detector/depth/points_in", 1);
        pc_out_pub = n.advertise<sensor_msgs::PointCloud2> ("/sidewalk_detector/depth/points_out", 1);
    }


    ~sw_detector()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
      ROS_INFO("I heard: ");

      cv_bridge::CvImagePtr cv_ptr, cv_ptr_flip, canny_edge, hough_tf;
      try
      {
          //this image is a copy of ros message
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          //Since ROS message is inverted, this is a flipped image
          cv_ptr_flip = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          //Image for Caany edge detection
          canny_edge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          //Image to run Hough Transform
          hough_tf = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }

      //Received ROSbag is upside down. Hence I flipped the image
      cv::flip(cv_ptr->image, cv_ptr_flip->image, -1);

      //Using canny edge detector (parameters are determined by trial and error)
      cv::Canny( cv_ptr_flip->image, canny_edge->image, 2, 100, 3 );

      //Using Hough transform
      cv::vector<cv::Vec2f> hough_lines;

      //In a single image, there are multiple lines detected by Hough Transform
      //To chose the best one, following are the thresholds

      double left_angle_threshold = 0;
      double right_angle_threshold = 7;


      cvtColor( canny_edge->image, hough_tf->image, CV_GRAY2BGR );
      //Parameters are configured by trial end error
      HoughLines( canny_edge->image, hough_lines, 1, CV_PI/180, 100, 0, 0 );


      for( size_t i = 0; i < hough_lines.size(); i++ )
         {
          float r = hough_lines[i][0], t = hough_lines[i][1];

          //This statement gives rho and theta of each line dettected by hough transform
          //ROS_INFO("r = %f and t = %f", r,t);

          //Finding out two points on the line to draw them
          double cos_t = cos(t), sin_t = sin(t);
          double x0 = r*cos_t, y0 = r*sin_t;
          double alpha = 1000;


          cv::Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
          cv::Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );

          //Following statement draws all the detected lines
          //cv::line( hough_tf->image, pt1, pt2, cv::Scalar(255,0,0), 3, CV_AA);

          //classifying lines in left and right quadrant
          if (t<1.39 && t>0.17 && t>left_angle_threshold)
          {
              left_angle_threshold = t;
              left_p1 = pt1;
              left_p2 = pt2;
          }
          else if(t>1.75 && t<3.14 && t<right_angle_threshold)
          {
              right_angle_threshold = t;
              right_p1 = pt1;
              right_p2 = pt2;
          }



         }
      //Draws left sidewalk border in red color
      cv::line( hough_tf->image, left_p1, left_p2, cv::Scalar(0,0,255), 3, CV_AA);
      //Draws right sidewalk border in green color
      cv::line( hough_tf->image, right_p1, right_p2, cv::Scalar(0,255,0), 3, CV_AA);



      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, hough_tf->image);
      cv::waitKey(3);
      pub.publish(cv_ptr_flip->toImageMsg());
    }

    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc_input)
    {
        // Create a container for the data.
        sensor_msgs::PointCloud2 output_inlier, output_outlier;

        // Do data processing here...
        //output_inlier = *pc_input;
        //output_outlier = *pc_input;

        //Finding intercept of sidewalk lines on the image frame
        std::vector<int> lline_intercept, rline_intercept;
        lline_intercept = frame_intercept(left_p1, left_p2);
        rline_intercept = frame_intercept(right_p1, right_p2);

        //image frame is 640X480 and point cloud frame is 480X360
        //to find corresponding intercepts on cloud frame,multiply intercepts by 3/4
        for(int i=0; i<lline_intercept.size(); i++)
        {
            lline_intercept.at(i) = lline_intercept.at(i)*3/4;
            rline_intercept.at(i) = rline_intercept.at(i)*3/4;
        }

        for(int i; i<pc_input->data.size(); i++)
        {
            if(i<pc_input->data.size()/2)
            {
                output_inlier.data[i]=pc_input->data[i];
                output_outlier.data[i]=0;
            }
            else
            {
                output_inlier.data[i]=0;
                output_outlier.data[i]=pc_input->data[i];
            }
        }
        //ROS_INFO("lx1 = %d, ", pc_input->data.);
        //pc_input->data[3][4];

        //ROS_INFO("lx1 = %d, ly1 = %d, lx2 = %d, ly2 = %d", rline_intercept.at(0),rline_intercept.at(1),rline_intercept.at(2)
        //         ,rline_intercept.at(3));


        // Publish the data.
        pc_in_pub.publish(output_inlier);
        pc_out_pub.publish(output_outlier);
    }

    std::vector<int> frame_intercept(cv::Point p1, cv::Point p2)
    {
        std::vector<int> v;
        if(p1.y-p2.y==0 || p1.x-p2.x==0)
        {
            //better solution can be implemented here
            for(int i=0;i<4;i++)
            {
                v.push_back(0);
            }
            return v;
        }

        int x1, x2, y3, y4;
        x1 = ((0-p1.y)*(p1.x-p2.x)/(p1.y-p2.y))+p1.x;
        x2 = ((480-p1.y)*(p1.x-p2.x)/(p1.y-p2.y))+p1.x;

        y3 = ((0-p1.x)*(p1.y-p2.y)/(p1.x-p2.x))+p1.y;
        y4 = ((640-p1.x)*(p1.y-p2.y)/(p1.x-p2.x))+p1.y;


        if(x1>=0 && x1<=640)
        {
            v.push_back(x1);
            v.push_back(0);
        }
        if(x2>=0 && x2<=640)
        {
            v.push_back(x2);
            v.push_back(480);
        }
        if(y3>=0 && y3<=480)
        {
            v.push_back(0);
            v.push_back(y3);
        }
        if(y4>=0 && y4<=480)
        {
            v.push_back(640);
            v.push_back(y4);
        }
        return v;

    }

};



int main(int argc, char **argv)
{

  ros::init(argc, argv, "sidewalk_detection");
  sw_detector sw;
  ros::spin();

  return 0;
}
