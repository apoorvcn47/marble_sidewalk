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
#include <pcl_ros/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>

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
      pub.publish(hough_tf->toImageMsg());
    }

    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc_input)
    {
        // Create a container for the data.
        sensor_msgs::PointCloud2::Ptr ros_inlier (new sensor_msgs::PointCloud2 ()),
                ros_outlier (new sensor_msgs::PointCloud2 ());


        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr
            pcl_input(new pcl::PointCloud<pcl::PointXYZ> ()),
            pcl_inlier(new pcl::PointCloud<pcl::PointXYZ> ()),
            pcl_outlier(new pcl::PointCloud<pcl::PointXYZ> ());

        pcl::fromROSMsg (*pc_input, *pcl_input);

        //Conditional outlier filter is used to divide the point cloud
        //Parameters are taken by trial and error
        //Better solution can be implemented by filtering every point by the equation of sidewalk lines
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond_in (new pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond_in->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -1)));
        range_cond_in->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, 1.5)));

        pcl::ConditionalRemoval<pcl::PointXYZ> condrem_in (range_cond_in);
        condrem_in.setInputCloud (pcl_input);
        condrem_in.setKeepOrganized(true);
        condrem_in.filter (*pcl_inlier);


        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond_out (new pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond_out->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, 1.5)));

        pcl::ConditionalRemoval<pcl::PointXYZ> condrem_out (range_cond_out);
        condrem_out.setInputCloud (pcl_input);
        condrem_out.setKeepOrganized(true);
        condrem_out.filter (*pcl_outlier);


        pcl::toROSMsg (*pcl_inlier, *ros_inlier);
        pcl::toROSMsg (*pcl_outlier, *ros_outlier);

        // Publish the dataSize
        //pc_in_pub.publish (output_inlier);

/*
        for (size_t i = 0; i < pcl_inlier->points.size (); ++i)
        {
            //if(pcl_inlier->points[i].y < 5 || pcl_inlier->points[i].z < 5)
            ROS_INFO("x = %f, y = %f, z = %f", pcl_inlier->points[i].x,pcl_inlier->points[i].y,pcl_inlier->points[i].z);
        }

*/

        // Publish the data.
        pc_in_pub.publish(ros_inlier);
        pc_out_pub.publish(ros_outlier);
    }




};



int main(int argc, char **argv)
{

  ros::init(argc, argv, "sidewalk_detection");
  sw_detector sw;
  ros::spin();

  return 0;
}
