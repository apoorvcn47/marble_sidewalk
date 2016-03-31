# marble_sidewalk
This code is written for detecting sidewalk by a depth sensor mounted differential drive robot
It uses OpenCV to calculate hough transform lines and depending on the orientation, selects the one that matches sidewalk.
It also uses PCL to split pointcloud and publish it on two different topics

Resources:
hough tf: 
https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughLines_Demo.cpp


cv bridge: http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

http://wiki.ros.org/pcl/Tutorials
