# marble_sidewalk
This code is written for detecting sidewalk by a depth sensor mounted differential drive robot
It uses OpenCV to calculate hough transform lines and depending on the orientation, selects the one that matches sidewalk.
It also uses PCL to split pointcloud and publish it on two different topics
