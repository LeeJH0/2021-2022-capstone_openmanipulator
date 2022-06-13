#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <termios.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "roscpp_tutorials/TwoInts.h"
#include "open_manipulator_pick_and_place/gripper_state.h" // topic하는 부분 (중호)


int mode ;
int color = 0 ;

bool kbhit()
{
  termios term;
  tcgetattr(0, &term);

  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);

  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);

  tcsetattr(0, TCSANOW, &term);

  return byteswaiting > 0;
}

void printText() {
  system("clear");
  
  printf("\n");
  printf("-----------------------------\n") ;
  if(mode == 1) printf("Mode = 1\n") ;
  if(mode == 2) printf("Mode = 2\n") ;
  if(mode == 0) printf("Mode = 0\n") ;
  printf("-----------------------------\n\n") ;

  printf("color : %d", color) ;

  
}

void setModeState(char ch) {
  if(ch == '1') mode = 1 ;
  else if(ch == '2') mode = 2 ;
  else if(ch == '3') mode = 3 ;
}

void msgCallback(const open_manipulator_pick_and_place::gripper_state::ConstPtr& msg)
{
	ROS_INFO("value: [%d]", msg->grip_state);
	
	color = msg->grip_state ;	
	
}

// without using the image transport
// we have to REMAKE this int with variables and so on. 
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  ros::NodeHandle node_handle_;
  ros::ServiceClient window_client ;

  window_client = node_handle_.serviceClient<roscpp_tutorials::TwoInts>("add_two_ints");
  roscpp_tutorials::TwoInts srv;
  
  int height = 960 ;
  int width = 540 ;
  int box = 400 ;

  int color_B = 1 ;
  int color_G = 1 ;
  int color_R = 1 ;

  cv::Rect rect1 = cv::Rect(height/2-box/2, width/2-box/2 , box, box) ;
  cv::Rect rect2 = cv::Rect(300, 300, 270, 60) ;

  if(kbhit())
    setModeState(std::getchar());

  try
  {
    printText() ;

      
   if(color == 0) {
	color_B = 255 ;
	color_G = 0 ;
	color_R = 0 ;
	}
    else if(color == 1) {
	color_B = 0 ;
	color_G = 0 ;
	color_R = 255 ;
	}

    
     

    cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
    cv::resize(image, image, cv::Size(height, width)) ;

    if (mode == 1)
    {
	cv::rectangle(image, rect1, cv::Scalar(color_B, color_G, color_R), 3, 8, 0);
	}
    
    else if (mode == 2)
    {
	cv::rectangle(image, rect2, cv::Scalar(color_B, color_G, color_R), 3, 8, 0);
	}



    cv::imshow("view", image);

    cv::waitKey(500);

    // ros::Duration(1).sleep();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to image!");
  }
}
 



int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ros::NodeHandle msgnh;

  cv::namedWindow("view");
  cv::startWindowThread();
  

  ros::Subscriber sub = nh.subscribe("camera/image/compressed", 1, imageCallback);
  ros::Subscriber msgsub = nh.subscribe("gripper_state_msg", 10, msgCallback);
  ros::spin();

  cv::destroyWindow("view");
}


   // ros_tutorial_sub = node_handle_.subscribe<ros_tutorials_topic::MsgTutorial>("ros_tutorial_msg", 100, boost::bind(&OpenManipulatorMaster::msgCallback, this, _1));
   
    /*
    roscpp_tutorials::TwoInts srv;

    srv.request.a = 3;
    window_client.call(srv) ;

    // if(client.call(srv)) printf("%d", srv.response.master) ;

    if(srv.response.sum == 0) {
	color_B = 255 ;
	color_G = 0 ;
	color_R = 0 ;
	}
    else if(srv.response.sum == 1) {
	color_B = 255 ;
	color_G = 255 ;
	color_R = 0 ;
	}

*/
