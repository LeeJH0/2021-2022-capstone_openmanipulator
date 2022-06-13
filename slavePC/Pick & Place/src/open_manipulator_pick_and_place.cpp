/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */
//library
#include "open_manipulator_pick_and_place/open_manipulator_pick_and_place.h"

#include "ros/ros.h" // 기본 헤더 파일
#include "ros_tutorials_topic/MsgTutorial.h"// MsgTutorial 메시지 파일 헤더(빌드 후 자동 생성됨)
#include "open_manipulator_pick_and_place/gripper_state.h" // topic하는 부분 (중호)
#include "roscpp_tutorials/TwoInts.h"
#include <cstdlib>



/////////////////////////
// pick and place 예제 //
/////////////////////////

int grip_state_change = 0;

// OpenManipulatorPickandPlace class 선언
OpenManipulatorPickandPlace::OpenManipulatorPickandPlace()
: node_handle_(""),
  priv_node_handle_("~"),
  mode_state_(0),
  demo_count_(0),
  pick_ar_id_(0)
{
  node_handle_ = ros::NodeHandle() ;

  present_joint_angle_.resize(NUM_OF_JOINT_AND_TOOL, 0.0);
  present_kinematic_position_.resize(3, 0.0);

  // joint_name 에 joint 1,2,3,4 넣기
  joint_name_.push_back("joint1");
  joint_name_.push_back("joint2");
  joint_name_.push_back("joint3");
  joint_name_.push_back("joint4");

  gripper_state_pub = node_handle_.advertise<open_manipulator_pick_and_place::gripper_state>("gripper_state_msg", 100); // 추가 (중호)
  open_manipulator_pick_and_place::gripper_state msg;// 추가 (중호)
  
  initServiceClient();
  initSubscribe();

}

// 만약 작동 안 될 경우
OpenManipulatorPickandPlace::~OpenManipulatorPickandPlace()
{
  if (ros::isStarted()) 
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

// OpenManipulatorPickandPlace()
void OpenManipulatorPickandPlace::initServiceClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
  goal_task_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path");
  client = node_handle_.serviceClient<roscpp_tutorials::TwoInts>("add_two_ints");

  // 추가한거
  set_actuator_state_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetActuatorState>("set_actuator_state");
}

// OpenManipulatorPickandPlace()
void OpenManipulatorPickandPlace::initSubscribe()
{
  open_manipulator_states_sub_ = node_handle_.subscribe("states", 10, &OpenManipulatorPickandPlace::manipulatorStatesCallback, this);
  open_manipulator_joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorPickandPlace::jointStatesCallback, this);
  open_manipulator_kinematics_pose_sub_ = node_handle_.subscribe("gripper/kinematics_pose", 10, &OpenManipulatorPickandPlace::kinematicsPoseCallback, this);
  ar_pose_marker_sub_ = node_handle_.subscribe("/ar_pose_marker", 10, &OpenManipulatorPickandPlace::arPoseMarkerCallback, this);
}

// OpenManipulatorPickandPlace()
// initSubscribe()
void OpenManipulatorPickandPlace::manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
{
  if (msg->open_manipulator_moving_state == msg->IS_MOVING)
    open_manipulator_is_moving_ = true;
  else
    open_manipulator_is_moving_ = false;
}

// OpenManipulatorPickandPlace()
// initSubscribe()
void OpenManipulatorPickandPlace::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT_AND_TOOL);
  for (int i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("gripper"))  temp_angle.at(4) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

// OpenManipulatorPickandPlace()
// initSubscribe()
void OpenManipulatorPickandPlace::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);

  present_kinematic_position_ = temp_position;
}

// OpenManipulatorPickandPlace()
// initSubscribe()
void OpenManipulatorPickandPlace::arPoseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
  std::vector<ArMarker> temp_buffer;
  for (int i = 0; i < msg->markers.size(); i ++)
  {
    ArMarker temp;
    temp.id = msg->markers.at(i).id;
    temp.position[0] = msg->markers.at(i).pose.pose.position.x;
    temp.position[1] = msg->markers.at(i).pose.pose.position.y;
    temp.position[2] = msg->markers.at(i).pose.pose.position.z;

    temp_buffer.push_back(temp);
  }

  ar_marker_pose = temp_buffer;
}

// main 함수 timer
void OpenManipulatorPickandPlace::publishCallback(const ros::TimerEvent&)
{
  printText();

 // 추가한 코드
/*
  open_manipulator_msgs::SetActuatorState srv;

  srv.request.set_actuator_state = false;
  set_actuator_state_client_.call(srv) ;
*/

  setModeState();
  
 
  if (mode_state_ == DEMO_START)
  {
    if (!open_manipulator_is_moving_) demoSequence();
  }
  else if (mode_state_ == DEMO_STOP)
  {

  }

}

// main 함수 timer
// publishCallback(const ros::TimerEvent&)
void OpenManipulatorPickandPlace::setModeState()
{
  roscpp_tutorials::TwoInts srv ;

  srv.request.a = 0 ;
  client.call(srv) ;

  int j = srv.response.slave_mode_ ; 

  if (j == 1)
  {
    //open_manipulator_actuator_enabled_ = true;
    mode_state_ = DEMO_START;

  }
  else if (j == 2)
    mode_state_ = DEMO_STOP;
}

// main 함수 timer
// publishCallback(const ros::TimerEvent&)
void OpenManipulatorPickandPlace::printText()
{


/*
  srv.request.joint_1 = present_joint_angle_.at(0);
  srv.request.joint_2 = present_joint_angle_.at(1);
  srv.request.joint_3 = present_joint_angle_.at(2);
  srv.request.joint_4 = present_joint_angle_.at(3);
*/


  system("clear");

  printf("\n");
  printf("-----------------------------\n");
  printf("Pick and Place demonstration!\n");
  printf("-----------------------------\n");

  printf("1 : Pick and Place start\n");
  printf("2 : Pick and Place Stop\n");
  
  // ROS_INFO("Jung-ho' computer already BAKSALs");

   
  roscpp_tutorials::TwoInts srv;
  

  client.call(srv) ;

  ROS_INFO("master: %ld", (long int)srv.response.master);
  
  printf("-----------------------------\n");

  if (mode_state_ == DEMO_START)
  {
    switch(demo_count_)
    {
    case 0: // home pose
      printf("Detect and pick the box_____0\n");
      break;
    case 1:
      printf("___1\n");
      break;
    case 2:
      printf("Detecting...__2\n");
      break;
    case 3:
      printf("place the box_____3\n");
      break;
    case 4:
      printf("Detecting..._______4\n");
      break;
    case 5:
      printf("wait & place________5\n");
      break;
    case 6:
      printf("move up after place the box________6\n");
      break;
    case 7:
      printf("home pose and reconnect to master_open_Manipulator____7\n");
      break;
    }
  }
  else if (mode_state_ == DEMO_STOP)
  {
    printf("The end of Pick and Place\n");
  }

  printf("-----------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         present_joint_angle_.at(0),
         present_joint_angle_.at(1),
         present_joint_angle_.at(2),
         present_joint_angle_.at(3));

  printf("Present Tool Position: %.3lf\n", present_joint_angle_.at(4));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         present_kinematic_position_.at(0),
         present_kinematic_position_.at(1),
         present_kinematic_position_.at(2));
  printf("-----------------------------\n");

  if (ar_marker_pose.size()) printf("AR marker detected.\n");
  for (int i = 0; i < ar_marker_pose.size(); i ++)
  {
    printf("ID: %d --> X: %.3lf\tY: %.3lf\tZ: %.3lf\n",
           ar_marker_pose.at(i).id,
           ar_marker_pose.at(i).position[0],
           ar_marker_pose.at(i).position[1],
           ar_marker_pose.at(i).position[2]);
  }
}

// main 함수 timer
// publishCallback(const ros::TimerEvent&)
bool OpenManipulatorPickandPlace::kbhit()
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


// main 함수 timer
// publishCallback(const ros::TimerEvent&)
void OpenManipulatorPickandPlace::demoSequence()
{
  std::vector<double> joint_angle;
  std::vector<double> kinematics_position;
  std::vector<double> kinematics_orientation;
  std::vector<double> gripper_value;
  open_manipulator_pick_and_place::gripper_state msg;

  switch (demo_count_)
  {
  case 0: // pick the box
    
    for (int i = 0; i < ar_marker_pose.size(); i ++)
    {

      if (ar_marker_pose.at(i).id == pick_ar_id_)
      {

    	///////////////////////////////////////////////
    	roscpp_tutorials::TwoInts srv;

 	srv.request.a = 1;
	grip_state_change = 1;
	msg.grip_state = grip_state_change;
	gripper_state_pub.publish(msg); // 추가 (중호)
	
    	if (client.call(srv))
      	{
        	ROS_INFO("master: %ld", (long int)srv.response.master);
      	}
	ros::Duration(5).sleep();// time delay """"real fix!""""
	
    	///////////////////////////////////////////////

        setJointSpacePath(joint_name_, present_joint_angle_, 3.0);
	gripper_value.push_back(0.010);
	setToolControl(gripper_value);// gripper_open
	
        kinematics_position.push_back(ar_marker_pose.at(i).position[0]);
        kinematics_position.push_back(ar_marker_pose.at(i).position[1]);
        kinematics_position.push_back(0.04);// original state is 0.05
        kinematics_orientation.push_back(0.74);
        kinematics_orientation.push_back(0.00);
        kinematics_orientation.push_back(0.66);
        kinematics_orientation.push_back(0.00);
        setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);
        demo_count_ ++;
        return;
      }
    }
    demo_count_ = 0;  // If the detection fails.
    break;
  case 1: // wait & grip
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.push_back(0.000); // original state : -0.002
    setToolControl(gripper_value);
    demo_count_ ++;
    msg.grip_state = grip_state_change;
    gripper_state_pub.publish(msg); // 추가 (중호)
    break;
  case 2: // initial pose
    joint_angle.push_back( 0.01);
    joint_angle.push_back(-0.80);
    joint_angle.push_back( 0.00);
    joint_angle.push_back( 1.90);
    setJointSpacePath(joint_name_, joint_angle, 2.0);// 1.0   
    demo_count_ ++;
    msg.grip_state = grip_state_change;
    gripper_state_pub.publish(msg); // 추가 (중호)
    break;
  case 3: // place pose
    joint_angle.push_back( 1.57);
    joint_angle.push_back(-0.21);
    joint_angle.push_back(-0.15);
    joint_angle.push_back( 1.89);
    setJointSpacePath(joint_name_, joint_angle, 2.0);// 1.0
    demo_count_ ++;
    msg.grip_state = grip_state_change;
    gripper_state_pub.publish(msg); // 추가 (중호)
    break;
  case 4: // place the box
    kinematics_position.push_back(present_kinematic_position_.at(0));
    kinematics_position.push_back(present_kinematic_position_.at(1));
    if (pick_ar_id_ == 0)  kinematics_position.push_back(present_kinematic_position_.at(2)-0.076);
    else if (pick_ar_id_ == 1)  kinematics_position.push_back(present_kinematic_position_.at(2)-0.041);
    else if (pick_ar_id_ == 2)  kinematics_position.push_back(present_kinematic_position_.at(2)-0.006);
    kinematics_orientation.push_back(0.74);
    kinematics_orientation.push_back(0.00);
    kinematics_orientation.push_back(0.66);
    kinematics_orientation.push_back(0.00);
    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);
    demo_count_ ++;
    msg.grip_state = grip_state_change;
    gripper_state_pub.publish(msg); // 추가 (중호)
    break;
  case 5: // wait & place
    setJointSpacePath(joint_name_, present_joint_angle_, 2.0);// 1.0
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_ ++;
    msg.grip_state = grip_state_change;
    gripper_state_pub.publish(msg); // 추가 (중호)
    break;
  case 6: // move up after place the box
    kinematics_position.push_back(present_kinematic_position_.at(0));
    kinematics_position.push_back(present_kinematic_position_.at(1));
    kinematics_position.push_back(0.135);
    kinematics_orientation.push_back(0.74);
    kinematics_orientation.push_back(0.00);
    kinematics_orientation.push_back(0.66);
    kinematics_orientation.push_back(0.00);
    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);
    demo_count_ ++;
    msg.grip_state = grip_state_change;
    gripper_state_pub.publish(msg); // 추가 (중호)
    break;
  case 7: // home pose
    joint_angle.push_back( 0.00);
    joint_angle.push_back(-1.05);
    joint_angle.push_back( 0.35);
    joint_angle.push_back( 0.70);
    setJointSpacePath(joint_name_, joint_angle, 1.5);
    ////////////////////////

    ros::Duration(5).sleep();// time delay
    grip_state_change = 0;
    demo_count_ = 0; // 수정 원래 여기 없어
    roscpp_tutorials::TwoInts srv;

    srv.request.a = 2;

    if (client.call(srv))
      {
        ROS_INFO("receive master: %ld", (long int)srv.response.master);
	
      }
    ///////////////////////
    
    if (pick_ar_id_ == 0) pick_ar_id_ = 1;
    else if (pick_ar_id_ == 1) pick_ar_id_ = 2;
    else if (pick_ar_id_ == 2)
    {
      pick_ar_id_ = 1;
      demo_count_ = 0;// 수정 
      mode_state_ = DEMO_STOP;//나중에 수정할 부분(상자오면)
    }
    mode_state_ = DEMO_STOP;//넣었음(나중에 수정할 부분 상자오면)
    msg.grip_state = grip_state_change;
    gripper_state_pub.publish(msg); // 추가 (중호)
    break;
  } // end of switch-case

 


}

// main 함수 timer
// +publishCallback(const ros::TimerEvent&)
// +demoSequence()
bool OpenManipulatorPickandPlace::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

// main 함수 timer
// +publishCallback(const ros::TimerEvent&)
// +demoSequence()
bool OpenManipulatorPickandPlace::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back("gripper");
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

// main 함수 timer
// +publishCallback(const ros::TimerEvent&)
// +demoSequence()
bool OpenManipulatorPickandPlace::setTaskSpacePath(std::vector<double> kinematics_pose,std::vector<double> kienmatics_orientation, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";

  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kienmatics_orientation.at(0);
  srv.request.kinematics_pose.pose.orientation.x = kienmatics_orientation.at(1);
  srv.request.kinematics_pose.pose.orientation.y = kienmatics_orientation.at(2);
  srv.request.kinematics_pose.pose.orientation.z = kienmatics_orientation.at(3);

  srv.request.path_time = path_time;

  if (goal_task_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}




int main(int argc, char **argv)
{

  // Init ROS node
  ros::init(argc, argv, "open_manipulator_pick_and_place");
  ros::NodeHandle node_handle("");
  
  OpenManipulatorPickandPlace open_manipulator_pick_and_place ;


  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(0.100)/*100ms*/, &OpenManipulatorPickandPlace::publishCallback, &open_manipulator_pick_and_place);


  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}












