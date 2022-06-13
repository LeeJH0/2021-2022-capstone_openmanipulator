/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* 	http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

//library
#include "open_manipulator_master_slave/open_manipulator_master.h"
#include "ros_tutorials_topic/MsgTutorial.h"// MsgTutorial 메시지 파일 헤더(빌드 후 자동 생성됨)
#include "roscpp_tutorials/TwoInts.h"
#include <boost/bind.hpp>



///////////////////////
// Master slave 예제 //
///////////////////////

// OpenmanipulatorMaster class 선언
OpenManipulatorMaster::OpenManipulatorMaster(std::string usb_port, std::string baud_rate)
	:node_handle_(""),
 	priv_node_handle_("~"),
 	service_call_period_(0.01),
 	mode_state_(MASTER_SLAVE_MODE),
 	buffer_index_(0),
	num1(0),
	num2(0),
	onoff(0)
{
  node_handle_ = ros::NodeHandle();


  
  service_call_period_  = priv_node_handle_.param<double>("service_call_period", 0.010f);

  dxl_id_.push_back(priv_node_handle_.param<int32_t>("joint1_id", 1));
  dxl_id_.push_back(priv_node_handle_.param<int32_t>("joint2_id", 2));
  dxl_id_.push_back(priv_node_handle_.param<int32_t>("joint3_id", 3));
  dxl_id_.push_back(priv_node_handle_.param<int32_t>("joint4_id", 4));
  dxl_id_.push_back(priv_node_handle_.param<int32_t>("gripper_id", 5));

  goal_joint_position_.resize(NUM_OF_JOINT);
  goal_tool_position_ = 0.0;
 
  initOpenManipulator(usb_port, baud_rate, service_call_period_);
  initServiceClient();

  //syncOpenManipulator(false);
}

// 오류날때
OpenManipulatorMaster::~OpenManipulatorMaster()
{
  delete actuator_;
  delete tool_;

  if(ros::isStarted()) {
	ros::shutdown();
	ros::waitForShutdown();
  }
}

// OpenManipulatorMaster(std::string usb_port, std::string baud_rate)
void OpenManipulatorMaster::initServiceClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");

  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");

  master_client = node_handle_ .serviceClient<roscpp_tutorials::TwoInts>("add_two_ints");
  roscpp_tutorials::TwoInts srv;
  
  // 추가한거
  set_actuator_state_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetActuatorState>("set_actuator_state");
}

// OpenManipulatorMaster(std::string usb_port, std::string baud_rate)
void OpenManipulatorMaster::initOpenManipulator(STRING usb_port, STRING baud_rate, double service_call_period)
{
  /*****************************************************************************
  ** Initialize Manipulator Parameter
  *****************************************************************************/
  addWorld("world",   // world name
       	"joint1"); // child name

  addJoint("joint1",  // my name
       	"world",   // parent name
       	"joint2",  // child name
       	math::vector3(0.012, 0.0, 0.017),            	// relative position
       	math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
       	Z_AXIS,    	// axis of rotation
       	dxl_id_.at(0), // actuator id
       	M_PI,      	// max joint limit (3.14 rad)
       	-M_PI);    	// min joint limit (-3.14 rad)

  addJoint("joint2",  // my name
       	"joint1",  // parent name
       	"joint3",  // child name
       	math::vector3(0.0, 0.0, 0.0595),             	// relative position
       	math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
       	Y_AXIS,    	// axis of rotation
       	dxl_id_.at(1), // actuator id
       	M_PI_2,    	// max joint limit (1.67 rad)
       	-2.05);    	// min joint limit (-2.05 rad)

  addJoint("joint3",  // my name
       	"joint2",  // parent name
       	"joint4",  // child name
       	math::vector3(0.024, 0.0, 0.128),            	// relative position
       	math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
       	Y_AXIS,    	// axis of rotation
       	dxl_id_.at(2), // actuator id
       	1.53,      	// max joint limit (1.53 rad)
       	-M_PI_2);  	// min joint limit (-1.67 rad)

  addJoint("joint4",  // my name
       	"joint3",  // parent name
       	"gripper", // child name
       	math::vector3(0.124, 0.0, 0.0),              	// relative position
       	math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
       	Y_AXIS,    	// axis of rotation
       	dxl_id_.at(3), // actuator id
       	2.0,       	// max joint limit (2.0 rad)
       	-1.8);     	// min joint limit (-1.8 rad)

  addTool("gripper",  // my name
      	"joint4",   // parent name
      	math::vector3(0.126, 0.0, 0.0),              	// relative position
      	math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
      	dxl_id_.at(4),  // actuator id
      	0.010,      	// max gripper limit (0.01 m)
      	-0.010,     	// min gripper limit (-0.01 m)
      	-0.015);    	// Change unit from `meter` to `radian`

  /*****************************************************************************
  ** Initialize Joint Actuator
  *****************************************************************************/
  actuator_ = new dynamixel::JointDynamixelProfileControl(service_call_period);

  // Set communication arguments
  STRING dxl_comm_arg[2] = {usb_port, baud_rate};
  void *p_dxl_comm_arg = &dxl_comm_arg;

  // Set joint actuator id
  std::vector<uint8_t> jointDxlId;
  jointDxlId.push_back(dxl_id_.at(0));
  jointDxlId.push_back(dxl_id_.at(1));
  jointDxlId.push_back(dxl_id_.at(2));
  jointDxlId.push_back(dxl_id_.at(3));
  addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);

  // Set joint actuator control mode
  STRING joint_dxl_mode_arg = "position_mode";
  void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
  setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

  /*****************************************************************************
  ** Initialize Tool Actuator
  *****************************************************************************/
  tool_ = new dynamixel::GripperDynamixel();

  uint8_t gripperDxlId = dxl_id_.at(4);
  addToolActuator(TOOL_DYNAMIXEL, tool_, gripperDxlId, p_dxl_comm_arg);

  // Set gripper actuator control mode
  STRING gripper_dxl_mode_arg = "current_based_position_mode";
  void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
  setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_mode_arg);

  // Disable All Actuators
  disableAllActuator();
}

// OpenManipulatorMaster(std::string usb_port, std::string baud_rate)
void OpenManipulatorMaster::syncOpenManipulator(bool recorded_state)
{
  log::println("Synchronizing Open Manipulators.", "GREEN");
  double sync_path_time = 1.0;

  receiveAllJointActuatorValue();
  receiveAllToolActuatorValue();

  if(recorded_state) // move to first pose of recorded buffer
  {
    setJointSpacePath(sync_path_time, record_buffer_.at(buffer_index_).joint_angle);
    setToolPath(record_buffer_.at(buffer_index_).tool_position);
  }
  else // move to present master pose
  {
    setJointSpacePath(sync_path_time);
    setToolPath();
  }

  ros::WallDuration sleep_time(sync_path_time);
  sleep_time.sleep();

  return;
}

// OpenManipulatorMaster(std::string usb_port, std::string baud_rate)
// syncOpenManipulator(bool recorded_state)
// + OpenManipulatorMaster::publishCallback(const ros::TimerEvent&)
bool OpenManipulatorMaster::setJointSpacePath(double path_time, std::vector<double> set_goal_joint_position)
{
  auto joint_name = getManipulator()->getAllActiveJointComponentName();
  std::vector<double> joint_value;
  if(set_goal_joint_position.size())  joint_value = set_goal_joint_position;
  else
  {
	joint_value.push_back(getJointValue(joint_name.at(0)).position);
	joint_value.push_back(getJointValue(joint_name.at(1)).position);
	joint_value.push_back(getJointValue(joint_name.at(2)).position);
	joint_value.push_back(getJointValue(joint_name.at(3)).position);
  }

  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;

  for(int i = 0; i < NUM_OF_JOINT; i ++)
  {
	if(getManipulator()->checkJointLimit(joint_name.at(i), joint_value.at(i)))
  	srv.request.joint_position.position.push_back(joint_value.at(i));
	else
  	srv.request.joint_position.position.push_back(goal_joint_position_.at(i));
  }

  goal_joint_position_ = srv.request.joint_position.position;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
	return srv.response.is_planned;
  }
  return false;
}




// OpenManipulatorMaster(std::string usb_port, std::string baud_rate)
// syncOpenManipulator(bool recorded_state)
// + OpenManipulatorMaster::publishCallback(const ros::TimerEvent&)
bool OpenManipulatorMaster::setToolPath(double set_goal_tool_position)
{
  double tool_value;
  if(set_goal_tool_position < -0.1)
	tool_value = getAllToolValue().at(0).position;
  else
	tool_value = set_goal_tool_position;

  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back("gripper");

  if(getManipulator()->checkJointLimit("gripper", tool_value))
	srv.request.joint_position.position.push_back(tool_value);
  else
	srv.request.joint_position.position.push_back(goal_tool_position_);

  goal_tool_position_ = srv.request.joint_position.position.at(0);

  if(goal_tool_control_client_.call(srv))
  {
	return srv.response.is_planned;
  }

  return false;
}

// main 함수 timer
void OpenManipulatorMaster::publishCallback(const ros::TimerEvent&)
{
  // ??: 굳이 count를 왜 넣는 거지? / 어차피 돌아갈 것 같은데
  static int count = 0;
  if(!(count % 5))
  {
    printText();

    if(kbhit())
      // 입력받기
      setModeState(std::getchar());
     
    
  }
  count ++;
  // 여기서 상대방의 정보 받기
  receiveAllJointActuatorValue();
  receiveAllToolActuatorValue();

/*
  // topic 받기
  ros_tutorial_sub = node_handle_.subscribe<ros_tutorials_topic::MsgTutorial>("ros_tutorial_msg", 100, boost::bind(&OpenManipulatorMaster::msgCallback, this, _1));

 ROS_INFO("recieve msg = %d", onoff);

*/

  roscpp_tutorials::TwoInts srv;
  
  srv.request.a = 3 ; 
  master_client.call(srv) ;

  
  if(srv.response.master == 0) 
  {
	setJointSpacePath(service_call_period_);
	setToolPath();
  }
  else if ( srv.response.master != 0) 
  {
  // 추가한거
/*
  open_manipulator_msgs::SetActuatorState srv;

  srv.request.set_actuator_state = false;
  set_actuator_state_client_.call(srv) ;
*/
    
  }
 
}

// main 함수 timer
// publishCallback(const ros::TimerEvent&)
void OpenManipulatorMaster::msgCallback(const ros_tutorials_topic::MsgTutorialConstPtr& msg_master)
{
 num1 = msg_master->stamp.sec ;
 num2 = msg_master->stamp.nsec ;
 onoff = msg_master->num ;
}

// main 함수 timer
// publishCallback(const ros::TimerEvent&)
bool OpenManipulatorMaster::kbhit()
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
void OpenManipulatorMaster::setModeState(char ch)
{
  if(ch == '1')
  {
    syncOpenManipulator(false);
    mode_state_ = MASTER_SLAVE_MODE;
  }
  else if(ch == '2')
  {
   // syncOpenManipulator(false);
   // mode_state_ = MASTER_SLAVE_MODE;
    mode_state_ = 2;

    roscpp_tutorials::TwoInts srv;

    srv.request.a = 5 ; 
    master_client.call(srv) ;

  }
  else if(ch == '3')
  {
    // syncOpenManipulator(false);
    // mode_state_ = MASTER_SLAVE_MODE;
    mode_state_ = 3;


    roscpp_tutorials::TwoInts srv;

    srv.request.a = 6 ; 
    master_client.call(srv) ;
  }
}

// main 함수 timer
// publishCallback(const ros::TimerEvent&)
void OpenManipulatorMaster::printText()
{
  system("clear");

  roscpp_tutorials::TwoInts srv;
  master_client.call(srv) ;

  srv.request.a = 4 ; 

  printf("\n");
  printf("-----------------------------\n");
  printf("Control Your OpenManipulator!\n");

  printf("-----------------------------\n");
  printf("Present Control Mode\n");
  printf("1. Master - Slave Mode\n") ;
  printf("2. Slave - detecting start \n") ;
  printf("3. Slave - detecting stop  \n") ;

  if(mode_state_ == MASTER_SLAVE_MODE)
  {
	printf("-----------------------------\n");
	printf("-----------------------------\n");
	printf("Master - Slave Mode\n");
	printf("-----------------------------\n");
	printf("Present Master-Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
     		goal_joint_position_.at(0),
    	 	goal_joint_position_.at(1),
     		goal_joint_position_.at(2),
     		goal_joint_position_.at(3));
  	printf("Present Slave-Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
     		srv.response.joint_1,
     		srv.response.joint_2,
     		srv.response.joint_3,
     		srv.response.joint_4);
  	printf("Present Tool Position: %.3lf\n", goal_tool_position_);
  	printf("-----------------------------\n");
  }
  
  if(mode_state_ == 2)
  {
	printf("-----------------------------\n");
	printf("-----------------------------\n");
	printf("Slave - detecting Mode\n");
	printf("-----------------------------\n");
	printf("DETECTING... ") ;

	if (srv.response.master == 1) {
		printf("!!!!!!!!!!! \n") ;
		printf("-----------------------------\n");
		printf("Pick and Place !!!\n") ;
  	}
  }

  if(mode_state_ == 3) {
	printf("-----------------------------\n");
	printf("-----------------------------\n");
	printf("Slave - detecting stop \n") ;
	printf("-----------------------------\n");
	printf("Press '1' !!\n") ;
	}

 //  if (srv.response.master == 0) printf("Detecting... \n") ;

  printf("\n-----------------------------\n");
  ROS_INFO("master: %ld", (long int)srv.response.master);  
}




///// Main 함수 (Master slave)/////
int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_master_slave");
  ros::NodeHandle node_handle("");

  std::string usb_port = "/dev/ttyUSB0";
  std::string baud_rate = "1000000";

  if (argc < 3)
  {
	log::error("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
	return 0;
  }
  else
  {
	usb_port = argv[1];
	baud_rate = argv[2];
  }
 
  OpenManipulatorMaster open_manipulator_master(usb_port, baud_rate);

  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(open_manipulator_master.getServiceCallPeriod()), &OpenManipulatorMaster::publishCallback, &open_manipulator_master);


  while (ros::ok())
  {
	ros::spinOnce();
  }

  return 0;
}


