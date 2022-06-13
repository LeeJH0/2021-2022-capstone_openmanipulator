/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "roscpp_tutorials/TwoInts.h"

#define ON 0
#define OFF 1

int slave ;
int master ;
int window ;

int slave_mode ;

float joint_1_ ;
float joint_2_ ;
float joint_3_ ;
float joint_4_ ;



bool add(roscpp_tutorials::TwoInts::Request  &req,
         roscpp_tutorials::TwoInts::Response &res )
{
  system("clear");
  
  ROS_INFO("request: a=%ld", (long int)req.a);
  
  // master 에서 보내주기
  if (req.a == 1)
  {
    master = OFF ;
    window = OFF ;
  }
  else if (req.a == 2) 
  {
    master = ON ;
    window = ON ;
  }
  else if (req.a == 4)
  {
    joint_1_ = req.joint_1 ;
    joint_2_ = req.joint_2 ;
    joint_3_ = req.joint_3 ;
    joint_4_ = req.joint_4 ;
  }
  // slave 에서 보내주기
  else if (req.a == 5) 
  {
    slave_mode = 1 ;
  }
  else if (req.a == 6) 
  {
    slave_mode = 2 ;
  }


  res.master = master ;
  /*
  res.joint_1 = joint_1_ ;
  res.joint_2 = joint_2_ ;
  res.joint_3 = joint_3_ ;
  res.joint_4 = joint_4_ ;
  */
  res.sum = window ;
  
  res.slave_mode_ = slave_mode ;


  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

// %Tag(SERVICE_SERVER)%
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
// %EndTag(SERVICE_SERVER)%

  ros::spin();

  return 0;
}



  // printf("joint_1:%f, joint_2:%f, joint_3:%f, joint_4:%f \n", joint_1_, joint_2_, joint_3_, joint_4_) ;

