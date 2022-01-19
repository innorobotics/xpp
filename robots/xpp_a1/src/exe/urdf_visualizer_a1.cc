/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ros/init.h>

#include <xpp_a1/inverse_kinematics_a1.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_states/joints.h>
#include <xpp_states/endeffector_mappings.h>

#include <xpp_vis/cartesian_joint_converter.h>
#include <xpp_vis/urdf_visualizer.h>

using namespace xpp;
using namespace quad;

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "a1_urdf_visualizer");

  const std::string joint_desired_hyq = "xpp/joint_a1_des";

  auto hyq_ik = std::make_shared<InverseKinematicsA1>();
  CartesianJointConverter inv_kin_converter(hyq_ik,
					    xpp_msgs::robot_state_desired,
					    joint_desired_hyq);

  std::vector<UrdfVisualizer::URDFName> joint_names(12);

  joint_names.at(0) = "FL_hip_joint";
  joint_names.at(1) = "FL_thigh_joint";
  joint_names.at(2) = "FL_calf_joint";

  joint_names.at(3) = "FR_hip_joint";
  joint_names.at(4) = "FR_thigh_joint";
  joint_names.at(5) = "FR_calf_joint";

  joint_names.at(6) = "RL_hip_joint";
  joint_names.at(7) = "RL_thigh_joint";
  joint_names.at(8) = "RL_calf_joint";

  joint_names.at(9) = "RR_hip_joint";
  joint_names.at(10) = "RR_thigh_joint";
  joint_names.at(11) = "RR_calf_joint";

  std::string urdf = "a1_rviz_urdf_robot_description";
  UrdfVisualizer hyq_desired(urdf, joint_names, "trunk", "world",
			     joint_desired_hyq, "a1");

  ::ros::spin();

  return 1;
}

