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

#include <xpp_a1/inverse_kinematics_a1.h>

#include <xpp_states/cartesian_declarations.h>
#include <cmath>

namespace xpp {

Joints
InverseKinematicsA1::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
  Vector3d ee_pos_H; // foothold expressed in hip frame
  
  std::vector<Eigen::VectorXd> q_vec;

  // make sure always exactly 4 elements
  auto pos_B = x_B.ToImpl();
  pos_B.resize(4, pos_B.front());

  for (int ee=0; ee<pos_B.size(); ++ee) {
    ee_pos_H = pos_B.at(ee)-hipOffsets.row(ee).transpose();

    double l_up = legParameters(0);
    double l_low = legParameters(1);
    double l_hip = legParameters(2) * pow(-1,ee + 2);

    double x = ee_pos_H(0), y = ee_pos_H(1), z = ee_pos_H(2);

    double theta_knee = -acos((pow(x,2) + pow(y,2) + pow(z,2) - pow(l_hip,2) - pow(l_low,2) - pow(l_up,2)) / (2 * l_low * l_up));

    double l = sqrt(pow(l_up,2) + pow(l_low,2) + 2 * l_up * l_low * cos(theta_knee));

    double theta_hip = asin(-x / l) - theta_knee / 2;
    double c1 = l_hip * y - l * cos(theta_hip + theta_knee / 2) * z;

    double s1 = l * cos(theta_hip + theta_knee / 2) * y + l_hip * z;
    double theta_ab = atan2(s1, c1);
    
    //q_vec.push_back(Vector3d(0.0, 0.0, 0.3));
    q_vec.push_back(Vector3d(theta_ab, theta_hip, theta_knee));
  }

  return Joints(q_vec);
}

} /* namespace xpp */
