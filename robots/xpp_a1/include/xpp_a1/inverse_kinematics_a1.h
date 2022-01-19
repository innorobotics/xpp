#ifndef XPP_VIS_INVERSEKINEMATICS_A1_H_
#define XPP_VIS_INVERSEKINEMATICS_A1_H_

#include <xpp_vis/inverse_kinematics.h>

namespace xpp {

/**
 * @brief Inverse kinematics function for the A1 robot.
 */
class InverseKinematicsA1 : public InverseKinematics {
public:
  virtual ~InverseKinematicsA1() = default;

  InverseKinematicsA1()
  {
      hipOffsets <<
      0.1805,   0.047, 0.0,
      0.1805,  -0.047, 0.0, 
      -0.1805,  0.047, 0.0,
      -0.1805, -0.047, 0.0;
  }
  /**
   * @brief Returns joint angles to reach for a specific foot position.
   * @param pos_B  3D-position of the foot expressed in the base frame (B).
   */
  Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const override;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects.
   */
  int GetEECount() const override { return 4; };

private:
  Vector3d legParameters = Vector3d(0.2, 0.2,0.0838);
  Eigen::Matrix<double, 4, 3> hipOffsets;
};

} /* namespace xpp */

#endif
