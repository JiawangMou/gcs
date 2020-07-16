#include <Eigen/Dense>

namespace Sensor
{

bool ellipsoidFit(const Eigen::VectorXd &x, const Eigen::VectorXd &y, const Eigen::VectorXd &z,
                  Eigen::Vector3d &offset, Eigen::Vector3d &gain, int mode);

} // namespace Sensor


