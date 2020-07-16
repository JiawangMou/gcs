#include <Eigen/Dense>
#include <iostream>

// int main(){

//     int num = 628;
//     Eigen::VectorXd x, y, z;
//     x.resize(num);
//     y.resize(num);
//     z.resize(num);
    
//     //file read
//     ifstream fin("imu_data.txt");
//     char tmp[100];
//     double ax, ay, az, gx, gy, gz, mx, my, mz;
//     int count = 0;
//     while(fin.getline(tmp, 100, '\n')){
//         sscanf(tmp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
//         x(count) = mx;
//         y(count) = my;
//         z(count) = mz;
//         count ++;
//         // cout << mx << " " << my << " " << mz << " " << ++count << endl;
//     }
//     Eigen::Vector3d offset;
//     Eigen::Vector3d gain;

//     ellipsoidFit(x, y, z, offset, gain, 1);
    
//     cout << offset << endl << gain << endl;
//     return 0;
// }
namespace Sensor
{

bool ellipsoidFit(const Eigen::VectorXd &x, const Eigen::VectorXd &y, const Eigen::VectorXd &z,
                  Eigen::Vector3d &offset, Eigen::Vector3d &gain, int mode){
    
    int num = x.size();

    if(x.size() != y.size() || y.size() != z.size()){
        return false;
    }
    
    Eigen::MatrixXd D;
    D.resize(num, 6);
    switch(mode){
        case(1):    //XYZ axes (non-rotated ellipsoid)
            D.resize(num, 6);
            D << x.array()*x.array(),   y.array()*y.array(),    z.array()*z.array(),
                 2*x.array(),           2*y.array(),            2*z.array();
        break;

        case(2):    //(1) and radius x=y
            D.resize(num, 5);
            D << x.array()*x.array() + y.array()*y.array(),     z.array()*z.array(),
                 2*x.array(),           2*y.array(),            2*z.array();
        break;

        case(3):    //(1) and radius x=z
            D.resize(num, 5);
            D << x.array()*x.array() + z.array()*z.array(),     y.array()*y.array(),
                 2*x.array(),           2*y.array(),            2*z.array();
        break;

        case(4):    //(1) and radius y=z
            D.resize(num, 5);
            D << y.array()*y.array() + z.array()*z.array(),     x.array()*x.array(),
                 2*x.array(),           2*y.array(),            2*z.array();
        break;

        case(5):    //(1) and radius x=y=z (sphere)
            D.resize(num, 5);
            D << x.array()*x.array() + y.array()*y.array(),     z.array()*z.array(),
                 2*x.array(),           2*y.array(),            2*z.array();
        break;

        default:
            std::cerr << "Wrong mode number." << std::endl;
            return 0;
        break;
    }

    //Least square fitting (Matrix inversing)
    Eigen::VectorXd v;
    v = (D.transpose() * D).inverse() * (D.transpose() * Eigen::VectorXd::Ones(num));
    
    //Result arranging
    Eigen::VectorXd v_r;
    v_r.resize(6);
    switch(mode){
        case(1):
            v_r << v;
        break;

        case(2):
            v_r << v(0),v(0),v(1),v(2),v(3),v(4);
        break;

        case(3):
            v_r << v(0),v(1),v(0),v(2),v(3),v(4);
        break;

        case(4):
            v_r << v(1),v(0),v(0),v(2),v(3),v(4);
        break;

        case(5):
            v_r << v(0),v(0),v(0),v(1),v(2),v(3);
        break;
    }
    
    offset << - v_r(3) / v_r(0),
              - v_r(4) / v_r(1),
              - v_r(5) / v_r(2);
    double g = 1 + v_r(3) * v_r(3) / v_r(0) + v_r(4) * v_r(4) / v_r(1) + v_r(5) * v_r(5) / v_r(2);

    gain << sqrt(g / v_r(0)),
            sqrt(g / v_r(1)),
            sqrt(g / v_r(2));

    return true;
}
    
} // namespace Sensor