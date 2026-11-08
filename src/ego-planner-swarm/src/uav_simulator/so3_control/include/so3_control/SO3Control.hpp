#ifndef __SO3_CONTROL_H__
#define __SO3_CONTROL_H__

#include <Eigen/Geometry>

class SO3Control
{
   public:
    SO3Control();

    // set函数,设置控制指令
    void setMass(const double mass);
    void setGravity(const double g);
    void setPosition(const Eigen::Vector3d& position);
    void setVelocity(const Eigen::Vector3d& velocity);
    void setAcc(const Eigen::Vector3d& acc);

    // 计算得到so3控制指令
    void calculateControl(const Eigen::Vector3d& des_pos, const Eigen::Vector3d& des_vel,
                          const Eigen::Vector3d& des_acc, const double des_yaw,
                          const double des_yaw_dot, const Eigen::Vector3d& kx,
                          const Eigen::Vector3d& kv);

    // get函数,获取计算得到的控制指令
    const Eigen::Vector3d&    getComputedForce(void);
    const Eigen::Quaterniond& getComputedOrientation(void);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   private:
    // Inputs for the controller
    double          mass_;
    double          g_;
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d acc_;

    // Outputs of the controller
    Eigen::Vector3d    force_;
    Eigen::Quaterniond orientation_;  // Quaterniond的d是double
};

#endif
