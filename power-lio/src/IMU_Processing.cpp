#include "IMU_Processing.h"

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_vel_scale = scaler;
}

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true)
{
  imu_en = true;
  init_iter_num = 1;
  mean_acc      = V3D(0, 0, 0.0);
  mean_gyr      = V3D(0, 0, 0);
  after_imu_init_ = false;
  state_cov.setIdentity();
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, 0.0);
  mean_gyr      = V3D(0, 0, 0);
  imu_need_init_    = true;
  init_iter_num     = 1;
  after_imu_init_   = false;
  
  time_last_scan = 0.0;
}

void ImuProcess::Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance 初始化重力、陀螺偏差、acc和陀螺协方差
   ** 2. normalize the acceleration measurenments to unit gravity 将加速度测量归一化为单位重力**/
  // V3D tmp_gravity = - mean_acc / mean_acc.norm() * G_m_s2; // state_gravity;
  M3D hat_grav;
  hat_grav << 0.0, gravity_(2), -gravity_(1),
              -gravity_(2), 0.0, gravity_(0),
              gravity_(1), -gravity_(0), 0.0;//根据重力向量构造反对称矩阵
  double align_norm = (hat_grav * tmp_gravity).norm() / gravity_.norm() / tmp_gravity.norm();
  //计算两个向量的叉积的模（即旋转轴的长度），并通过各自的模来标准化。这表示旋转的大小。
  double align_cos = gravity_.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();
  //计算两个向量的点积，得出它们之间夹角的余弦值，表示方向的相似度。
  if (align_norm < 1e-6)//垂直
  {
    if (align_cos > 1e-6)
    {
      rot = Eye3d;
    }
    else
    {
      rot = -Eye3d;
    }
  }
  else
  {
    V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos); 
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));//得到旋转矩阵
  }
}

void ImuProcess::IMU_init(const MeasureGroup &meas, int &N)//记录imu偏置
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance 初始化重力、陀螺仪偏差、加速度和陀螺仪协方差
   ** 2. normalize the acceleration measurenments to unit gravity 将加速度测量标准化为单位重力**/
  ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)//记录imu偏置
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  for (const auto &imu : meas.imu)//求第一个测量包内的全部imu数据的平均
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    N ++;
  }
}

void ImuProcess::Process(const MeasureGroup &meas, PointCloudXYZI::Ptr cur_pcl_un_)
{  
  if (imu_en)
  {
    if(meas.imu.empty())  return;

    if (imu_need_init_)//imu_init
    {
      
      {
        /// The very first lidar frame
        IMU_init(meas, init_iter_num);

        imu_need_init_ = true;

        if (init_iter_num > MAX_INI_COUNT)//用100个
        {
          ROS_INFO("IMU Initializing: %.1f %%", 100.0);
          imu_need_init_ = false;
          *cur_pcl_un_ = *(meas.lidar);
        }
        // *cur_pcl_un_ = *(meas.lidar);
      }
      return;
    }
    if (!after_imu_init_) after_imu_init_ = true;
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
  else
  {
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}