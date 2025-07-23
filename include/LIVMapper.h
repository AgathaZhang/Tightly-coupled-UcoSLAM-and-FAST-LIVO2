/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <vikit/camera_loader.h>

#include "ceres_icp_tool.h"

#include <std_msgs/String.h>
#include <fast_livo/ImagePoseTime.h>

//#include "run_kiss_matcher.hpp"
/**
//========kiss-matcher start ========
#include <kiss_matcher/FasterPFH.hpp>
#include <kiss_matcher/GncSolver.hpp>
#include <kiss_matcher/KISSMatcher.hpp>
//========kiss-matcher end ========
*/

#include "teaser_cpp_fpfh.hpp"
#define USE_KISS_MATCH 0
#define USE_TEASER_MATCH 0
#define USE_LOOP_PARAM 1

//******************** backend  start ******************************
class EigenPose {
public:
    EigenPose() {
        R_ << 1.0, .0, .0, .0, 1.0, .0, .0, .0, 1.0;
        t_ << .0, .0, .0;
    }

    EigenPose(double x, double y, double z, double roll, double pitch, double yaw){
        t_ << x, y, z;
        Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
                               Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())) *
                               Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
        R_ = q.toRotationMatrix();
    }

    EigenPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
        R_ = q.toRotationMatrix();
        t_ = t;
    }

    EigenPose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) : R_(R), t_(t) {}

    EigenPose(const EigenPose& e_pose)
    {
        R_ = e_pose.R_;
        t_ = e_pose.t_;
    }


/*        EigenPose(const geometry_msgs::Pose& msg) {
            Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
            Eigen::Vector3d t;
            t << msg.position.x, msg.position.y, msg.position.z;
            R_ = q.toRotationMatrix();
            t_ = t;
        }*/
    EigenPose(Eigen::Affine3f pose) {
        R_ = pose.linear().cast<double>();
        t_ = pose.translation().cast<double>();
        //    float x, y, z, roll, pitch, yaw;
        //    pcl::getTranslationAndEulerAngles(pose, x, y, z, roll, pitch, yaw);
        //    t_ << x, y, z;
        //    Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
        //                           Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())) *
        //                           Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
        //    R_ = q.toRotationMatrix();
    }
    /**
    EigenPose(const gtsam::Pose3& pose) {
        t_ << pose.x(), pose.y(), pose.z();
        Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(pose.rotation().yaw(), Eigen::Vector3d::UnitZ())) *
                               Eigen::Quaterniond(Eigen::AngleAxisd(pose.rotation().pitch(), Eigen::Vector3d::UnitY())) *
                               Eigen::Quaterniond(Eigen::AngleAxisd(pose.rotation().roll(), Eigen::Vector3d::UnitX()));
        R_ = q.toRotationMatrix();
    }
    **/

    void SetFloat6(float pose_in[]) {
        t_[0] = pose_in[3];
        t_[1] = pose_in[4];
        t_[2] = pose_in[5];
        Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(pose_in[2], Eigen::Vector3d::UnitZ())) *
                               Eigen::Quaterniond(Eigen::AngleAxisd(pose_in[1], Eigen::Vector3d::UnitY())) *
                               Eigen::Quaterniond(Eigen::AngleAxisd(pose_in[0], Eigen::Vector3d::UnitX()));
        R_ = q.toRotationMatrix();
    }
    void GetFloat6(float pose_out[]) {
        pose_out[0] = GetRPY()[0];
        pose_out[1] = GetRPY()[1];
        pose_out[2] = GetRPY()[2];
        pose_out[3] = t_[0];
        pose_out[4] = t_[1];
        pose_out[5] = t_[2];
    }

    void Restrict2D() {
        Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(GetRPY()[2], Eigen::Vector3d::UnitZ()));
        R_ = q.toRotationMatrix();
    }
    bool IsIdentity() {
        double dis = t_.norm() + GetRPY().norm();
        return dis < 0.00001;
    }

    EigenPose Get2DPose() { return EigenPose(t_[0], t_[1], 0, 0, 0, GetRPY()[2]); }

    EigenPose& operator=(const EigenPose& e_pose){
        R_ = e_pose.R_;
        t_ = e_pose.t_;
    }

    EigenPose operator*(const EigenPose& T_rel) { return EigenPose(R_ * T_rel.R_, R_ * T_rel.t_ + t_); }

    Eigen::Vector3d operator*(const Eigen::Vector3d& point_in){
        return R_*point_in+t_;
    }

    //  Eigen::Vector3d GetRPY() const
    //  {
    //    Eigen::Vector3d YPR = R_.eulerAngles(2, 1, 0);
    //    Eigen::Vector3d RPY;
    //    RPY << YPR(2), YPR(1), YPR(0);
    //    return RPY;
    //  }

    Eigen::Vector3d GetRPY() const {
        Eigen::Vector3d RPY;
        RPY(1) = asin(-R_(2, 0));
        double c_pitch = cos(RPY(1));
        RPY(0) = atan2(R_(2, 1) / c_pitch, R_(2, 2) / c_pitch);
        RPY(2) = atan2(R_(1, 0) / c_pitch, R_(0, 0) / c_pitch);
        return RPY;
    }

    Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d R) const{

        double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
        bool singular = sy < 1e-6;
        double x, y, z;
        if (!singular)
        {
            x = atan2( R(2,1), R(2,2));
            y = atan2(-R(2,0), sy);
            z = atan2( R(1,0), R(0,0));
        }
        else
        {
            x = atan2(-R(1,2), R(1,1));
            y = atan2(-R(2,0), sy);
            z = 0;
        }
        return {x, y, z};
    }

    void printRPY() const {
        Eigen::Vector3d eulerAngle1 = rotationMatrixToEulerAngles( R_ ); // ZYX顺序，yaw,pitch,roll
        std::cout << "r p y :" << eulerAngle1[0] << " " << eulerAngle1[1]
                  << " " << eulerAngle1[2] << std::endl << std::endl;
        /* std::cout << "roll_1 pitch_1 yaw_1 = " << eulerAngle1[2] << " " << eulerAngle1[1]
              << " " << eulerAngle1[0] << std::endl << std::endl;*/

    }

    EigenPose inverse() { return EigenPose(R_.inverse(), -R_.inverse() * t_); }

    //EigenPose CalRelativeEigenPose(const EigenPose& pose_target);

    /* geometry_msgs::Quaternion GetGeoQ() {
         geometry_msgs::Quaternion q;
         Eigen::Quaterniond eigen_q(R_);
         q.w = eigen_q.w();
         q.x = eigen_q.x();
         q.y = eigen_q.y();
         q.z = eigen_q.z();
         return q;
     }*/

    void norm_axis(Eigen::Vector3d& axis){
        double axis_norm = axis.norm();
        axis/=axis_norm;
    }


    void rotateArbitraryAxis(Eigen::Matrix4d& pOut, Eigen::Vector3d axis, float theta) {
        norm_axis(axis);
        float u = axis.x();
        float v = axis.y();
        float w = axis.z();


        pOut(0,0) = cosf(theta) + (u * u) * (1 - cosf(theta));
        pOut(0,1) = u * v * (1 - cosf(theta)) + w * sinf(theta);
        pOut(0,2) = u * w * (1 - cosf(theta)) - v * sinf(theta);
        pOut(0,3) = 0;

        pOut(1,0) = u * v * (1 - cosf(theta)) - w * sinf(theta);
        pOut(1,1)= cosf(theta) + v * v * (1 - cosf(theta));
        pOut(1,2) = w * v * (1 - cosf(theta)) + u * sinf(theta);
        pOut(1,3) = 0;

        pOut(2,0) = u * w * (1 - cosf(theta)) + v * sinf(theta);
        pOut(2,1) = v * w * (1 - cosf(theta)) - u * sinf(theta);
        pOut(2,2) = cosf(theta) + w * w * (1 - cosf(theta));
        pOut(2,3) = 0;

        pOut(3,0) = 0;
        pOut(3,1) = 0;
        pOut(3,2) = 0;
        pOut(3,3) = 1;
    }

    void rotateArbitraryLine(Eigen::Matrix4d& pOut, Eigen::Vector3d& v1, Eigen::Vector3d& v2, float theta)
    {
        float a = v1.x();
        float b = v1.y();
        float c = v1.z();

        Eigen::Vector3d p = v2 - v1;
        norm_axis(p);
        float u = p.x();
        float v = p.y();
        float w = p.z();

        float uu = u * u;
        float uv = u * v;
        float uw = u * w;
        float vv = v * v;
        float vw = v * w;
        float ww = w * w;
        float au = a * u;
        float av = a * v;
        float aw = a * w;
        float bu = b * u;
        float bv = b * v;
        float bw = b * w;
        float cu = c * u;
        float cv = c * v;
        float cw = c * w;

        float costheta = cosf(theta);
        float sintheta = sinf(theta);

        pOut(0,0) = uu + (vv + ww) * costheta;
        pOut(0,1) = uv * (1 - costheta) + w * sintheta;
        pOut(0,2) = uw * (1 - costheta) - v * sintheta;
        pOut(0,3) = 0;

        pOut(1,0) = uv * (1 - costheta) - w * sintheta;
        pOut(1,1) = vv + (uu + ww) * costheta;
        pOut(1,2) = vw * (1 - costheta) + u * sintheta;
        pOut(1,3) = 0;

        pOut(2,0) = uw * (1 - costheta) + v * sintheta;
        pOut(2,1) = vw * (1 - costheta) - u * sintheta;
        pOut(2,2) = ww + (uu + vv) * costheta;
        pOut(2,3) = 0;

        pOut(3,0) = (a * (vv + ww) - u * (bv + cw)) * (1 - costheta) + (bw - cv) * sintheta;
        pOut(3,1) = (b * (uu + ww) - v * (au + cw)) * (1 - costheta) + (cu - aw) * sintheta;
        pOut(3,2) = (c * (uu + vv) - w * (au + bv)) * (1 - costheta) + (av - bu) * sintheta;
        pOut(3,3) = 1;
    }

    void rotateArbitraryLine2(Eigen::Matrix4d& pOut, Eigen::Vector3d& v1, Eigen::Vector3d& v2, float theta)
    {
        float a = v1.x();
        float b = v1.y();
        float c = v1.z();


        Eigen::Vector3d p = v2 - v1;

        //std::cout<<"--> p1"<<std::endl<< p <<std::endl;
        norm_axis(p);
        //std::cout<<"--> p2"<<std::endl<< p <<std::endl;


        float a1 = p[0];
        float b1 = p[1];
        float c1 = p[2];

        //std::cout<<"a1:"<<a1<<std::endl;
        //std::cout<<"b1:"<<b1<<std::endl;
        //std::cout<<"c1:"<<c1<<std::endl;
        Eigen::Matrix4d T;
        T(0,0) = 1;T(0,1) = 0;T(0,2) = 0;T(0,3) = -v1.x();
        T(1,0) = 0;T(1,1) = 1;T(1,2) = 0;T(1,3) = -v1.y();
        T(2,0) = 0;T(2,1) = 0;T(2,2) = 1;T(2,3) = -v1.z();
        T(3,0) = 0;T(3,1) = 0;T(3,2) = 0;T(3,3) = 1;

        //std::cout<<"--> T"<<std::endl<< T <<std::endl;

        float d1 = sqrt(b1*b1+c1*c1);
        Eigen::Matrix4d R_x_alpha;
        R_x_alpha(0,0) = 1;R_x_alpha(0,1) = 0;    R_x_alpha(0,2) = 0;     R_x_alpha(0,3) = 0;
        R_x_alpha(1,0) = 0;R_x_alpha(1,1) = c1/d1;R_x_alpha(1,2) = -b1/d1;R_x_alpha(1,3) = 0;
        R_x_alpha(2,0) = 0;R_x_alpha(2,1) = b1/d1;R_x_alpha(2,2) = c1/d1; R_x_alpha(2,3) = 0;
        R_x_alpha(3,0) = 0;R_x_alpha(3,1) = 0;    R_x_alpha(3,2) = 0;     R_x_alpha(3,3) = 1;

        //std::cout<<"--> R_x_alpha"<<std::endl<< R_x_alpha <<std::endl;
        Eigen::Matrix4d R_y_beta;
        R_y_beta(0,0) = d1;R_y_beta(0,1) = 0;    R_y_beta(0,2) = -a1;     R_y_beta(0,3) = 0;
        R_y_beta(1,0) = 0;R_y_beta(1,1) = 1;     R_y_beta(1,2) = 0;       R_y_beta(1,3) = 0;
        R_y_beta(2,0) = a1;R_y_beta(2,1) = 0;    R_y_beta(2,2) = d1;                 R_y_beta(2,3) = 0;
        R_y_beta(3,0) = 0;R_y_beta(3,1) = 0;     R_y_beta(3,2) = 0;       R_y_beta(3,3) = 1;

        //std::cout<<"--> R_y_beta"<<std::endl<< R_y_beta <<std::endl;
        Eigen::Matrix4d R_z_theta;
        R_z_theta(0,0) = cosf(theta);   R_z_theta(0,1) = -sinf(theta);   R_z_theta(0,2) = 0;       R_z_theta(0,3) = 0;
        R_z_theta(1,0) = sinf(theta);   R_z_theta(1,1) = cosf(theta);    R_z_theta(1,2) = 0;       R_z_theta(1,3) = 0;
        R_z_theta(2,0) = 0;             R_z_theta(2,1) = 0;              R_z_theta(2,2) = 1;       R_z_theta(2,3) = 0;
        R_z_theta(3,0) = 0;             R_z_theta(3,1) = 0;              R_z_theta(3,2) = 0;       R_z_theta(3,3) = 1;

        //std::cout<<"--> R_z_theta"<<std::endl<< R_z_theta <<std::endl;
        Eigen::Matrix4d full_R = T.inverse()*R_x_alpha.inverse()*R_y_beta.inverse()*R_z_theta*R_y_beta*R_x_alpha*T;

        //std::cout<<"full_R:"<<std::endl<<full_R<<std::endl;
        pOut(0,0) = full_R(0,0);
        pOut(0,1) = full_R(0,1);
        pOut(0,2) = full_R(0,2);
        pOut(0,3) = full_R(0,3);

        pOut(1,0) = full_R(1,0);
        pOut(1,1) = full_R(1,1);
        pOut(1,2) = full_R(1,2);
        pOut(1,3) = full_R(1,3);

        pOut(2,0) = full_R(2,0);
        pOut(2,1) = full_R(2,1);
        pOut(2,2) = full_R(2,2);
        pOut(2,3) = full_R(2,3);

        pOut(3,0) = full_R(3,0);
        pOut(3,1) = full_R(3,1);
        pOut(3,2) = full_R(3,2);
        pOut(3,3) = full_R(3,3);
    }

    EigenPose getEigenPoseByMatrix4d(Eigen::Matrix4d pOut){
        Eigen::Matrix3d r;
        Eigen::Vector3d t;
        r(0,0) = pOut(0,0);
        r(0,1) = pOut(0,1);
        r(0,2) = pOut(0,2);

        r(1,0) = pOut(1,0);
        r(1,1) = pOut(1,1);
        r(1,2) = pOut(1,2);

        r(2,0) = pOut(2,0);
        r(2,1) = pOut(2,1);
        r(2,2) = pOut(2,2);

        t[0] = pOut(0,3);
        t[1] = pOut(1,3);
        t[2] = pOut(2,3);

        EigenPose ep(r,t);
        return ep;
    }

    Eigen::Quaterniond GetQ() {
        Eigen::Quaterniond eigen_q(R_);
        return eigen_q;
    }

    double DisTo(EigenPose& other) {
        EigenPose delta = this->inverse() * other;
        return delta.Norm();
    }

    double Norm() { return this->GetRPY().norm() + t_.norm(); }

    double get_dis() { return t_.norm() + GetRPY().norm(); }

    bool IsStill() { return get_dis() < 0.0003; }

    void GetEuler(float* data) {
        data[0] = GetRPY()[0];
        data[1] = GetRPY()[1];
        data[2] = GetRPY()[2];
        data[3] = t_[0];
        data[4] = t_[1];
        data[5] = t_[2];
    }

    EigenPose GetRatioPose(double ratio) {
        return EigenPose(ratio * t_[0], ratio * t_[1], ratio * t_[2], ratio * GetRPY()[0], ratio * GetRPY()[1], ratio * GetRPY()[2]);
    }

    void Reset(const double& x = .0, const double& y = .0, const double& z = .0, const double& roll = .0, const double& pitch = .0, const double& yaw = .0);
    void Reset(const Eigen::Vector3d& tran, const Eigen::Vector3d& rot) { Reset(tran.x(), tran.y(), tran.z(), rot.x(), rot.y(), rot.z()); }

    Eigen::Matrix3d R_;
    Eigen::Vector3d t_;

    friend std::ostream& operator<<(std::ostream& os, const EigenPose& pose) {
        os << "x:" << pose.t_[0] << " y:" << pose.t_[1] << " z:" << pose.t_[2] << " roll:" << pose.GetRPY()[0] << " pitch:" << pose.GetRPY()[1]
           << " yaw:" << pose.GetRPY()[2];
        return os;
    }
    std::string GetPrint() {
        char buff[200] = {};
        sprintf(buff, "x:%f y:%f z:%f roll:%f pitch:%f yaw:%f \n", t_[0], t_[1], t_[2], GetRPY()[0], GetRPY()[1], GetRPY()[2]);
        return std::string(buff);
    }


    void print_angle()
    {
        int pi = 3.14159265359;
        double rd=(GetRPY()[0]/ pi * 180.0);
        double pd=(GetRPY()[1]/ pi * 180.0);
        double yd=(GetRPY()[2]/ pi * 180.0);

        std::cout<<"deg -> roll："<< rd <<" pitch:"<<pd <<" yaw:"<< yd<<std::endl;
    }


};
class LoopDetectFactor{
    public:
        int curr_lidar_loop_id;
        int last_lidar_loop_id;
        LoopDetectFactor(){

        }

        LoopDetectFactor(int curr_lidar_loop_id_, int last_lidar_loop_id_){
            curr_lidar_loop_id = curr_lidar_loop_id_;
            last_lidar_loop_id = last_lidar_loop_id_;
        }

        ~LoopDetectFactor(){
            
        }
};

class ImgLidarFrame{
    public:
        //图像时间戳
        double img_timestamp;
        //雷达时间戳
        double lidar_timestamp;
        //关联的雷达关键帧的id
        int lidar_keyframe_id;
        //是否是雷达关键帧，雷达的时间和相机的时间相差(30ms以内)
        int is_keyframe;
        //当前相机帧到雷达关键帧的转换
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
        ImgLidarFrame(double img_timestamp_p,double lidar_timestamp_p,int lidar_keyframe_id_p,int is_keyframe_p,Eigen::Quaterniond q_,Eigen::Vector3d t_){
            std::cout<<"line:"<<__LINE__<<std::endl;
            img_timestamp = img_timestamp_p;
            lidar_timestamp = lidar_timestamp_p;
            lidar_keyframe_id = lidar_keyframe_id_p;
            is_keyframe = is_keyframe_p;
              std::cout<<"line:"<<__LINE__<<std::endl;
            q.x() = q_.x();
            q.y() = q_.y();
            q.z() = q_.z();
            q.w() = q_.w();
            t[0] = t_[0];
            t[1] = t_[1];
            t[2] = t_[2];
              std::cout<<"line:"<<__LINE__<<std::endl;
        }

        ImgLidarFrame(){
                
        }

        ~ImgLidarFrame(){
            
        }
};

typedef struct other_no_keyframe{
    double time;
    Eigen::Quaterniond q_to_current_keyframe;
    Eigen::Vector3d t_to_current_keyframe;

    
    /**
     // 显式声明默认拷贝构造函数和赋值操作符
    other_no_keyframe() = default;
    other_no_keyframe(const other_no_keyframe&) = default;
    other_no_keyframe& operator=(const other_no_keyframe&) = default;
    */

   //// 显式实现拷贝操作
   // Other_No_Keyframe(const Other_No_Keyframe& other) = default;
   // Other_No_Keyframe& operator=(const Other_No_Keyframe& other) = default;
    /**

    // 默认构造函数
    other_no_keyframe() = default;

    // 手动实现拷贝构造函数
    other_no_keyframe(const other_no_keyframe& other) 
        : time(other.time) {
            q_to_current_keyframe.x() = other.q_to_current_keyframe.x();
            q_to_current_keyframe.y() = other.q_to_current_keyframe.y();
            q_to_current_keyframe.z() = other.q_to_current_keyframe.z();
            q_to_current_keyframe.w() = other.q_to_current_keyframe.w();

            t_to_current_keyframe = other.t_to_current_keyframe;
          }

    // 手动实现拷贝赋值运算符
    other_no_keyframe& operator=(const other_no_keyframe& other) {
        if (this != &other) { // 避免自赋值
            time = other.time;
            q_to_current_keyframe.x() = other.q_to_current_keyframe.x();
            q_to_current_keyframe.y() = other.q_to_current_keyframe.y();
            q_to_current_keyframe.z() = other.q_to_current_keyframe.z();
            q_to_current_keyframe.w() = other.q_to_current_keyframe.w();

            t_to_current_keyframe = other.t_to_current_keyframe;
        }
        return *this;
    }
    */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 默认构造函数
    other_no_keyframe() = default;
    // 显式实现拷贝操作
    other_no_keyframe(const other_no_keyframe& other) = default;
    other_no_keyframe& operator=(const other_no_keyframe& other) = default;
}Other_No_Keyframe;

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D     
    PCL_ADD_INTENSITY;  
    float roll;         
    float pitch;
    float yaw;
    double time;
    std::vector<Other_No_Keyframe, Eigen::aligned_allocator<Other_No_Keyframe>> other_buffer;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                    

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

// gstam的头文件
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <pcl/registration/icp.h>
#include <std_srvs/SetBool.h>

//#include "hba/hba.h"
//******************** backend  end ******************************
class LIVMapper
{

//******************** backend  start ******************************
public:
std::shared_ptr< std::thread >        loop_thread_boot;
    std::string full_camera_trajectory;
    std::string full_camera_trajectory_str;


    //回环检测可以调节参数 start
   int loopdetect_enable = 1;
   int loopdetect_enable_just_final = 0;//#是否仅进行最后一次回环
   
   int use_first_localization_constraint = 0;//#默认不开启

   float ld_loopdetect_scale = 1.0;// #回环检测模块的缩放比率,这个值越大,点云越密集,需要的内存也越多
   float ld_map_icp_teaser_scale = 30;//#这个值越大,对于大场景的点云配准效果越好,越小,例如10,对于小场景小物体的配准越好
   int ld_use_map_icp_teaser = 0; //#是否使用teaser配准 1:使用 0:不使用

   float downSizeFilterMap_loopdetect_ds = 0.05;  //# 回环检测每一个关键帧存储的时候的降采样率
   float ld_surroundingkeyframeAddingAngleThreshold = 0.2; // #关键帧的旋转:rad -> 除以loopdetect_scale
   float ld_surroundingkeyframeAddingDistThreshold = 0.5; //#关键帧的位移: 米 -> 除以loopdetect_scale
   float ld_min_pos_keyframe_color_cloud = 0.25; //# unit: 米(剔除重复值) -> 除以loopdetect_scale
   float ld_min_rot_keyframe_color_cloud = 10; //# unit: °(剔除重复关键帧的角度) -> 除以loopdetect_scale

   int ld_historyKeyframeSearchNum_target = 40; //#多少个关键帧组成target子地图 -> 乘以loopdetect_scale
   int ld_historyKeyframeSearchNum_source = 20; //#多少个关键帧组成source子地图 -> 乘以loopdetect_scale
   int ld_historyKeyframeSearchNum_target_finish = 20; //#多少个关键帧组成target子地图 -> 乘以loopdetect_scale
   int ld_historyKeyframeSearchNum_source_finish = 20; //#多少个关键帧组成source子地图 -> 乘以loopdetect_scale

   float ld_last_loop_detect_time_diff_threshold = 60;//距离上一次回环的时间间隔,超过这个间隔再进行下一次回环检测

   float ld_inlier_value = 0.4;// icp匹配成绩为多少时认为匹配成功
   float ld_inlier_value_finish = 0.4;// icp匹配成绩为多少时认为匹配成功
   float ld_historyKeyframeSearchRadius = 2;//下一次回到相同的地方距离多近算可以进行回环检测 单位米
   float ld_historyKeyframeSearchTimeDiff = 60;//单位秒,当回到相同的地方时判断之前的关键帧与当前的时间间隔超过多少秒才认为是一个回环
    //回环检测可以调节参数 end


    double lidar_begin_global_time = 0.0;
    PointTypePose last_lidar_pose;
    int last_lidar_frame_id = 0.0;
    double last_lidar_begin_global_time = 0.0;
    pcl::VoxelGrid<PointType> downSizeFilterMap_loopdetect;
    ros::ServiceClient client_search;
    ros::ServiceClient client_insert;
    ros::Publisher pub_odom_finish_str_msg;

    std::unordered_map<double,std::shared_ptr<ImgLidarFrame>> cam_lidar_frame_map;
    vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> cornerCloudKeyFrames; // 历史所有关键帧的角点集合（降采样）
    vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> surfCloudKeyFrames;   // 具体的点云就存储在这里，历史所有关键帧的平面点集合（降采样）
    std::vector<EigenPose> keyframe_pose_buffer;//存储keyframe
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_temp;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_temp_down;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudKeyPoses3D;//(new pcl::PointCloud<PointCloudXYZINormal>());         // 历史关键帧位姿（位置）
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;//(new pcl::PointCloud<PointTypePose>()); // 历史关键帧位姿
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D_test;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr copy_cloudKeyPoses3D;//(new pcl::PointCloud<PointCloudXYZINormal>());
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;//(new pcl::PointCloud<PointTypePose>());
    float transformTobeMapped[6];
    pcl::PointCloud<PointTypePose>::Ptr fastlio_unoptimized_cloudKeyPoses6D;//(new pcl::PointCloud<PointTypePose>()); //  存储fastlio 未优化的位姿
    pcl::PointCloud<PointTypePose>::Ptr gnss_cloudKeyPoses6D;//(new pcl::PointCloud<PointTypePose>()); //  gnss 轨迹
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kdtreeHistoryKeyPoses;//用于查找当前最近的关键帧
    pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilterICP;
    pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilterICP_hba;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr feats_undistort_r3live;//( new PointCloudXYZINormal() );

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr feats_undistort_imu_r3live;//(new PointCloudXYZINormal);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr subMapKeyFrames;//(new pcl::PointCloud<pcl::PointXYZINormal>());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr subMapKeyPoses;//(new pcl::PointCloud<pcl::PointXYZINormal>());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr subMapKeyPosesDS;//(new pcl::PointCloud<pcl::PointXYZINormal>());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr subMapKeyFramesDS;//(new pcl::PointCloud<pcl::PointXYZINormal>());
    EigenPose current_to_new_transform;

    std::mutex loop_factor_buffer_mtx;
    std::deque<std::shared_ptr<LoopDetectFactor>> loop_factor_buffer;
    double last_search_img_time;//上次搜索到回环的时间


    //=====test====
    EigenPose ep_test_zero;
    bool is_first_test = true;
    EigenPose last_eigen_pose;
    EigenPose last_eigen_pose_true;
    double total_trajectory_test = 0;
    double total_trajectory_test_true = 0;
    //=====test====
//******************** backend  end ******************************

//******************** backend start **************
public:
    /**
     * 功能: 判断当前帧是否为关键帧
     * 计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
     */
    /**
     * Eigen格式的位姿变换
     */
    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    /**
     * Eigen格式的位姿变换
     */
    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    /**
     * 位姿格式变换
     */
    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                            gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    /**
     * 位姿格式变换
     */
    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                            gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    }

    // gtsam
    gtsam::NonlinearFactorGraph gtSAMgraph;//gtsam因子图，用于添加边的约束
    gtsam::Values initialEstimate;//gtsam初始的估计值，用于存储初始值
    gtsam::Values optimizedEstimate;//gtsam优化后的估计值
    gtsam::ISAM2 *isam;//gtsam的求解器
    gtsam::Values isamCurrentEstimate;//gtsam当前优化后的值
    Eigen::MatrixXd poseCovariance;//优化后的整体协方差矩阵

    map<int, int> loopIndexContainer; // from new to old ，用于判断当前帧是否已经加入了回环，没有才使用当前帧
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;

    bool aLoopIsClosed; //是否已经加入了新的回环
    //deque<std_msgs::Float64MultiArray> loopInfoVec;
    std::mutex mtx_loopdetect;//回环检测锁

    std::mutex mtx_loopdetect_playbag;
    int mtx_loopdetect_playbag_flag = 0;
    ros::Publisher pubLoopDetectSource,pubLoopDetectTarget,pubLoopDetectSourceTransform,m_cloud_pub,m_cloud_calibration_pub;

    kilox::CeresIcpTool ceresIcpTool_handle;//(0.4,0.4);

    std::string rosbag_play_service_name;
    double last_loop_detect_time;
    std::string get_rosbag_play_ser_name(){
        if (rosbag_play_service_name.find("pause_playback") != std::string::npos) {
            return rosbag_play_service_name;
        }

            
        const char* cmd = "rosservice list";
        // 使用popen执行命令并获取输出
        FILE* pipe = popen(cmd, "r");
        if (!pipe) {
            return ""; // 如果popen失败，则返回错误
        }

        // 读取命令的输出
        char buffer[128];
        while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
            // 检查输出中是否包含"pause_playback"
            //std::cout << "=====================++> Found service: " << buffer;
            std::string buffer_name(buffer);
            //std::cout<<"buffer_name:"<<buffer_name<<std::endl;
            if (buffer_name.find("pause_playback") != std::string::npos) {
                rosbag_play_service_name = buffer_name;
                if (!rosbag_play_service_name.empty() && rosbag_play_service_name.back() == '\n') {
                    // 移除最后的换行符
                    rosbag_play_service_name.erase(rosbag_play_service_name.size() - 1);
                }
            }
            
        }
        int status2 = pclose(pipe);
        return rosbag_play_service_name;
    }

    bool check_rosbag_play_is_alive(){
        const char* cmd = "rosservice list";
        // 使用popen执行命令并获取输出
        FILE* pipe = popen(cmd, "r");
        if (!pipe) {
            return false; // 如果popen失败，则返回错误
        }

        
        bool is_node_alive = true;
        if(rosbag_play_service_name.find("player") != std::string::npos){
            std::string get_ros_service_name = get_rosbag_play_ser_name();
            if (rosbag_play_service_name.find("pause_playback") != std::string::npos) {
                std::string command = "rosservice call " +get_ros_service_name+" \"data: false\"";
                std::cout<<"play command:"<<command<<std::endl;
                //system(command.c_str());
                ros::NodeHandle nh;
                ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>(get_ros_service_name);
                //system(command.c_str());
                std_srvs::SetBool srv;
                srv.request.data = false;
                if (client.call(srv))
                {
                    // 服务调用成功
                    ROS_INFO("paly Service call successful: %s", srv.response.message.c_str());
                }
                else
                {
                   is_node_alive = false;
                }
            }
        }

        // 读取命令的输出
        char buffer[128];
        while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
            // 检查输出中是否包含"pause_playback"
            //std::cout << "=====================++> Found service: " << buffer;
            std::string buffer_name(buffer);
            //std::cout<<"buffer_name:"<<buffer_name<<std::endl;
            if (buffer_name.find("pause_playback") != std::string::npos && is_node_alive) {
                return true;
            }
            
        }
        int status2 = pclose(pipe);
        return false;
    }

   
    bool isNodeAlive(const std::string& target_node) {
    // 获取所有活跃节点列表
        std::vector<std::string> active_nodes;
        if (!ros::master::getNodes(active_nodes)) {
            ROS_ERROR("Failed to get node list from master");
            return false;
        }
        
        // 遍历查找目标节点
        return (std::find(active_nodes.begin(), 
                        active_nodes.end(), 
                        target_node) != active_nodes.end());
    }
    bool pause_ros_bag(){
        std::string get_ros_service_name = get_rosbag_play_ser_name();
        if (rosbag_play_service_name.find("pause_playback") == std::string::npos) {
            return true;
        }
        
        std::string command = "rosservice call " +get_ros_service_name+" \"data: true\"";
        std::cout<<"pause command:"<<command<<std::endl;
        ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>(get_ros_service_name);
        //system(command.c_str());
        std_srvs::SetBool srv;
        srv.request.data = true;
        if (client.call(srv))
        {
            // 服务调用成功
            ROS_INFO("pause Service call successful: %s", srv.response.message.c_str());
        }
        else
        {
            // 服务调用失败
            ROS_ERROR("Failed to call pause service");
        }
        return true;
    }

    bool play_ros_bag(){
        std::string get_ros_service_name = get_rosbag_play_ser_name();
        if (rosbag_play_service_name.find("pause_playback") == std::string::npos) {
            std::cout<<"rosbag_play_service_name:"<<rosbag_play_service_name<<std::endl;
            return true;
        }
        std::string command = "rosservice call " +get_ros_service_name+" \"data: false\"";
        std::cout<<"play command:"<<command<<std::endl;
        //system(command.c_str());
        ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>(get_ros_service_name);
        //system(command.c_str());
        std_srvs::SetBool srv;
        srv.request.data = false;
        if (client.call(srv))
        {
            // 服务调用成功
            ROS_INFO("paly Service call successful: %s", srv.response.message.c_str());
        }
        else
        {
            // 服务调用失败
            ROS_ERROR("Failed to call play service");
            if(check_rosbag_play_is_alive()==false){
                //检查是否播包已经彻底结束，如果彻底结束，则将最后一个关键帧做一次回环
                return false;
            }
        }
        return true;
    }
    

    /**
     * 添加激光里程计因子
     */
    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            // 第一帧初始化先验因子，也就是固定住第一帧的位置
            gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) <<1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished()); // rad*rad, meter*meter   // indoor 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12    //  1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8
            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            std::cout<<"====> add odom factor:"<<__LINE__<<std::endl;
            // 变量节点设置初始值，transformTobeMapped这个是当前帧的位姿，将第一帧插入initialEstimate
            std::cout<<"transformTobeMapped "<<transformTobeMapped[0]<<" "<<transformTobeMapped[1]<<" "<<transformTobeMapped[2]<<" "<<transformTobeMapped[3]<<" "<<transformTobeMapped[4]<<" "<<transformTobeMapped[5]<<std::endl;
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }
        else
        {
            //std::cout<<"====> add odom factor else : "<<__LINE__<<std::endl; 
            // 变量节点设置初始值，transformTobeMapped这个是当前帧的位姿，将第一帧插入initialEstimate
            //std::cout<<"transformTobeMapped "<<transformTobeMapped[0]<<" "<<transformTobeMapped[1]<<" "<<transformTobeMapped[2]<<" "<<transformTobeMapped[3]<<" "<<transformTobeMapped[4]<<" "<<transformTobeMapped[5]<<std::endl;
            float scale_noise = 1;
            // 添加激光里程计因子 这里里程计因子的协方差如果太小则优化不动，因为它的权重很高，里程计无法优化
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 5e-1*scale_noise, 5e-1*scale_noise, 5e-1*scale_noise, 3e-1*scale_noise, 3e-1*scale_noise, 3e-1*scale_noise).finished());//3e-1, 3e-1, 3e-1
            //gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back()); /// pre
            //这里加入当前的帧作为关键帧到gtsam
            gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);                   // cur
            // 参数：前一帧id，当前帧id，前一帧与当前帧的位姿变换（作为观测值），噪声协方差
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            // 变量节点设置初始值
            //
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    /**
     * 添加闭环因子
     * 将之前检测到的回环加入到
     */
    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;

        // 闭环队列
        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            // 闭环边对应两帧的索引
            int indexFrom = loopIndexQueue[i].first; //   cur
            int indexTo = loopIndexQueue[i].second;  //    pre
            // 闭环边的位姿变换
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            std::cout<<"add loop detect: "<<__LINE__<<std::endl;
            std::cout<<"indexFrom:"<<indexFrom<<" indexTo:"<<indexTo<<std::endl;
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();

        //是否已经加入了新的回环
        aLoopIsClosed = true;
    }
    bool saveFrame()
    {
        std::cout<<"line:"<<__LINE__<<std::endl;
        float surroundingkeyframeAddingAngleThreshold = 0.2;
        float surroundingkeyframeAddingDistThreshold = 0.5;//source:1 

        if(USE_LOOP_PARAM){
            surroundingkeyframeAddingAngleThreshold = ld_surroundingkeyframeAddingAngleThreshold;
            surroundingkeyframeAddingDistThreshold = ld_surroundingkeyframeAddingDistThreshold;
        }

        std::cout<<"saveFrame loopdetect_enable:"<<loopdetect_enable<<std::endl;
        //exit(0);

        //当前点云关键帧如果不是关键帧,则持久化当前雷达点云利用pcl保存点云,然后当前点云的坐标就是原始的雷达坐标,然后点云的位姿和关键帧绑定,绑定为到关键帧的相对位姿
        if(loopdetect_enable == 0){
            surroundingkeyframeAddingDistThreshold = 0.1;
            surroundingkeyframeAddingAngleThreshold = 0.1;
        }
        std::cout<<"line:"<<__LINE__<<std::endl;
        if (cloudKeyPoses3D->points.empty())
            return true;
        
        
        std::cout<<"line:"<<__LINE__<<std::endl;
        // 前一帧位姿
        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        // 当前帧位姿
        Eigen::Affine3f transFinal = trans2Affine3f(transformTobeMapped);
        // Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
        //                                                     transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        std::cout<<"line:"<<__LINE__<<std::endl;                
        // 位姿变换增量
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw); //  获取上一帧 相对 当前帧的 位姿

        std::cout<<"line:"<<__LINE__<<std::endl;
        // 1米一个关键帧 0.2弧度的旋转(11.4°）)
        // 旋转和平移量都较小，当前帧不设为关键帧
        if (abs(roll) < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
            abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
            return false;

        if(loopdetect_enable == 0){
            return true;
        }
        bool can_pro = true;
        float min_pos_keyframe_color_cloud = 0.25;//0.5米
        float min_rot_keyframe_color_cloud = 10;//15°

        if(USE_LOOP_PARAM){
            min_pos_keyframe_color_cloud = ld_min_pos_keyframe_color_cloud;
            min_rot_keyframe_color_cloud = ld_min_rot_keyframe_color_cloud;
        }

        Eigen::Quaternionf rotationQuaternion(transFinal.rotation());
        Eigen::Quaterniond rotationQuaternionDouble(rotationQuaternion.w(), rotationQuaternion.x(), rotationQuaternion.y(), rotationQuaternion.z());
        Eigen::Vector3d translationVector(transFinal.translation().x(), transFinal.translation().y(), transFinal.translation().z());
        EigenPose ep(rotationQuaternionDouble, translationVector);

        std::cout<<"line:"<<__LINE__<<std::endl;
        if(keyframe_pose_buffer.size()>0){
        for(int t=0;t<keyframe_pose_buffer.size();t++){
                EigenPose pose_item_w = keyframe_pose_buffer[t];
                
                EigenPose last_ep = (pose_item_w.inverse() * ep);

                Eigen::Vector3d l_rpy = last_ep.GetRPY();
                Eigen::Vector3d l_t = last_ep.t_;
                if(l_t.norm() < min_pos_keyframe_color_cloud &&
                    l_rpy.norm() * 57.3 < min_rot_keyframe_color_cloud){
                    can_pro= false;
                    break;
                }
            }
        }

        if(can_pro){
            EigenPose pose_item(rotationQuaternionDouble,translationVector);
            //camera to world
            keyframe_pose_buffer.push_back(pose_item);
        }else{
            return false;
        }

        return true;
    }

    /**
     * 根据优化结果更新里程计轨迹
     * 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹
     */
    void correctPoses()
    {
        //根据最近的一些关键帧重新构建点云
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
            // 清空里程计轨迹
            //globalPath.poses.clear();
            // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿
            int numPoses = isamCurrentEstimate.size();
            //save_loop_pcd_file("/home/kilox/wutong/binary_source_"+std::to_string(cloudKeyPoses3D->points.size())+".pcd");
            //pcl::io::savePCDFileBinary("/home/kilox/wutong/binary_source_trajectory_"+std::to_string(cloudKeyPoses3D->points.size())+".pcd", *cloudKeyPoses3D);
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();

                //维护一个新老关系的转换量，而不用去实际更新voxelmap 计算新老关系的变换

                // 更新里程计轨迹
                //updatePath(cloudKeyPoses6D->points[i]);
            }
            //save_loop_pcd_file("/home/kilox/wutong/binary_final_"+ std::to_string(cloudKeyPoses3D->points.size()) +".pcd");
            //exit(0);
            // 清空局部map， reconstruct  ikdtree submap
            //recontructIKdTree();
            ROS_INFO("ISMA2 Update");
            aLoopIsClosed = false;
        }
    }

    //  eulerAngle 2 Quaterniond
    Eigen::Quaterniond  EulerToQuat(float roll_, float pitch_, float yaw_)
    {
        Eigen::Quaterniond q ;            //   四元数 q 和 -q 是相等的
        Eigen::AngleAxisd roll(double(roll_), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(double(pitch_), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(double(yaw_), Eigen::Vector3d::UnitZ());
        q = yaw * pitch * roll ;
        q.normalize();
        return q ;
    }

    bool saveKeyFramesAndFactor()
    {
        //  计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
        if (saveFrame() == false){
          std::cout<<"line:"<<__LINE__<<std::endl;
            if(cloudKeyPoses6D->points.size() > 0){
                    PointTypePose& last_keyframe = cloudKeyPoses6D->points[cloudKeyPoses6D->points.size()-1];

                    Eigen::Vector3d pos_lidar(last_keyframe.x, last_keyframe.y, last_keyframe.z);
                    Eigen::Quaterniond q_lidar = EulerToQuat(last_keyframe.roll, last_keyframe.pitch, last_keyframe.yaw);
                    EigenPose last_lidar_to_world(q_lidar, pos_lidar);

                    Eigen::Vector3d pos_lidar_curr(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
                    Eigen::Quaterniond q_lidar_curr = EulerToQuat(transformTobeMapped[0], transformTobeMapped[1],transformTobeMapped[2]);
                    EigenPose current_lidar_to_world(q_lidar_curr, pos_lidar_curr);
                    
                    //相机到上一个雷达关键帧的距离
                    EigenPose current_to_last = last_lidar_to_world.inverse() * current_lidar_to_world;
                    Other_No_Keyframe other_No_Keyframe;
                    other_No_Keyframe.q_to_current_keyframe.x() = current_to_last.GetQ().x();
                    other_No_Keyframe.q_to_current_keyframe.y() = current_to_last.GetQ().y();
                    other_No_Keyframe.q_to_current_keyframe.z() = current_to_last.GetQ().z();
                    other_No_Keyframe.q_to_current_keyframe.w() = current_to_last.GetQ().w();

                    other_No_Keyframe.t_to_current_keyframe = current_to_last.t_;
                    other_No_Keyframe.time = lidar_begin_global_time;
                    last_keyframe.other_buffer.push_back(other_No_Keyframe);
                   
            }
            //将当前帧数据存储到原始数据帧
            return false;
        }
            
        
          /**
        PointTypePose thisPose6D2;
        thisPose6D2.x = transformTobeMapped[3] ;
        thisPose6D2.y = transformTobeMapped[4] ;
        thisPose6D2.z = transformTobeMapped[5];
        thisPose6D2.intensity = 0;
        thisPose6D2.roll = transformTobeMapped[0];
        thisPose6D2.pitch = transformTobeMapped[1];
        thisPose6D2.yaw = transformTobeMapped[2];
        cloudKeyPoses6D_test->push_back(thisPose6D2);
        */
        //需要的是最原始的没有优化过的雷达关键帧的状态

        std::cout<<"line:"<<__LINE__<<std::endl;
        last_lidar_pose.x = transformTobeMapped[3];
        last_lidar_pose.y = transformTobeMapped[4];
        last_lidar_pose.z = transformTobeMapped[5];
        last_lidar_pose.intensity = 0;
        last_lidar_pose.roll = transformTobeMapped[0];
        last_lidar_pose.pitch = transformTobeMapped[1];
        last_lidar_pose.yaw = transformTobeMapped[2];
        last_lidar_pose.time = lidar_begin_global_time;
        last_lidar_frame_id = cloudKeyPoses3D->size();

        // 激光里程计因子(from fast-lio),  输入的是frame_relative pose  帧间位姿(body 系下)
        addOdomFactor();
        std::cout<<"line:"<<__LINE__<<std::endl;
        // GPS因子 (UTM -> WGS84)
        //addGPSFactor();
        // 闭环因子 (rs-loop-detect)  基于欧氏距离的检测
        addLoopFactor();
        std::cout<<"line:"<<__LINE__<<std::endl;
        // 执行优化
        std::cout<<"initialEstimate size : "<<initialEstimate.size()<<std::endl;

        
        
        if(initialEstimate.size() ==1 ){

        }

        // 当前帧位姿结果
        gtsam::Pose3 latestEstimate_isam;// = isam->calculateBestEstimate().at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);
        if(isamCurrentEstimate.size() > 0){
            //latestEstimate_isam = isam->calculateBestEstimate().at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);
        }

        isam->update(gtSAMgraph, initialEstimate);

        std::cout<<"line:"<<__LINE__<<std::endl;
        isam->update();
        std::cout<<"=================================> aLoopIsClosed:"<<aLoopIsClosed<<std::endl;
        if (aLoopIsClosed == true) // 有回环因子，多update几次
        {
            std::cout<<"=================================> has loopdetect factor"<<std::endl;

            for(int i=0;i<10;i++){
                isam->update();
                isam->update();
                isam->update();
                isam->update();
                isam->update();
            }

        }
        // update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了,也就是说图已经保存在isam里面了？
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        pcl::PointXYZINormal thisPose3D;
        PointTypePose thisPose6D;
        gtsam::Pose3 latestEstimate;

        // 优化结果
        isamCurrentEstimate = isam->calculateBestEstimate();
        // 当前帧位姿结果
        latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);

        // cloudKeyPoses3D加入当前帧位置
        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        // 索引
        thisPose3D.intensity = cloudKeyPoses3D->size(); //  使用intensity作为该帧点云的index
        cloudKeyPoses3D->push_back(thisPose3D);         //  新关键帧帧放入队列中

        // cloudKeyPoses6D加入当前帧位姿
        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity;
        thisPose6D.roll = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw = latestEstimate.rotation().yaw();
        thisPose6D.time = lidar_begin_global_time;

        

        last_lidar_begin_global_time = lidar_begin_global_time;
        cloudKeyPoses6D->push_back(thisPose6D);

        // 位姿协方差
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

        std::cout<<"------> cloudKeyPoses6D size:"<<cloudKeyPoses6D->points.size()<<std::endl;

        if(aLoopIsClosed == true && cloudKeyPoses6D->points.size()>0){
            //exit(0);
        }
        //exit(0);
        /**
        Eigen::Vector3d pos_new(latestEstimate.translation().x(), latestEstimate.translation().y(), latestEstimate.translation().z());
        Eigen::Quaterniond q_new = EulerToQuat(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(), latestEstimate.rotation().yaw());
        Eigen::Quaternionf q_float(q_new.w(), q_new.x(), q_new.y(), q_new.z());
        Eigen::Vector3f pos_float(pos_new.x(), pos_new.y(), pos_new.z());

        //当前帧位姿
        Eigen::Affine3f current_Pose = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f new_Pose = Eigen::Affine3f::Identity();
        pose.translation() = pos_float;
        pose.linear() = q_float.toRotationMatrix();

        Eigen::Affine3f current_to_newPose = new_Pose.inverse()*current_Pose;
        */

        Eigen::Vector3d pos_new(latestEstimate.translation().x(), latestEstimate.translation().y(), latestEstimate.translation().z());
        Eigen::Quaterniond q_new = EulerToQuat(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(), latestEstimate.rotation().yaw());

        Eigen::Vector3d pos_old(latestEstimate_isam.translation().x(), latestEstimate_isam.translation().y(), latestEstimate_isam.translation().z());
        Eigen::Quaterniond q_old = EulerToQuat(latestEstimate_isam.rotation().roll(), latestEstimate_isam.rotation().pitch(), latestEstimate_isam.rotation().yaw());
        

        //当前帧位姿
        Eigen::Affine3f current_Pose = trans2Affine3f(transformTobeMapped);
        Eigen::Quaternionf rotationQuaternion(current_Pose.rotation());
        Eigen::Quaterniond rotationQuaternionDouble(rotationQuaternion.w(), rotationQuaternion.x(), rotationQuaternion.y(), rotationQuaternion.z());
        Eigen::Vector3d translationVector(current_Pose.translation().x(), current_Pose.translation().y(), current_Pose.translation().z());

        EigenPose new_ep(q_new,pos_new);
        EigenPose current_ep(rotationQuaternionDouble,translationVector);

        EigenPose current_ep_old(q_old,pos_old);
        
        //当前帧的位置的变化量（在世界坐标系的变化量）todo: 这里是累乘的关系?
        EigenPose current_to_new = new_ep * current_ep.inverse() * current_to_new_transform;
        //EigenPose current_to_new = new_ep * current_ep_old.inverse() * current_to_new_transform;

        if (aLoopIsClosed == true) // 有回环因子，多update几次
        {
            current_to_new_transform.R_ = current_to_new.R_;
            current_to_new_transform.t_ = current_to_new.t_;
            std::cout<<"current_to_new_transform r: "<<current_to_new_transform.R_<<std::endl;
            std::cout<<"current_to_new_transform t: "<<current_to_new_transform.t_<<std::endl;
            //exit(0);
        }
        /**
        
        EigenPose current_to_new =current_to_new_transform* new_ep * current_ep.inverse();
        
        if (aLoopIsClosed == true) // 有回环因子，多update几次
        {
            // 更新transform
            current_to_new_transform.R_ = current_to_new.R_;
            current_to_new_transform.t_ = current_to_new.t_;
            std::cout<<"current_to_new R:"<<current_to_new.R_<<" current_to_new t:"<<current_to_new.t_<<std::endl;
        }
        */
       
        //
        /**
        // ESKF状态和方差  更新
        state_ikfom state_updated = kf.get_x(); //  获取cur_pose (还没修正)
        Eigen::Vector3d pos(latestEstimate.translation().x(), latestEstimate.translation().y(), latestEstimate.translation().z());
        Eigen::Quaterniond q = EulerToQuat(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(), latestEstimate.rotation().yaw());

        //  更新状态量
        state_updated.pos = pos;
        state_updated.rot =  q;
        state_point = state_updated; // 对state_point进行更新，state_point可视化用到
        
        kf.change_x(state_updated);  //  对cur_pose 进行isam2优化后的修正

        // TODO:  P的修正有待考察，按照yanliangwang的做法，修改了p，会跑飞
        // esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P_updated = kf.get_P(); // 获取当前的状态估计的协方差矩阵
        // P_updated.setIdentity();
        // P_updated(6, 6) = P_updated(7, 7) = P_updated(8, 8) = 0.00001;
        // P_updated(9, 9) = P_updated(10, 10) = P_updated(11, 11) = 0.00001;
        // P_updated(15, 15) = P_updated(16, 16) = P_updated(17, 17) = 0.0001;
        // P_updated(18, 18) = P_updated(19, 19) = P_updated(20, 20) = 0.001;
        // P_updated(21, 21) = P_updated(22, 22) = 0.00001;
        // kf.change_P(P_updated);

        // 当前帧激光角点、平面点，降采样集合
        // pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        // pcl::copyPointCloud(*feats_undistort,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*feats_undistort, *thisSurfKeyFrame); // 存储关键帧,没有降采样的点云

        // 保存特征点降采样集合
        // cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        updatePath(thisPose6D); //  可视化update后的path


        
        
        */
       //
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr thisSurfKeyFrame(new pcl::PointCloud<pcl::PointXYZINormal>());
        //pcl::copyPointCloud(*feats_undistort_imu, *thisSurfKeyFrame); // 存储关键帧,没有降采样的点云

        //pcl::PointCloud<pcl::PointXYZINormal>::Ptr thisSurfKeyFrame(new pcl::PointCloud<pcl::PointXYZINormal>());
        downSizeFilterMap_loopdetect.setInputCloud( feats_undistort_imu_r3live );
        downSizeFilterMap_loopdetect.filter( *thisSurfKeyFrame );
        //todo: 这里可能导致内存溢出 所以最好存储降采样之后的点
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);
        
        /**
        PointTypePose thisPose6D2;
        thisPose6D2.x = transformTobeMapped[3] ;
        thisPose6D2.y = transformTobeMapped[4] ;
        thisPose6D2.z = transformTobeMapped[5];
        thisPose6D2.intensity = 0;
        thisPose6D2.roll = transformTobeMapped[0];
        thisPose6D2.pitch = transformTobeMapped[1];
        thisPose6D2.yaw = transformTobeMapped[2];
        cloudKeyPoses6D_test->push_back(thisPose6D2);
        */
       return true;
    }

    void transformLidar_to_IMU(const pcl::PointCloud<PointType>::Ptr &input_cloud, pcl::PointCloud<PointType>::Ptr &trans_cloud)
    {
        trans_cloud->clear();

        for (size_t i = 0; i < input_cloud->size(); i++) {
            pcl::PointXYZINormal p_c = input_cloud->points[i];
            Eigen::Vector3d p_lidar(p_c.x, p_c.y, p_c.z);
            // HACK we need to specify p_body as a V3D type!!!
            Eigen::Vector3d p_body =  extR * p_lidar + extT;
            //std::cout<<"state lidar R:"<<std::endl<<state_point.offset_R_L_I.toRotationMatrix()<<" px:"<<p_c.x<<" py:"<<p_c.y<<" z:"<<p_c.z<<std::endl;

            PointType pi;
            pi.x = p_body(0);
            pi.y = p_body(1);
            pi.z = p_body(2);
            pi.intensity = p_c.intensity;
            trans_cloud->points.push_back(pi);
        }
    }
    //回环检测三大要素
    // 1.设置最小时间差，太近没必要
    // 2.控制回环的频率，避免频繁检测，每检测一次，就做一次等待
    // 3.根据当前最小距离重新计算等待时间
    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        //return false;
        float historyKeyframeSearchTimeDiff = 60;
        float historyKeyframeSearchRadius = 2;//source 5 2

        if(USE_LOOP_PARAM){
            historyKeyframeSearchRadius = ld_historyKeyframeSearchRadius;
            historyKeyframeSearchTimeDiff = ld_historyKeyframeSearchTimeDiff;
        }

        // 当前关键帧帧
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1; //  当前关键帧索引
        int loopKeyPre = -1;

        // 当前帧已经添加过闭环对应关系，不再继续添加
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;
        // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
        std::vector<int> pointSearchIndLoop;                        //  候选关键帧索引
        std::vector<float> pointSearchSqDisLoop;                    //  候选关键帧距离
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D); //  历史帧构建kdtree
        //根据半径搜索，找到合适的关键帧
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        //std::cout<<"===========================================> hello here:"<<pointSearchIndLoop.size()<<std::endl;
        
        // 在候选关键帧集合中，找到与当前帧时间相隔较远的帧，设为候选匹配帧
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            //std::cout<<"time:"<<abs(copy_cloudKeyPoses6D->points[id].time - copy_cloudKeyPoses6D->points[copy_cloudKeyPoses6D->points.size()-1].time)<<" copy_cloudKeyPoses6D->points[id].time:"<<copy_cloudKeyPoses6D->points[id].time<<" copy_cloudKeyPoses6D->points[copy_cloudKeyPoses6D->points.size()-1].time:"<<copy_cloudKeyPoses6D->points[copy_cloudKeyPoses6D->points.size()-1].time<<std::endl;
            if (abs(copy_cloudKeyPoses6D->points[id].time - copy_cloudKeyPoses6D->points[copy_cloudKeyPoses6D->points.size()-1].time) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }
        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;
        *latestID = loopKeyCur;
        *closestID = loopKeyPre;
        ROS_INFO("Find loop clousre frame ");
        return true;
    }

    Eigen::Matrix4f readExtrinsicMatrix(const std::string& file_path) {
        Eigen::Matrix4f extrinsic = Eigen::Matrix4f::Identity(); // 初始化为单位矩阵
        std::ifstream file(file_path);
        std::string line;
        int row = 0;

        while (std::getline(file, line) && row < 3) { // 读取前三行
            std::istringstream iss(line);
            float m11, m12, m13, m14;
            if (!(iss >> m11 >> m12 >> m13 >> m14)) {
                throw std::runtime_error("Error parsing extrinsic file");
            }
            extrinsic.row(row) << m11, m12, m13, m14;
            ++row;
        }

        // 确保第四行为 [0, 0, 0, 1]
        extrinsic.row(3) << 0.0f, 0.0f, 0.0f, 1.0f;

        return extrinsic;
    }
    /**
     * 对点云cloudIn进行变换transformIn，返回结果点云， 修改liosam, 考虑到外参的表示
     */
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformPointCloud_loopdetect(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudIn, PointTypePose *transformIn)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZINormal>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);
        
    // 注意：lio_sam 中的姿态用的euler表示，而fastlio存的姿态角是旋转矢量。而 pcl::getTransformation是将euler_angle 转换到rotation_matrix 不合适，注释
    // Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
       // Eigen::Isometry3d T_b_lidar(state_point.offset_R_L_I  );       //  获取  body2lidar  外参
       // T_b_lidar.pretranslate(state_point.offset_T_L_I);        

        Eigen::Affine3f T_w_b_ = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        Eigen::Isometry3d T_w_b ;          //   world2body  
        T_w_b.matrix() = T_w_b_.matrix().cast<double>();

        Eigen::Isometry3d  T_w_lidar  =  T_w_b;// * T_b_lidar  ;           //  T_w_lidar  转换矩阵

        Eigen::Isometry3d transCur = T_w_lidar;        
        int numberOfCores = 4;
    #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    // 将更新的pose赋值到 transformTobeMapped
    void updateCurPose(StatesGroup cur_state)
    {
        //  欧拉角是没有群的性质，所以从SO3还是一般的rotation matrix 转换过来的结果一样
        Eigen::Vector3d eulerAngle = cur_state.rot_end.matrix().eulerAngles(2,1,0);        //  yaw pitch roll  单位：弧度

        EigenPose current_state(cur_state.rot_end,cur_state.pos_end);
        EigenPose new_pose = current_to_new_transform*current_state;
        // transformTobeMapped[0] = eulerAngle(0);                //  roll     使用 SO3ToEuler 方法时，顺序是 rpy
        // transformTobeMapped[1] = eulerAngle(1);                //  pitch
        // transformTobeMapped[2] = eulerAngle(2);                //  yaw
         //todo:在这里加上旋转与平移增量
        Eigen::Vector3d euler_cur_lidar = RotMtoEuler(new_pose.R_);
        // V3D eulerAngle  =  SO3ToEuler(cur_state.rot)/57.3 ;     //   fastlio 自带  roll pitch yaw  单位: 度，旋转顺序 zyx

        transformTobeMapped[0] = euler_cur_lidar(0);                //  roll 
        transformTobeMapped[1] = euler_cur_lidar(1);                //  pitch
        transformTobeMapped[2] = euler_cur_lidar(2);                //  yaw
        transformTobeMapped[3] = new_pose.t_[0];          //  x
        transformTobeMapped[4] = new_pose.t_[1];          //   y
        transformTobeMapped[5] = new_pose.t_[2];          // z

    }
    /**
     * 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
     */
    void loopFindNearKeyframes(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &nearKeyframes, const int &key, const int &searchNum)
    {
        
        cloud_temp->points.clear();
        // 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        auto surfcloud_keyframes_size = surfCloudKeyFrames.size() ;
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize)
                continue;

            if (keyNear < 0 || keyNear >= surfcloud_keyframes_size)
                continue;

            // *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            // 注意：cloudKeyPoses6D 存储的是 T_w_b , 而点云是lidar系下的，构建icp的submap时，需要通过外参数T_b_lidar 转换 , 参考pointBodyToWorld 的转换
            *nearKeyframes += *transformPointCloud_loopdetect(surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]); //  fast-lio 没有进行特征提取，默认点云就是surf
        }

        if (nearKeyframes->empty())
            return;

        // 降采样
        
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void write_camera_and_lidar_trajectory(std::string filename){
        std::fstream outfile;
        outfile.open(filename.c_str(), std::istream::out);

        if (!outfile) {
            std::cout << "Error opening the file: " << filename<<std::endl;
            return;
        }

        if (cloudKeyPoses3D->points.empty())
        {
            std::cout<<"没有任何关键帧，建图结束"<<std::endl;
            return;
        }

        boost::filesystem::path meta_folder = "/home/kilox/wutong/data";
        if (boost::filesystem::exists(meta_folder)) {
            boost::filesystem::remove_all(meta_folder);
        }
        boost::filesystem::create_directory(meta_folder);
        if (true)
        {

            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();
                
                Eigen::Quaterniond q_lidar = EulerToQuat(cloudKeyPoses6D->points[i].roll, cloudKeyPoses6D->points[i].pitch, cloudKeyPoses6D->points[i].yaw);
                outfile<<"1"<<" " <<std::setprecision (16)<< cloudKeyPoses6D->points[i].time << " " << cloudKeyPoses3D->points[i].x << " "<< cloudKeyPoses3D->points[i].y << " "<<cloudKeyPoses3D->points[i].z << " "
                << q_lidar.x() << " " << q_lidar.y() << " "
                << q_lidar.z() << " " << q_lidar.w() << '\n';
                // updatePath(cloudKeyPoses6D->points[i]);
                std::stringstream path_pcd;
                path_pcd <<"/home/kilox/wutong/data/"<< std::setprecision (16)  << cloudKeyPoses6D->points[i].time<<".pcd";
               
                if(surfCloudKeyFrames[i]->points.size()>0){
                    pcl::io::savePCDFileBinary(path_pcd.str(), *transformPointCloud_loopdetect(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]));
                }
                // surfCloudKeyFrames[i]
                
            }
            ROS_INFO("====== finish save trajectory file =======");
        }
    }

    //输出所有的激光雷达轨迹而不仅仅是关键帧的轨迹
    void write_camera_and_lidar_trajectory_total(std::string filename){

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr world_merge_all_color(new  pcl::PointCloud<pcl::PointXYZINormal>);

        
        std::fstream outfile;
        outfile.open(filename.c_str(), std::istream::out);

        if (!outfile) {
            std::cout << "Error opening the file: " << filename<<std::endl;
            return;
        }

        if (cloudKeyPoses3D->points.empty())
        {
            std::cout<<"没有任何关键帧，建图结束"<<std::endl;
            return;
        }

        if (true)
        {

            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();
                
                // 直接存储颜色点云???????
                Eigen::Quaterniond q_lidar = EulerToQuat(cloudKeyPoses6D->points[i].roll, cloudKeyPoses6D->points[i].pitch, cloudKeyPoses6D->points[i].yaw);
                outfile<<"1"<<" " <<std::setprecision (16)<< cloudKeyPoses6D->points[i].time << " " << cloudKeyPoses3D->points[i].x << " "<< cloudKeyPoses3D->points[i].y << " "<<cloudKeyPoses3D->points[i].z << " "
                << q_lidar.x() << " " << q_lidar.y() << " "
                << q_lidar.z() << " " << q_lidar.w() << '\n';
                
                Eigen::Vector3d t_lidar(cloudKeyPoses3D->points[i].x, cloudKeyPoses3D->points[i].y, cloudKeyPoses3D->points[i].z);

                EigenPose lidar_to_world(q_lidar, t_lidar);
                for(int j=0;j<cloudKeyPoses6D->points[i].other_buffer.size();j++){
                       Other_No_Keyframe other_key_frame = cloudKeyPoses6D->points[i].other_buffer[j];
                       EigenPose other_key_frame_to_current(other_key_frame.q_to_current_keyframe,other_key_frame.t_to_current_keyframe);
                       EigenPose other_to_world = lidar_to_world*other_key_frame_to_current;
                       outfile<<"0"<<" " <<std::setprecision (16)<< other_key_frame.time << " " << other_to_world.t_[0] << " "<< other_to_world.t_[1] << " "<< other_to_world.t_[2] << " "
                        << other_to_world.GetQ().x() << " " << other_to_world.GetQ().y() << " "
                        << other_to_world.GetQ().z() << " " << other_to_world.GetQ().w() << '\n';

                        pcl::PointXYZINormal pt;
                        pt.x = other_to_world.t_[0];
                        pt.y = other_to_world.t_[1];
                        pt.z = other_to_world.t_[2];
                        world_merge_all_color->points.push_back(pt);
                }
            }
            ROS_INFO("====== finish save trajectory file =======");
        }

        pcl::io::savePCDFileBinary("/home/kilox/wutong/loop_detect/full_trajecotry_all_total.pcd", *world_merge_all_color);
        outfile.close();
    }
   
    void save_loop_pcd_file(std::string save_path){
        subMapKeyFrames->points.clear();
        subMapKeyPoses->points.clear();
        subMapKeyPosesDS->points.clear();
        subMapKeyFramesDS->points.clear();

        for (int i = 0; i < (int)surfCloudKeyFrames.size(); ++i)
        {

            //没有降采样的原始去畸变点云
            // *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            *subMapKeyFrames += *transformPointCloud_loopdetect(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]); //  fast_lio only use  surfCloud
            //*subMapKeyFrames += *surfCloudKeyFrames[i]; //  fast_lio only use  surfCloud
            //if(i != 0 && i!=surfCloudKeyFrames.size()-1 && i<surfCloudKeyFrames.size()-300 && i > 600){
            //    continue;
            //}
           // pcl::io::savePCDFileBinary("/home/kilox/wutong/z_loop_detect_"+std::to_string(i)+".pcd", *transformPointCloud_loopdetect(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]));
        }


       
        std::cout<<"surfCloudKeyFrames size:"<<surfCloudKeyFrames.size()<<" cloudKeyPoses6D_test size:"<<cloudKeyPoses6D->points.size()<<std::endl;
        std::cout<<"subMapKeyFrames size:"<<subMapKeyFrames->points.size()<<std::endl;
        std::cout<<"save_path:"<<save_path<<std::endl;
        pcl::io::savePCDFileBinary(save_path, *subMapKeyFrames);
        //pcl::io::savePCDFileBinary(save_path, *transformedCloud);
    }

   
    /**
    //========kiss-matcher start ========
    std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<pcl::PointXYZINormal>& cloud) {
    std::vector<Eigen::Vector3f> vec;
    vec.reserve(cloud.size());
    for (const auto& pt : cloud.points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        vec.emplace_back(pt.x, pt.y, pt.z);
    }
    return vec;
    }
    //========kiss-matcher end ========
   */


    /**
     * 功能:
     * 回环查找:方法：根据距离进行icp
    */
    bool performLoopClosure(){
        //回环scale数

        //指定要多少个关键帧组成子地图
        int historyKeyframeSearchNum = 40;//这个得和关键帧的选取距离强行关联 目前效果比较好的是20个关键帧 每隔1米一个关键帧
        int historyKeyframeSearchNum_source = 20;
        //如果没有关键帧，则返回

        if(USE_LOOP_PARAM){
            historyKeyframeSearchNum = ld_historyKeyframeSearchNum_target;
            historyKeyframeSearchNum_source = ld_historyKeyframeSearchNum_source;
        }
        while(mtx_loopdetect_playbag_flag==1){
            usleep(1000*50);
        }

        if (cloudKeyPoses3D->points.empty() == true)
        {
            mtx_loopdetect_playbag.lock();
            mtx_loopdetect_playbag_flag = 1;
            mtx_loopdetect_playbag.unlock();
            return false;
        }

        if(loopdetect_enable == 0){

            mtx_loopdetect_playbag.lock();
            mtx_loopdetect_playbag_flag = 1;
            mtx_loopdetect_playbag.unlock();
            return false;
        }
        //return false;

        mtx_loopdetect.lock();
        //拷贝最新的轨迹点云以及位姿 这里可以进行主动内存释放
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx_loopdetect.unlock();

         // 当前关键帧索引，候选闭环匹配帧索引
        int loopKeyCur;
        int loopKeyPre;

        double last_loop_detect_time_diff = copy_cloudKeyPoses6D->points[copy_cloudKeyPoses6D->points.size()-1].time - last_loop_detect_time;
        
        // last_loop_detect_time_diff 两次回环时间间隔: 30
        float last_loop_detect_time_diff_threshold = 60;//30 300用于调试一次回环的结果,避免第二次回环影响第一次的结果

        if(USE_LOOP_PARAM){
            last_loop_detect_time_diff_threshold = ld_last_loop_detect_time_diff_threshold;
        }
        
        // historyKeyframeSearchNum 之前的多少个关键帧合并为一个关键帧计算点云: 40
        // 当前的多少个关键帧合并为一个关键帧计算点云 20
        // 
        //std::cout<<"===================================================> cloudKeyPoses3D->points size:"<<cloudKeyPoses3D->points.size()<<" time diff:"<<last_loop_detect_time_diff<<std::endl;
        if(last_loop_detect_time != 0 && last_loop_detect_time_diff < last_loop_detect_time_diff_threshold){
            mtx_loopdetect_playbag.lock();
            mtx_loopdetect_playbag_flag = 1;
            mtx_loopdetect_playbag.unlock();
            return false;
        }

        
        // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
        if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false )
        {
            mtx_loopdetect_playbag.lock();
            mtx_loopdetect_playbag_flag = 1;
            mtx_loopdetect_playbag.unlock();
            return false;
        }

        // 提取当前的关键帧的点云
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cureKeyframeCloud(new pcl::PointCloud<pcl::PointXYZINormal>()); //  cue keyframe
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr prevKeyframeCloud(new pcl::PointCloud<pcl::PointXYZINormal>()); //   history keyframe submap
        {
            // 提取当前关键帧特征点集合，降采样
            //loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 20); //  将cur keyframe 转换到world系下
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, historyKeyframeSearchNum_source); //  将cur keyframe 转换到world系下
            // 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum); //  选取historyKeyframeSearchNum个keyframe拼成submap
            // 如果特征点较少，返回
            // if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
            //     return;

            /*
            // 发布闭环匹配关键帧局部map
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
            **/
        }

        //暂停播包
        //pause_ros_bag();
        
        // @todo: 强制进行最后一帧的回环       
        // 这里的icp使用ceres的icp
        // ICP Settings
        pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
        icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // 进行scan to map的icp匹配
        // scan-to-map，调用icp匹配
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZINormal>());
        
        icp.align(*unused_result);
        
        //sleep(3);
        //float historyKeyframeFitnessScore = 0.3;
        float historyKeyframeFitnessScore = 0.3;
        // 未收敛，或者匹配不够好
        if (false && icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        {
          
            mtx_loopdetect_playbag.lock();
            mtx_loopdetect_playbag_flag = 1;
            mtx_loopdetect_playbag.unlock();
            return false;
        }
        
        std::cout << "======================================================================================> [[[[[[[[[ icp  success ,icp score: "<<icp.getFitnessScore() << std::endl;
        std::cout<<"line:"<<__LINE__<<std::endl;
        double inlier_value = ceresIcpTool_handle.find_tranfrom_of_two_mappings(*prevKeyframeCloud, *cureKeyframeCloud );

        

         /**
        //========kiss-matcher start ========
        kiss_matcher::KISSMatcherConfig config = kiss_matcher::KISSMatcherConfig(0.1);
        
        kiss_matcher::KISSMatcher matcher(config);
       
        const auto& src_vec = convertCloudToVec(*cureKeyframeCloud);
        const auto& tgt_vec = convertCloudToVec(*prevKeyframeCloud);
        const auto solution = matcher.estimate(src_vec, tgt_vec);

            
         Eigen::Matrix4f solution_eigen      = Eigen::Matrix4f::Identity();
        solution_eigen.block<3, 3>(0, 0)    = solution.rotation.cast<float>();
        solution_eigen.topRightCorner(3, 1) = solution.translation.cast<float>();

        size_t num_final_inliers = matcher.getNumFinalInliers();
        size_t thres_num_inliers = 5;
        if (num_final_inliers < thres_num_inliers) {
            std::cout << "\033[1;33m=> Registration might have failed :(\033[0m\n";
        } else {
            std::cout << "\033[1;32m=> Registration likely succeeded XD\033[0m\n";
        }

        std::cout << solution_eigen << std::endl;
        std::cout << "=====================================" << std::endl;
        exit(0);
        //========kiss-matcher end ========
        */
        if(inlier_value!=inlier_value){
            std::cout<<"inlier_value:"<<inlier_value<<std::endl;
            //开始播包
            //play_ros_bag();
            mtx_loopdetect_playbag.lock();
            mtx_loopdetect_playbag_flag = 1;
            mtx_loopdetect_playbag.unlock();
            return false;
        }
        std::cout<<"line:"<<__LINE__<<std::endl;
        auto ICP_q = ceresIcpTool_handle.m_q_w_curr;
        std::cout<<"line:"<<__LINE__<<std::endl;
        auto ICP_t = ceresIcpTool_handle.m_t_w_curr;

        if(USE_KISS_MATCH == 1){
            std::cout<<"=======================> begin run kiss_matcher_pointcloud"<<std::endl;
            pcl::io::savePCDFileBinary("/home/kilox/wutong/source_kiss_match.pcd", *cureKeyframeCloud);
            pcl::io::savePCDFileBinary("/home/kilox/wutong/target_kiss_match.pcd", *prevKeyframeCloud);
            boost::filesystem::path path("/home/kilox/wutong/ext_param2.txt");
            if (boost::filesystem::exists("/home/kilox/wutong/ext_param2.txt")) {
                try {
                    boost::filesystem::remove("/home/kilox/wutong/ext_param2.txt");
                    std::cout << "File deleted successfully." << std::endl;
                } catch (const boost::filesystem::filesystem_error& e) {
                    std::cerr << "Error deleting file: " << e.what() << std::endl;
                }
            } else {
                std::cout << "File does not exist." << std::endl;
            }
            system("/home/kilox/KISS-Matcher/cpp/examples/build/run_kiss_matcher /home/kilox/wutong/source_kiss_match.pcd /home/kilox/wutong/target_kiss_match.pcd 0.1 /home/kilox/wutong/ext_param2.txt");
            
            if (boost::filesystem::exists("/home/kilox/wutong/ext_param2.txt")){
                Eigen::Matrix4f ext_param = readExtrinsicMatrix("/home/kilox/wutong/ext_param2.txt");
                // 提取平移部分（前三行第四列，float转double）
                Eigen::Vector3d translation = ext_param.block<3, 1>(0, 3).cast<double>();
                // 提取旋转矩阵（左上角3x3部分，float转double）
                Eigen::Matrix3d rotation_matrix = ext_param.block<3, 3>(0, 0).cast<double>();

                // 将旋转矩阵转换为四元数
                Eigen::Quaterniond quaternion = Eigen::Quaterniond(rotation_matrix);
                // 可选：对四元数进行归一化（消除数值误差）
                quaternion.normalize();

                ICP_q = quaternion;
                ICP_t = translation;
                std::cout<<"ext_param:"<<ext_param<<std::endl;
            }
            std::cout<<"=======================> end run kiss_matcher_pointcloud"<<std::endl;
            //Eigen::Matrix4f res_eigen_T = kiss_matcher_pointcloud(cureKeyframeCloud,prevKeyframeCloud);
        }

        if(USE_TEASER_MATCH == 1 || ld_use_map_icp_teaser == 1){
            //Eigen::Matrix4d ext_param = map_icp_teaser(cureKeyframeCloud, prevKeyframeCloud, 30);
            Eigen::Matrix4d ext_param = map_icp_teaser(cureKeyframeCloud, prevKeyframeCloud, ld_map_icp_teaser_scale);

            // 提取平移部分（前三行第四列，float转double）
            Eigen::Vector3d translation = ext_param.block<3, 1>(0, 3).cast<double>();
            // 提取旋转矩阵（左上角3x3部分，float转double）
            Eigen::Matrix3d rotation_matrix = ext_param.block<3, 3>(0, 0).cast<double>();

            // 将旋转矩阵转换为四元数
            Eigen::Quaterniond quaternion = Eigen::Quaterniond(rotation_matrix);
            // 可选：对四元数进行归一化（消除数值误差）
            quaternion.normalize();

            
            std::cout<<"ext_param:"<<ext_param<<std::endl;

            std::cout<<"rotation:"<<std::endl<<ICP_q.toRotationMatrix()<<std::endl;
            std::cout<<"translation:"<<std::endl<<ICP_t<<std::endl;

            ICP_q = quaternion;
            ICP_t = translation;
            //exit(0);
        }

        float inlier_value_threshold = 0.4;
        if(USE_LOOP_PARAM){
            inlier_value_threshold = ld_inlier_value;
        }
        
        //if(inlier_value > 0.4){
        if(inlier_value > inlier_value_threshold){
            std::cout<<"ceres match fail, inlier_value:"<<inlier_value<<" inlier_value_threshold:"<<inlier_value_threshold<<std::endl;
            //开始播包
            //play_ros_bag();
            mtx_loopdetect_playbag.lock();
            mtx_loopdetect_playbag_flag = 1;
            mtx_loopdetect_playbag.unlock();

            //last_loop_detect_time = copy_cloudKeyPoses6D->points[copy_cloudKeyPoses6D->points.size()-1].time;
            //放在这里可以有效减少两次检测回环的间隔
            return false;
        }else{
            std::cout<<"ceres match success, inlier_value:"<<inlier_value<<std::endl;
            std::cout<<"ICP_q:"<<std::endl<<ICP_q.toRotationMatrix()<<std::endl;
            std::cout<<"ICP_t:"<<std::endl<<ICP_t<<std::endl;
        }
        last_loop_detect_time = copy_cloudKeyPoses6D->points[copy_cloudKeyPoses6D->points.size()-1].time;
        std::cout<<"line:"<<__LINE__<<std::endl;

        EigenPose ep(ICP_q,ICP_t);
        ICP_q = ep.inverse().GetQ();
        ICP_t = ep.inverse().t_;

        std::cout<<"ICP_q:"<<ICP_q.toRotationMatrix()<<std::endl;
        std::cout<<"ICP_t:"<<ICP_t<<std::endl;
        //std::cout<<"安全退出程序"<<std::endl;
        //exit(0);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.prerotate(ICP_q.cast<float>());
        transform.pretranslate(ICP_t.cast<float>());
        pcl::PointCloud<pcl::PointXYZINormal> transformed_cloud;
        pcl::transformPointCloud(*cureKeyframeCloud, transformed_cloud, transform);

        if(transformed_cloud.points.size()>0){
            pcl::io::savePCDFileBinary("/home/kilox/wutong/transformed_cloud_"+std::to_string(cureKeyframeCloud->points.size())+".pcd", transformed_cloud);
        }else{
            std::cout<<"transformed_cloud.points:"<<transformed_cloud.points.size()<<std::endl;
        }
        
        std::cout<<"inlier_value:"<<inlier_value<<std::endl;
        //exit(0);
            // 闭环优化得到的当前关键帧与闭环关键帧之间的位姿变换
        sensor_msgs::PointCloud2 source_loop_detect;
        pcl::toROSMsg( *cureKeyframeCloud, source_loop_detect );
        source_loop_detect.header.stamp = ros::Time::now(); 
        source_loop_detect.header.frame_id = "camera_init"; // world; camera_init
        pubLoopDetectSource.publish( source_loop_detect );

        std::cout<<"cureKeyframeCloud size:"<<cureKeyframeCloud->points.size()<<std::endl;
        sensor_msgs::PointCloud2 source_transform;
        pcl::toROSMsg( transformed_cloud, source_transform );
        source_transform.header.stamp = ros::Time::now(); 
        source_transform.header.frame_id = "camera_init"; // world; camera_init
        pubLoopDetectSourceTransform.publish( source_transform );

        std::cout<<"transformed_cloud size:"<<transformed_cloud.points.size()<<std::endl;
        transformed_cloud.clear();
        sensor_msgs::PointCloud2 target_loop_detect;
        pcl::toROSMsg( *prevKeyframeCloud, target_loop_detect );
        target_loop_detect.header.stamp = ros::Time::now(); 
        target_loop_detect.header.frame_id = "camera_init"; // world; camera_init
        pubLoopDetectTarget.publish( target_loop_detect );

        if(cureKeyframeCloud->points.size() > 0){
            pcl::io::savePCDFileBinary("/home/kilox/wutong/cureKeyframeCloud_"+std::to_string(cureKeyframeCloud->points.size())+".pcd", *cureKeyframeCloud);
        }

        if(prevKeyframeCloud->points.size()){
            pcl::io::savePCDFileBinary("/home/kilox/wutong/prevKeyframeCloud_"+std::to_string(cureKeyframeCloud->points.size())+".pcd", *prevKeyframeCloud);
        }
        
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        correctionLidarFrame = transform;

        //current_to_new_transform = transform*current_to_new_transform;
        /**
        EigenPose ep_old(current_to_new_transform.GetQ(),current_to_new_transform.t_);
        EigenPose ep_new(ICP_q,ICP_t);
        EigenPose new_pos_c = (ep_new*ep_old);
        current_to_new_transform.R_ = new_pos_c.R_;
        current_to_new_transform.t_ = new_pos_c.t_;
        */
        // 闭环优化前当前帧位姿
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // 闭环优化后当前帧位姿 
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw); //  获取上一帧 相对 当前帧的 位姿
        gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
        
        // 闭环匹配帧的位姿
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore() ; //  loop_clousre  noise from icp 不确定性
        noiseScore = inlier_value;
        noiseScore = 1e-4;
        //noiseScore = inlier_value;
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
        std::cout<<"line: " <<__LINE__ << "loopNoiseQueue   =   " << noiseScore << std::endl;

        // 只是插空查找了一些回环，如果有回环的话，就更新一下当前的位置，等待在下一次计算时根据回环信息进行查找
        // 添加闭环因子需要的数据
        mtx_loopdetect.lock();
        // 存储回环的索引
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        // 这个是新的计算的两帧之间的正确的相对位置关系
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        // 存储回环检测icp之后的位置不确定度
        loopNoiseQueue.push_back(constraintNoise);
        mtx_loopdetect.unlock();

        loopIndexContainer[loopKeyCur] = loopKeyPre; // 存储和当前帧产生回环的关键帧的索引   使用hash map 存储回环对

        //开始播包
        //play_ros_bag();
        mtx_loopdetect_playbag.lock();
        mtx_loopdetect_playbag_flag = 1;
        mtx_loopdetect_playbag.unlock();
        return true;
    }

    bool performLoopClosureIMG(){

        if(loopdetect_enable == 0){
            return false;
        }

        loop_factor_buffer_mtx.lock();
        if (loop_factor_buffer.size() <= 0)
        {
            loop_factor_buffer_mtx.unlock();
            
            return false;
        }

        

        
        std::shared_ptr<LoopDetectFactor> ptr_loop_detect = loop_factor_buffer.front();
        loop_factor_buffer.pop_front();
        loop_factor_buffer_mtx.unlock();


        //指定要多少个关键帧组成子地图
        int historyKeyframeSearchNum = 20;
        //如果没有关键帧，则返回

         // 当前关键帧索引，候选闭环匹配帧索引
        int loopKeyCur;
        int loopKeyPre;

        loopKeyCur = ptr_loop_detect->curr_lidar_loop_id;
        loopKeyPre = ptr_loop_detect->last_lidar_loop_id;

        // 提取当前的关键帧的点云
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cureKeyframeCloud(new pcl::PointCloud<pcl::PointXYZINormal>()); //  cue keyframe
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr prevKeyframeCloud(new pcl::PointCloud<pcl::PointXYZINormal>()); //   history keyframe submap
        {
            // 提取当前关键帧特征点集合，降采样
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 20); //  将cur keyframe 转换到world系下
            // 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum); //  选取historyKeyframeSearchNum个keyframe拼成submap
            // 如果特征点较少，返回
            // if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
            //     return;

            /*
            // 发布闭环匹配关键帧局部map
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
            **/
        }

        // @todo: 强制进行最后一帧的回环       
        // 这里的icp使用ceres的icp
        // ICP Settings
        pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
        icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // 进行scan to map的icp匹配
        // scan-to-map，调用icp匹配
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZINormal>());
        
        icp.align(*unused_result);
        
        //sleep(3);
        //float historyKeyframeFitnessScore = 0.3;
        float historyKeyframeFitnessScore = 0.4;
        // 未收敛，或者匹配不够好
        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        {
            std::cout<<"======================================================================================> [[[[[[[[[ icp fail,icp score:"<< icp.getFitnessScore()<<std::endl;
            return false;
        }
        last_loop_detect_time = copy_cloudKeyPoses6D->points[copy_cloudKeyPoses6D->points.size()-1].time;
        std::cout << "======================================================================================> [[[[[[[[[ icp  success ,icp score: "<<icp.getFitnessScore() << std::endl;
        std::cout<<"line:"<<__LINE__<<std::endl;
        double inlier_value = ceresIcpTool_handle.find_tranfrom_of_two_mappings(*cureKeyframeCloud, *prevKeyframeCloud );

        if(inlier_value!=inlier_value || inlier_value > 0.5){
            std::cout<<"inlier_value:"<<inlier_value<<std::endl;
            return false;
        }
        std::cout<<"line:"<<__LINE__<<std::endl;
        auto ICP_q = ceresIcpTool_handle.m_q_w_curr;
        std::cout<<"line:"<<__LINE__<<std::endl;
        auto ICP_t = ceresIcpTool_handle.m_t_w_curr;
        std::cout<<"line:"<<__LINE__<<std::endl;

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.prerotate(ICP_q.cast<float>());
        transform.pretranslate(ICP_t.cast<float>());
        pcl::PointCloud<pcl::PointXYZINormal> transformed_cloud;
        pcl::transformPointCloud(*cureKeyframeCloud, transformed_cloud, transform);

        if(transformed_cloud.points.size()>0){
            pcl::io::savePCDFileBinary("/home/kilox/wutong/transformed_cloud.pcd", transformed_cloud);
        }
        

        sensor_msgs::PointCloud2 source_transform;
        pcl::toROSMsg( transformed_cloud, source_transform );
        source_transform.header.stamp = ros::Time::now(); 
        source_transform.header.frame_id = "camera_init"; // world; camera_init
        pubLoopDetectSourceTransform.publish( source_transform );

        transformed_cloud.clear();
        std::cout<<"inlier_value:"<<inlier_value<<std::endl;

            // 闭环优化得到的当前关键帧与闭环关键帧之间的位姿变换
        sensor_msgs::PointCloud2 source_loop_detect;
        pcl::toROSMsg( *cureKeyframeCloud, source_loop_detect );
        source_loop_detect.header.stamp = ros::Time::now(); 
        source_loop_detect.header.frame_id = "camera_init"; // world; camera_init
        pubLoopDetectSource.publish( source_loop_detect );

        sensor_msgs::PointCloud2 target_loop_detect;
        pcl::toROSMsg( *prevKeyframeCloud, target_loop_detect );
        target_loop_detect.header.stamp = ros::Time::now(); 
        target_loop_detect.header.frame_id = "camera_init"; // world; camera_init
        pubLoopDetectTarget.publish( target_loop_detect );

        
        pcl::io::savePCDFileBinary("/home/kilox/wutong/unused_result.pcd", *unused_result);
        pcl::io::savePCDFileBinary("/home/kilox/wutong/prevKeyframeCloud.pcd", *prevKeyframeCloud);

        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        correctionLidarFrame = transform;

        // 闭环优化前当前帧位姿
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // 闭环优化后当前帧位姿 
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw); //  获取上一帧 相对 当前帧的 位姿
        gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
        
        // 闭环匹配帧的位姿
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore() ; //  loop_clousre  noise from icp 不确定性
        noiseScore = inlier_value;
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
        std::cout<<__LINE__<<" " << "loopNoiseQueue   =   " << noiseScore << std::endl;

        // 只是插空查找了一些回环，如果有回环的话，就更新一下当前的位置，等待在下一次计算时根据回环信息进行查找
        // 添加闭环因子需要的数据
        mtx_loopdetect.lock();
        // 存储回环的索引
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        // 这个是新的计算的两帧之间的正确的相对位置关系
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        // 存储回环检测icp之后的位置不确定度
        loopNoiseQueue.push_back(constraintNoise);
        mtx_loopdetect.unlock();

        loopIndexContainer[loopKeyCur] = loopKeyPre; // 存储和当前帧产生回环的关键帧的索引   使用hash map 存储回环对

        std::cout<<"视觉回环成功"<<std::endl;
        //exit(0);
        return true;
    }

    bool performLoopClosureFinish(){
        //指定要多少个关键帧组成子地图
        int historyKeyframeSearchNum = 20;
        int historyKeyframeSearchNum_source = 20;
        //如果没有关键帧，则返回
        if (cloudKeyPoses3D->points.empty() == true)
        {
            return false;
        }

        if(USE_LOOP_PARAM){
            historyKeyframeSearchNum = ld_historyKeyframeSearchNum_target_finish;
            historyKeyframeSearchNum_source = ld_historyKeyframeSearchNum_source_finish;
        }
        
        //todo:这里的添加有问题
        if(loopdetect_enable == 0 && loopdetect_enable_just_final == 0){
            std::cout<<"line:"<<__LINE__<<std::endl;
            std::cout<<"loopdetect_enable:"<<loopdetect_enable<<" loopdetect_enable_just_final:"<<loopdetect_enable_just_final<<std::endl;
            return false;
        }

        /**
        if(loopdetect_enable == 0){
            return false;
        }
        */
        //return false;用于调试一次回环的结果

        mtx_loopdetect.lock();
        //拷贝最新的轨迹点云以及位姿 这里可以进行主动内存释放
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx_loopdetect.unlock();

        int loopKeyPre = 0;
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
       

        // 提取当前的关键帧的点云
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cureKeyframeCloud(new pcl::PointCloud<pcl::PointXYZINormal>()); //  cue keyframe
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr prevKeyframeCloud(new pcl::PointCloud<pcl::PointXYZINormal>()); //   history keyframe submap
        {
            // 提取当前关键帧特征点集合，降采样
            //loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 20); //  将cur keyframe 转换到world系下
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, historyKeyframeSearchNum_source); //  将cur keyframe 转换到world系下
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum); //  选取historyKeyframeSearchNum个keyframe拼成submap
        }

        //GeneralizedIterativeClosestPoint  IterativeClosestPoint
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
        icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZINormal>());
        
        icp.align(*unused_result);
        float historyKeyframeFitnessScore = 0.5;

        
        // 未收敛，或者匹配不够好
        if (false && (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore))
        {
            std::cout<<"======================================================================================> [[[[[[[[[finish icp fail,icp score:"<< icp.getFitnessScore()<<std::endl;
            return false;
        }
        last_loop_detect_time = copy_cloudKeyPoses6D->points[copy_cloudKeyPoses6D->points.size()-1].time;
        std::cout << "======================================================================================> [[[[[[[[[finish icp  success ,icp score: "<<icp.getFitnessScore() << std::endl;
        std::cout<<"line:"<<__LINE__<<std::endl;

        //============================== new finish start======================================
        Eigen::Affine3f curent_pose_ceres = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        Eigen::Affine3f pre_pose_ceres = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyPre]);

        Eigen::Affine3f rotation_matrx = Eigen::Affine3f::Identity();
        
        Eigen::Affine3f current_pose_to_pre_pose = pre_pose_ceres * curent_pose_ceres.inverse();

        //如果起点和终点的距离小于1米 就不进行转换,如果大于1米,就进行位置转换
        std::cout<<"line:"<<__LINE__<<std::endl;
        //Eigen::Matrix3f R_original = curent_pose_ceres.rotation();

        std::cout<<"====> curent_pose_ceres:"<<std::endl<<curent_pose_ceres.matrix().cast<double>();
        std::cout<<"====> pre_pose_ceres:"<<std::endl<<pre_pose_ceres.matrix().cast<double>();
        std::cout<<"====> current_pose_to_pre_pose:"<<std::endl<<current_pose_to_pre_pose.matrix().cast<double>();
        bool use_first_point_to = true;
        Eigen::Matrix4d matrix4d_curr_world_to_pre = current_pose_to_pre_pose.matrix().cast<double>();

        Eigen::Vector3d translation_diff = matrix4d_curr_world_to_pre.block<3, 1>(0, 3);
        if(translation_diff.norm() < 1 && use_first_localization_constraint == 1){
            std::cout<<"误差较小,不使用初始点约束:"<<translation_diff.norm()<<std::endl;
            use_first_point_to = false;
        }

        if(use_first_localization_constraint == 0){
            use_first_point_to = false;
        }

        pcl::PointCloud<pcl::PointXYZINormal> transformed_cloud_current_trans;

        Eigen::Matrix4d tmp_1 = Eigen::Matrix4d::Identity();
        tmp_1.block<3, 3>(0, 0) = matrix4d_curr_world_to_pre.block<3, 3>(0, 0).transpose();
        
        matrix4d_curr_world_to_pre = tmp_1*matrix4d_curr_world_to_pre;
        //matrix4d_curr_world_to_pre.block<3, 3>(0, 0) = curent_pose_ceres.matrix().cast<double>().block<3, 3>(0, 0);
        std::cout<<"=====> matrix4d_curr_world_to_pre:"<<std::endl<<matrix4d_curr_world_to_pre.matrix().cast<double>();
        pcl::transformPointCloud(*cureKeyframeCloud, transformed_cloud_current_trans, matrix4d_curr_world_to_pre);
        if(transformed_cloud_current_trans.points.size() > 0){
            pcl::io::savePCDFileBinary("/home/kilox/wutong/transformed_cloud_current_trans.pcd", transformed_cloud_current_trans);
        }else{
            std::cout<<"transformed_cloud_current_trans->points.size:" <<transformed_cloud_current_trans.points.size()<<std::endl;
        }
        //============================== new finish end======================================

        //double inlier_value = ceresIcpTool_handle.find_tranfrom_of_two_mappings(*cureKeyframeCloud, *prevKeyframeCloud );
        double inlier_value = 0;

        if(use_first_point_to){
            std::cout<<"使用初始点约束"<<std::endl;
            inlier_value = ceresIcpTool_handle.find_tranfrom_of_two_mappings(transformed_cloud_current_trans, *prevKeyframeCloud );
        }else{
            std::cout<<"距离原点距离较小,不使用初始点约束"<<std::endl;
            inlier_value = ceresIcpTool_handle.find_tranfrom_of_two_mappings(*cureKeyframeCloud, *prevKeyframeCloud );
        }

        std::cout<<"line:"<<__LINE__<<std::endl;
        auto ICP_q = ceresIcpTool_handle.m_q_w_curr;
        std::cout<<"line:"<<__LINE__<<std::endl;
        auto ICP_t = ceresIcpTool_handle.m_t_w_curr;
        std::cout<<"line:"<<__LINE__<<std::endl;

        //============================== new finish start======================================
        Eigen::Matrix4d ext_param3 = Eigen::Matrix4d::Identity();
        ext_param3.block<3, 3>(0, 0) = ICP_q.toRotationMatrix();
        ext_param3.block<3, 1>(0, 3) = ICP_t;
        Eigen::Matrix4d new_ext_param = ext_param3*matrix4d_curr_world_to_pre;

        Eigen::Vector3d translation = new_ext_param.block<3, 1>(0, 3).cast<double>();
        Eigen::Matrix3d rotation_matrix = new_ext_param.block<3, 3>(0, 0).cast<double>();
        Eigen::Quaterniond quaternion = Eigen::Quaterniond(rotation_matrix);
        quaternion.normalize();

        if(use_first_point_to){
            std::cout<<"使用初始点约束,根据修正解修改icp值"<<std::endl;
            ICP_q = quaternion;
            ICP_t = translation;
        }
        
         //============================== new finish end======================================

        if(USE_KISS_MATCH == 1){
            pcl::io::savePCDFileBinary("/home/kilox/wutong/source_kiss_match_final.pcd", *cureKeyframeCloud);
            pcl::io::savePCDFileBinary("/home/kilox/wutong/target_kiss_match_final.pcd", *prevKeyframeCloud);

            std::cout<<"=======================> begin run kiss_matcher_pointcloud final"<<std::endl;
            boost::filesystem::path path("/home/kilox/wutong/ext_param.txt");
            if (boost::filesystem::exists("/home/kilox/wutong/ext_param.txt")) {
                try {
                    boost::filesystem::remove("/home/kilox/wutong/ext_param.txt");
                    std::cout << "File deleted successfully." << std::endl;
                } catch (const boost::filesystem::filesystem_error& e) {
                    std::cerr << "Error deleting file: " << e.what() << std::endl;
                }
            } else {
                std::cout << "File does not exist." << std::endl;
            }
            system("/home/kilox/KISS-Matcher/cpp/examples/build/run_kiss_matcher /home/kilox/wutong/source_kiss_match_final.pcd /home/kilox/wutong/target_kiss_match_final.pcd 0.1 /home/kilox/wutong/ext_param.txt");
            
            if (boost::filesystem::exists("/home/kilox/wutong/ext_param.txt")){
                Eigen::Matrix4f ext_param = readExtrinsicMatrix("/home/kilox/wutong/ext_param.txt");
                // 提取平移部分（前三行第四列，float转double）
                Eigen::Vector3d translation = ext_param.block<3, 1>(0, 3).cast<double>();
                // 提取旋转矩阵（左上角3x3部分，float转double）
                Eigen::Matrix3d rotation_matrix = ext_param.block<3, 3>(0, 0).cast<double>();

                // 将旋转矩阵转换为四元数
                Eigen::Quaterniond quaternion = Eigen::Quaterniond(rotation_matrix);
                // 可选：对四元数进行归一化（消除数值误差）
                quaternion.normalize();

                ICP_q = quaternion;
                ICP_t = translation;
                std::cout<<"ext_param:"<<ext_param<<std::endl;
            }
            std::cout<<"=======================> end run kiss_matcher_pointcloud final"<<std::endl;
        }

        if(USE_TEASER_MATCH == 1 || ld_use_map_icp_teaser == 1){
            //Eigen::Matrix4d ext_param = map_icp_teaser(cureKeyframeCloud, prevKeyframeCloud, 30);
            Eigen::Matrix4d ext_param = map_icp_teaser(cureKeyframeCloud, prevKeyframeCloud, ld_map_icp_teaser_scale);

            // 提取平移部分（前三行第四列，float转double）
            Eigen::Vector3d translation = ext_param.block<3, 1>(0, 3).cast<double>();
            // 提取旋转矩阵（左上角3x3部分，float转double）
            Eigen::Matrix3d rotation_matrix = ext_param.block<3, 3>(0, 0).cast<double>();

            // 将旋转矩阵转换为四元数
            Eigen::Quaterniond quaternion = Eigen::Quaterniond(rotation_matrix);
            // 可选：对四元数进行归一化（消除数值误差）
            quaternion.normalize();

            
            std::cout<<"ext_param:"<<ext_param<<std::endl;

            std::cout<<"rotation:"<<std::endl<<ICP_q.toRotationMatrix()<<std::endl;
            std::cout<<"translation:"<<std::endl<<ICP_t<<std::endl;

            ICP_q = quaternion;
            ICP_t = translation;
            //exit(0);
        }

        float inlier_value_threshold = 0.4;
        
        if(USE_LOOP_PARAM){
            inlier_value_threshold = ld_inlier_value_finish;
        }

        //if(inlier_value!=inlier_value || inlier_value > 0.4){
        if(inlier_value!=inlier_value || inlier_value > inlier_value_threshold){
            std::cout<<"final ceres match fail, inlier_value:"<<inlier_value<<" inlier_value_threshold:"<<inlier_value_threshold<<std::endl;
            return false;
        }else{
            std::cout<<"final ceres match success, inlier_value:"<<inlier_value<<std::endl;
            std::cout<<"ICP_q:"<<std::endl<<ICP_q.toRotationMatrix()<<std::endl;
            std::cout<<"ICP_t:"<<std::endl<<ICP_t<<std::endl;
            
        }
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.prerotate(ICP_q.cast<float>());
        transform.pretranslate(ICP_t.cast<float>());
        pcl::PointCloud<pcl::PointXYZINormal> transformed_cloud;
        pcl::transformPointCloud(*cureKeyframeCloud, transformed_cloud, transform);
        std::cout<<"begin save transformed_cloud:"<<transformed_cloud.points.size()<<std::endl;
        if(transformed_cloud.points.size()>0){
            pcl::io::savePCDFileBinary("/home/kilox/wutong/transformed_cloud.pcd", transformed_cloud);
        }
        
        if(cureKeyframeCloud->points.size()>0){
            pcl::io::savePCDFileBinary("/home/kilox/wutong/cureKeyframeCloud.pcd", *cureKeyframeCloud);
        }
        
        std::cout<<"end save cureKeyframeCloud:"<<cureKeyframeCloud->points.size()<<std::endl;
        sensor_msgs::PointCloud2 source_transform;
        pcl::toROSMsg( transformed_cloud, source_transform );
        source_transform.header.stamp = ros::Time::now(); 
        source_transform.header.frame_id = "camera_init"; // world; camera_init
        pubLoopDetectSourceTransform.publish( source_transform );

        transformed_cloud.clear();
        std::cout<<"inlier_value:"<<inlier_value<<std::endl;
            // 闭环优化得到的当前关键帧与闭环关键帧之间的位姿变换
        sensor_msgs::PointCloud2 source_loop_detect;
        pcl::toROSMsg( *cureKeyframeCloud, source_loop_detect );
        source_loop_detect.header.stamp = ros::Time::now(); 
        source_loop_detect.header.frame_id = "camera_init"; // world; camera_init
        pubLoopDetectSource.publish( source_loop_detect );

        sensor_msgs::PointCloud2 target_loop_detect;
        pcl::toROSMsg( *prevKeyframeCloud, target_loop_detect );
        target_loop_detect.header.stamp = ros::Time::now(); 
        target_loop_detect.header.frame_id = "camera_init"; // world; camera_init
        pubLoopDetectTarget.publish( target_loop_detect );

        std::cout<<"begin save cureKeyframeCloud_finish:"<<cureKeyframeCloud->points.size()<<std::endl;

        if(cureKeyframeCloud->points.size()>0){
            pcl::io::savePCDFileBinary("/home/kilox/wutong/cureKeyframeCloud_finish.pcd", *cureKeyframeCloud);
        }
        
        if(prevKeyframeCloud->points.size()>0){
            pcl::io::savePCDFileBinary("/home/kilox/wutong/prevKeyframeCloud.pcd", *prevKeyframeCloud);
        }
        
        std::cout<<"end save prevKeyframeCloud:"<<prevKeyframeCloud->points.size()<<std::endl;

        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        //correctionLidarFrame = icp.getFinalTransformation();
        correctionLidarFrame = transform;

        // 闭环优化前当前帧位姿
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // 闭环优化后当前帧位姿 
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw); //  获取上一帧 相对 当前帧的 位姿
        gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
        
        // 闭环匹配帧的位姿
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore() ; //  loop_clousre  noise from icp 不确定性
        noiseScore = inlier_value*inlier_value/100000;
        noiseScore = 1e-2;
        //noiseScore = inlier_value;
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
        std::cout<<"line: "<<__LINE__ << " loopNoiseQueue   =   " << noiseScore << std::endl;

        // 只是插空查找了一些回环，如果有回环的话，就更新一下当前的位置，等待在下一次计算时根据回环信息进行查找
        // 添加闭环因子需要的数据
        mtx_loopdetect.lock();
        // 存储回环的索引
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        // 这个是新的计算的两帧之间的正确的相对位置关系
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        // 存储回环检测icp之后的位置不确定度
        loopNoiseQueue.push_back(constraintNoise);
        mtx_loopdetect.unlock();

        loopIndexContainer[loopKeyCur] = loopKeyPre; // 存储和当前帧产生回环的关键帧的索引   使用hash map 存储回环对
        std::cout<<"line:"<<__LINE__<<std::endl;
        return true;
    }

    void correctPosesFinish()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            std::cout<<"没有任何关键帧，建图结束"<<std::endl;
            return;
        }

        if (true)
        {

            int numPoses = isamCurrentEstimate.size();
            //save_loop_pcd_file("/home/kilox/wutong/finish_binary_source_"+std::to_string(cloudKeyPoses3D->points.size())+".pcd");
            //pcl::io::savePCDFileBinary("/home/kilox/wutong/finish_save_binary_source_trajectory_"+std::to_string(cloudKeyPoses3D->points.size())+".pcd", *cloudKeyPoses3D);
            for (int i = 0; i < numPoses; ++i)
            {
               
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();

                //updatePath(cloudKeyPoses6D->points[i]);
            }
            //save_loop_pcd_file("/home/kilox/wutong/finish_save_binary_final_"+ std::to_string(cloudKeyPoses3D->points.size()) +".pcd");
            ROS_INFO("====== finish save pcd =======");
        }
        
       write_camera_and_lidar_trajectory("/home/kilox/wutong/loop_detect/full_trajectory.txt");
       write_camera_and_lidar_trajectory_total("/home/kilox/wutong/loop_detect/full_trajectory_total_key.txt");
        
        /**
        vector<IMUST> x_buf;
        vector<pcl::PointCloud<PointTypeBALM>::Ptr> pl_fulls;
        unordered_map<VOXEL_LOC2, OCTO_TREE_ROOT*> surf_map;
         for (int i = 0; i < (int)surfCloudKeyFrames.size(); ++i)
        {
            std::cout<<"intsert i:"<<i<<std::endl;
            //没有降采样的原始去畸变点云
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZINormal>);
            //downSizeFilterICP_hba.setInputCloud(surfCloudKeyFrames[i]);
            //downSizeFilterICP_hba.filter(*cloud_ds);
            *cloud_ds = *surfCloudKeyFrames[i];
            Eigen::Affine3f current_Pose = pcl::getTransformation( cloudKeyPoses6D->points[i].x,  cloudKeyPoses6D->points[i].y, cloudKeyPoses6D->points[i].z,  cloudKeyPoses6D->points[i].roll,  cloudKeyPoses6D->points[i].pitch,  cloudKeyPoses6D->points[i].yaw);
            Eigen::Quaternionf rotationQuaternion(current_Pose.rotation());
            Eigen::Quaterniond rotationQuaternionDouble(rotationQuaternion.w(), rotationQuaternion.x(), rotationQuaternion.y(), rotationQuaternion.z());
            Eigen::Vector3d translationVector(current_Pose.translation().x(), current_Pose.translation().y(), current_Pose.translation().z());
            IMUST curr;
            curr.R = rotationQuaternionDouble.toRotationMatrix(); 
            curr.p = translationVector; 
            x_buf.push_back(curr);
            pl_fulls.push_back(cloud_ds);

        }
        int win_size = pl_fulls.size();
        VOX_HESS voxhess;
        voxhess.set_window_size(win_size);

        std::cout<<"cut_voxel"<<std::endl;
        for(int i=0; i<pl_fulls.size(); i++)
            cut_voxel(surf_map, *pl_fulls[i], x_buf[i], i,win_size);

        std::cout<<"line:"<<__LINE__<<std::endl<<std::endl;
        std::cout<<"surf_map:"<<surf_map.size()<<std::endl;
        
        for(auto iter=surf_map.begin(); iter!=surf_map.end() ; iter++)
        {
            
            iter->second->recut(win_size);
            iter->second->tras_opt(voxhess, win_size);
            //iter->second->tras_display(pl_send, win_size);
        }

        if(voxhess.plvec_voxels.size() < 3 * x_buf.size())
        {
        printf("Initial error too large.\n");
        printf("Please loose plane determination criteria for more planes.\n");
        printf("The optimization is terminated.\n");
        exit(0);
        }

        write_camera_and_lidar_trajectory("/home/kilox/wutong/loop_detect/full_trajectory.txt");

        publishGenMap(x_buf,pl_fulls);
        std::cout<<"damping_iter"<<std::endl;
        publishMap(x_buf,pl_fulls);
        sleep(6);

        std::cout<<"begin to balm2 optimize:"<<std::endl;
        sleep(10);
        ///std::cout<<"voxhess size:"<<voxhess.size()<<std::endl;
        BALM2 opt_lsv;
        opt_lsv.set_win_size(win_size);
        opt_lsv.damping_iter(x_buf, voxhess);
        


        std::cout<<"publishMap"<<std::endl;
        publishMap(x_buf,pl_fulls);
        std::cout<<"end success"<<std::endl;
        */
        fast_livo::ImagePoseTime srv_search;
        srv_search.request.img_time = 0;

        std_msgs::Int32 is_loop_data;
        is_loop_data.data = 2;
        std_msgs::String path_s;
        path_s.data = "";
        srv_search.request.is_loop = is_loop_data;
        srv_search.request.path = "";
        srv_search.request.img_name = "";
        
        if (client_search.call(srv_search)) {
            ROS_INFO("search Service response finish timestamp: %f", srv_search.response.img_time_result);
            double search_img_time = srv_search.response.img_time_result;
            if(search_img_time == 0){
                //std::cout<<"搜索结束失败"<<std::endl;
                std::cout<<"搜索结束成功"<<std::endl;  
            }else{
                //当前的雷达可能与之前的某一帧雷达数据相似
                std::cout<<"搜索结束成功"<<std::endl;    
            }
        } else {
            ROS_ERROR("Failed to call search service finish imageposetime");
        }

        //推送里程计与回环成功消息
        std::cout<<"===== 推送里程计与回环成功消息 ======"<<std::endl;
        std_msgs::String msg_odom_finish_str;
        std::stringstream ss_odom_finish_str;
        ss_odom_finish_str << "finish";
        msg_odom_finish_str.data = ss_odom_finish_str.str();
        pub_odom_finish_str_msg.publish(msg_odom_finish_str);

        sleep(1000);
        //malloc_trim(0);
        
        /**
           std::vector<pcl::PointCloud<PointTypeHBAType>::Ptr> buffer_pcd;
           std::vector<mypcl::pose> buffer_pose;
        */

       /**
        std::cout<<"line:"<<__LINE__<<std::endl;
        
        std::cout<<"line:"<<__LINE__<<std::endl;
        std::vector<pcl::PointCloud<PointTypeHBAType>::Ptr> buffer_pcd;
        std::vector<mypcl::pose> buffer_pose;
        for (int i = 0; i < (int)surfCloudKeyFrames.size(); ++i)
        {
            std::cout<<"intsert i:"<<i<<std::endl;
            //没有降采样的原始去畸变点云
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZINormal>);
            downSizeFilterICP_hba.setInputCloud(surfCloudKeyFrames[i]);
            downSizeFilterICP_hba.filter(*cloud_ds);
            Eigen::Affine3f current_Pose = pcl::getTransformation( cloudKeyPoses6D->points[i].x,  cloudKeyPoses6D->points[i].y, cloudKeyPoses6D->points[i].z,  cloudKeyPoses6D->points[i].roll,  cloudKeyPoses6D->points[i].pitch,  cloudKeyPoses6D->points[i].yaw);
            Eigen::Quaternionf rotationQuaternion(current_Pose.rotation());
            Eigen::Quaterniond rotationQuaternionDouble(rotationQuaternion.w(), rotationQuaternion.x(), rotationQuaternion.y(), rotationQuaternion.z());
            Eigen::Vector3d translationVector(current_Pose.translation().x(), current_Pose.translation().y(), current_Pose.translation().z());
            mypcl::pose pose;
            pose.q = rotationQuaternionDouble;
            pose.t = translationVector;
            std::cout<<"line:"<<__LINE__<<std::endl;
            buffer_pcd.push_back(cloud_ds);
            std::cout<<"line:"<<__LINE__<<std::endl;
            buffer_pose.push_back(pose);
            std::cout<<"line:"<<__LINE__<<std::endl;

        }
        HBA hba(3, "", 16,buffer_pcd,buffer_pose);

        for(int i = 0; i < hba.total_layer_num-1; i++)
        {
            std::cout<<"i:"<<i<<std::endl;
            distribute_thread(hba.layers[i], hba.layers[i+1]);
            std::cout<<"line:"<<__LINE__<<std::endl;
            hba.update_next_layer_state(i);
            std::cout<<"line:"<<__LINE__<<std::endl;
        }
        global_ba(hba.layers[hba.total_layer_num-1]);
        std::vector<mypcl::pose> result = hba.pose_graph_optimization();
        
        std::cout<<"line:"<<__LINE__<<std::endl;
        //hba.set_pose_vec();
        */

        /**
        HBAConfig m_hba_config;
        m_hba_config.window_size = 20;
        m_hba_config.down_sample = 0.2;
        
        std::shared_ptr<HBA> m_hba = std::make_shared<HBA>(m_hba_config);

        for (int i = 0; i < (int)surfCloudKeyFrames.size(); ++i)
        {
            std::cout<<"intsert i:"<<i<<std::endl;
            //没有降采样的原始去畸变点云
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZINormal>);
            downSizeFilterICP_hba.setInputCloud(surfCloudKeyFrames[i]);
            downSizeFilterICP_hba.filter(*cloud_ds);
            Eigen::Affine3f current_Pose = pcl::getTransformation( cloudKeyPoses6D->points[i].x,  cloudKeyPoses6D->points[i].y, cloudKeyPoses6D->points[i].z,  cloudKeyPoses6D->points[i].roll,  cloudKeyPoses6D->points[i].pitch,  cloudKeyPoses6D->points[i].yaw);
            Eigen::Quaternionf rotationQuaternion(current_Pose.rotation());
            Eigen::Quaterniond rotationQuaternionDouble(rotationQuaternion.w(), rotationQuaternion.x(), rotationQuaternion.y(), rotationQuaternion.z());
            Eigen::Vector3d translationVector(current_Pose.translation().x(), current_Pose.translation().y(), current_Pose.translation().z());
            Pose pose;
            pose.t = translationVector;
            pose.r = rotationQuaternionDouble.toRotationMatrix();
            
            m_hba->insert(cloud_ds, pose);
           

        }

        m_hba_config.hba_iter = 10;
        for (size_t i = 0; i < m_hba_config.hba_iter; i++)
        {
                std::cout<<"optimize:"<<i<<" max iter:"<<m_hba_config.hba_iter<<std::endl;
                publishMap(m_hba);
                sleep(3);
                m_hba->optimize();
                
        }
        */
        
        
    }

    /**
    void publishGenMap(vector<IMUST>& x_buf,vector<pcl::PointCloud<PointTypeBALM>::Ptr>& pl_fulls)
    {
       pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZINormal>);
        for(size_t i = 0; i < x_buf.size(); i++)
        {
        Eigen::Vector3d t = x_buf[i].p;
        Eigen::Quaterniond q(x_buf[i].R);

        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
        transformation_matrix.block<3, 3>(0, 0) = x_buf[i].R;
        transformation_matrix.block<3, 1>(0, 3) = x_buf[i].p;
        
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::transformPointCloud(*pl_fulls[i], *transformed_cloud, transformation_matrix.cast<float>());

        *cloud_ds += *transformed_cloud;

        }
       
        //pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = m_hba->getMapPoints();
        //PointCloudCalibrator calibrator;
        //std::cout<<"begin calibration  "<<std::endl;
        //Eigen::Affine3f transform = calibrator.calibratePointCloudMap(cloud_ds);
        //pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        //pcl::transformPointCloud(*cloud_ds, *transformedCloud, transform);
        //pcl::io::savePCDFileBinary("/home/kilox/wutong/calibration_cloud.pcd", *transformedCloud);

        //std::cout<<"extractFreeSpace"<<std::endl;
        //calibrator.extractFreeSpace(transformedCloud, "/home/kilox/wutong/free_space.svg");
        //std::cout<<"end extractFreeSpace"<<std::endl;

        //std::cout<<"extractGround"<<std::endl;
        //Plane2 ground;
        //calibrator.extractGround(transformedCloud, ground);
        //std::cout<<"end extractGround"<<std::endl;

        // // 墙面提取
        //std::cout<<"extractWall"<<std::endl;
        //vector<Plane2> walls;
        //calibrator.extractWall(transformedCloud, walls);

        std::cout<<"end extractWall"<<std::endl;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_ds, cloud_msg);
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        m_cloud_calibration_pub.publish(cloud_msg);
        sleep(1000);
    }
    **/

    /**
    void publishMap(vector<IMUST>& x_buf,vector<pcl::PointCloud<PointTypeBALM>::Ptr>& pl_fulls)
    {
       pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZINormal>);
        for(size_t i = 0; i < x_buf.size(); i++)
        {
        Eigen::Vector3d t = x_buf[i].p;
        Eigen::Quaterniond q(x_buf[i].R);

        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
        transformation_matrix.block<3, 3>(0, 0) = x_buf[i].R;
        transformation_matrix.block<3, 1>(0, 3) = x_buf[i].p;
        
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::transformPointCloud(*pl_fulls[i], *transformed_cloud, transformation_matrix.cast<float>());

        *cloud_ds += *transformed_cloud;

        }

        //pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = m_hba->getMapPoints();
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_ds, cloud_msg);
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        m_cloud_pub.publish(cloud_msg);

       
        
    }
    */

    /**
    void publishMap(std::vector<mypcl::pose> pose_vec,std::vector<pcl::PointCloud<PointTypeHBAType>::Ptr> buffer_pcd)
    {
        Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(), pose_vec[0].q.z());
        Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZINormal>);
        for(size_t i = 0; i < pose_vec.size(); i++)
        {
        Eigen::Vector3d t = q0.inverse()*(pose_vec[i].t-t0);
        Eigen::Quaterniond q;
        q.w() = (q0.inverse()*pose_vec[i].q).w();
        q.x() = (q0.inverse()*pose_vec[i].q).x();
        q.y() = (q0.inverse()*pose_vec[i].q).y();
        q.z() = (q0.inverse()*pose_vec[i].q).z();
        Eigen::Affine3f current_Pose;
        current_Pose.prerotate(q.cast<float>());
        current_Pose.pretranslate(t.cast<float>());
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::transformPointCloud(*buffer_pcd[i], *transformed_cloud, current_Pose);
        *cloud_ds += *transformed_cloud;

        }

        //pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = m_hba->getMapPoints();
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_ds, cloud_msg);
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        m_cloud_pub.publish(cloud_msg);
    }
    */
    /**
    void publishMap(std::shared_ptr<HBA> m_hba)
    {
        
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = m_hba->getMapPoints();
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        m_cloud_pub.publish(cloud_msg);
    }
    */
    void saveKeyFramesAndFactorFinish()
    {

        //addOdomFactor();
        std::cout<<"line:"<<__LINE__<<std::endl;
        addLoopFactor();
        std::cout<<"line:"<<__LINE__<<std::endl;
        // 执行优化
        isam->update(gtSAMgraph, initialEstimate);
        std::cout<<"initialEstimate size:"<<initialEstimate.size()<<" cloudKeyPoses3D:"<<cloudKeyPoses3D->points.size()<<std::endl;

        if(cloudKeyPoses3D->points.size() <= 1){
            return;
        }
        /**
        if(initialEstimate.size() == 0){
            return;
        }
        */
        std::cout<<"line:"<<__LINE__<<std::endl;
        isam->update();
        std::cout<<"=================================> aLoopIsClosed finish:"<<aLoopIsClosed<<std::endl;
        if (aLoopIsClosed == true) // 有回环因子，多update几次
        {
            std::cout<<"=================================> has loopdetect factor finish"<<std::endl;

            for(int i=0;i<10;i++){
                isam->update();
                isam->update();
                isam->update();
                isam->update();
                isam->update();
                // 计算总损失值（加权残差平方和）
                //double loss = graph->error(current_estimate);
                //std::cout << "当前损失值: " << loss << std::endl;
            }

        }
        // update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了,也就是说图已经保存在isam里面了？
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        pcl::PointXYZINormal thisPose3D;
        PointTypePose thisPose6D;
        gtsam::Pose3 latestEstimate;

        // 优化结果
        isamCurrentEstimate = isam->calculateBestEstimate();
        // 当前帧位姿结果
        latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);
        // 位姿协方差
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);
    }
    //以下为回环检测
    //回环检测线程
    void loopClosureThread()
    {
        /**
        if (loopClosureEnableFlag == false)
        {
            std::cout << "loopClosureEnableFlag   ==  false " << endl;
            return;
        }
        */
        bool startFlag = true;
        ros::Rate rate(4); //   回环频率
        std::cout<<"========================> start loopclusureThread ======================="<<std::endl;
        //exit(0);
        while (ros::ok() && startFlag)
        {
            rate.sleep();
            bool result = performLoopClosure();   //  回环检测，循环进行回环检测
            while(aLoopIsClosed == true){
                std::cout<<"=========> 检测到回环，等待回环处理完成再进行回环检测"<<std::endl;
                sleep(1);
            }

            if(result){
                std::cout<<"=========> 检测到回环，休眠10s再进行检测"<<std::endl;
                sleep(20);
                std::cout<<"=========> 检测到回环，完成休眠10s，继续检测"<<std::endl;
            }else{
                //std::cout<<"=====> 未检测到回环：正在进行图像回环检测"<<std::endl;
                performLoopClosureIMG();
            }
            //visualizeLoopClosure(); // rviz展示闭环边
        }
    }
    //******************** backend end **************
public:
  std::string cam_model_string;
  LIVMapper(ros::NodeHandle &nh);
  ~LIVMapper();
  void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void savePCD();
  void processImu();
  
  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback(const ros::TimerEvent &e);
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
 
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);

//   void img_compressed_cbk(const sensor_msgs::CompressedImage &msg_in);   // 07.14
  void img_compressed_cbk(const sensor_msgs::CompressedImageConstPtr &msg_in);   // 07.14晚

  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);
  void publish_mavros(const ros::Publisher &mavros_pose_publisher);
  void publish_path(const ros::Publisher pubPath);
  void readParameters(ros::NodeHandle &nh);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string root_dir;
  string lid_topic, imu_topic, seq_name, img_topic;
  string img_topic_left_compressed, img_topic_right_compressed; // 07.14 加入压缩图像源数据流
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false;
  int pcd_save_interval = -1, pcd_index = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::Imu> prop_imu_buffer;
  sensor_msgs::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::Odometry imu_prop_odom;
  ros::Publisher pubImuPropOdom;
  double imu_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  double IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_pre, fout_out, fout_pcd_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;

  ros::Publisher plane_pub;
  ros::Publisher voxel_pub;
  ros::Subscriber sub_pcl;
  ros::Subscriber sub_imu;
  ros::Subscriber sub_img;
  ros::Publisher pubLaserCloudFullRes;
  ros::Publisher pubNormal;
  ros::Publisher pubSubVisualMap;
  ros::Publisher pubLaserCloudEffect;
  ros::Publisher pubLaserCloudMap;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubPath;
  ros::Publisher pubLaserCloudDyn;
  ros::Publisher pubLaserCloudDynRmed;
  ros::Publisher pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  ros::Publisher mavros_pose_publisher;
  ros::Timer imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif