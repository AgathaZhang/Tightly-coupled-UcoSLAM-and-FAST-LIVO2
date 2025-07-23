// An example showing TEASER++ registration with FPFH features with the Stanford bunny model
#include <chrono>
#include <iostream>
#include <random>

#include <Eigen/Core>

#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser/matcher.h>
#include "teaser_cpp_fpfh.hpp"
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
//******************** backend  start ******************************
class EigenPoseTeaser {
public:
    EigenPoseTeaser() {
        R_ << 1.0, .0, .0, .0, 1.0, .0, .0, .0, 1.0;
        t_ << .0, .0, .0;
    }

    EigenPoseTeaser(double x, double y, double z, double roll, double pitch, double yaw){
        t_ << x, y, z;
        Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
                               Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())) *
                               Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
        R_ = q.toRotationMatrix();
    }

    EigenPoseTeaser(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
        R_ = q.toRotationMatrix();
        t_ = t;
    }

    EigenPoseTeaser(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) : R_(R), t_(t) {}

    EigenPoseTeaser(const EigenPoseTeaser& e_pose)
    {
        R_ = e_pose.R_;
        t_ = e_pose.t_;
    }


/*        EigenPoseTeaser(const geometry_msgs::Pose& msg) {
            Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
            Eigen::Vector3d t;
            t << msg.position.x, msg.position.y, msg.position.z;
            R_ = q.toRotationMatrix();
            t_ = t;
        }*/
    EigenPoseTeaser(Eigen::Affine3f pose) {
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
    EigenPoseTeaser(const gtsam::Pose3& pose) {
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

    EigenPoseTeaser Get2DPose() { return EigenPoseTeaser(t_[0], t_[1], 0, 0, 0, GetRPY()[2]); }

    EigenPoseTeaser& operator=(const EigenPoseTeaser& e_pose){
        R_ = e_pose.R_;
        t_ = e_pose.t_;
    }

    EigenPoseTeaser operator*(const EigenPoseTeaser& T_rel) { return EigenPoseTeaser(R_ * T_rel.R_, R_ * T_rel.t_ + t_); }

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

    EigenPoseTeaser inverse() { return EigenPoseTeaser(R_.inverse(), -R_.inverse() * t_); }

    //EigenPoseTeaser CalRelativeEigenPoseTeaser(const EigenPoseTeaser& pose_target);

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

    EigenPoseTeaser getEigenPoseTeaserByMatrix4d(Eigen::Matrix4d pOut){
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

        EigenPoseTeaser ep(r,t);
        return ep;
    }

    Eigen::Quaterniond GetQ() {
        Eigen::Quaterniond eigen_q(R_);
        return eigen_q;
    }

    double DisTo(EigenPoseTeaser& other) {
        EigenPoseTeaser delta = this->inverse() * other;
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

    EigenPoseTeaser GetRatioPose(double ratio) {
        return EigenPoseTeaser(ratio * t_[0], ratio * t_[1], ratio * t_[2], ratio * GetRPY()[0], ratio * GetRPY()[1], ratio * GetRPY()[2]);
    }

    void Reset(const double& x = .0, const double& y = .0, const double& z = .0, const double& roll = .0, const double& pitch = .0, const double& yaw = .0);
    void Reset(const Eigen::Vector3d& tran, const Eigen::Vector3d& rot) { Reset(tran.x(), tran.y(), tran.z(), rot.x(), rot.y(), rot.z()); }

    Eigen::Matrix3d R_;
    Eigen::Vector3d t_;

    friend std::ostream& operator<<(std::ostream& os, const EigenPoseTeaser& pose) {
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

// Macro constants for generating noise and outliers
#define NOISE_BOUND 0.001
#define N_OUTLIERS 1700
#define OUTLIER_TRANSLATION_LB 5
#define OUTLIER_TRANSLATION_UB 10

inline double getAngularError(Eigen::Matrix3d R_exp, Eigen::Matrix3d R_est) {
  return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0)));
}

void addNoiseAndOutliers(Eigen::Matrix<double, 3, Eigen::Dynamic>& tgt) {
  // Add uniform noise
  Eigen::Matrix<double, 3, Eigen::Dynamic> noise =
      Eigen::Matrix<double, 3, Eigen::Dynamic>::Random(3, tgt.cols()) * NOISE_BOUND / 2;
  tgt = tgt + noise;

  // Add outliers
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis2(0, tgt.cols() - 1); // pos of outliers
  std::uniform_int_distribution<> dis3(OUTLIER_TRANSLATION_LB,
                                       OUTLIER_TRANSLATION_UB); // random translation
  std::vector<bool> expected_outlier_mask(tgt.cols(), false);
  for (int i = 0; i < N_OUTLIERS; ++i) {
    int c_outlier_idx = dis2(gen);
    assert(c_outlier_idx < expected_outlier_mask.size());
    expected_outlier_mask[c_outlier_idx] = true;
    tgt.col(c_outlier_idx).array() += dis3(gen); // random translation
  }
}

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

float signf_pcl(float value) {
    return (value > 0) - (value < 0);
}
template <typename T>
pcl::PointCloud<T> downsample_laser_cloud3(typename pcl::PointCloud<T>::Ptr cloud, float downsample_rate) {
    pcl::VoxelGrid<T> downSizeFilterSurfDisplay;
    typename pcl::PointCloud<T>::Ptr pc_rigid(new pcl::PointCloud<T>());
    typename pcl::PointCloud<T>::Ptr down_sample_cloud(new pcl::PointCloud<T>);

    pcl::PointCloud<T> merge_point_cloud;

    merge_point_cloud = *down_sample_cloud;
    float max_x, max_y, min_x, min_y, max_z, min_z;

    std::map<int, std::map<int, typename pcl::PointCloud<T>::Ptr>> meta_group;

    for (int i = 0; i < cloud->points.size(); i++) {
        T pt = cloud->points[i];

        if (pt.x != pt.x || pt.y != pt.y || pt.z != pt.z) {
            continue;
        }
        // 其他所有点, 包括预先地图中没有定义点类型的点也算进去
        // PointType pt = pc_map->points[i]; pc_rigid->points.push_back(pt);
        pc_rigid->points.push_back(pt);

        max_x = std::max(pt.x, max_x);
        max_y = std::max(pt.y, max_y);
        min_x = std::min(pt.x, min_x);
        min_y = std::min(pt.y, min_y);
        max_z = std::max(pt.z, max_z);
        min_z = std::min(pt.z, min_z);

        float resolution = 10;

        int col = pt.x / resolution + signf_pcl(pt.x) * 0.5f;
        int row = pt.y / resolution + signf_pcl(pt.y) * 0.5f;

        if (meta_group.find(row) == meta_group.end()) {
            meta_group[row] = std::map<int, typename pcl::PointCloud<T>::Ptr>();
        }

        if (meta_group[row].find(col) == meta_group[row].end()) {
            meta_group[row][col] = typename pcl::PointCloud<T>::Ptr(new pcl::PointCloud<T>);
        }
        meta_group[row][col]->push_back(pt);
    }

    for (auto &row_group : meta_group) {
        for (auto &meta : row_group.second) {
            if (meta_group[row_group.first][meta.first]->size() > 0) {
                downSizeFilterSurfDisplay.setInputCloud(meta_group[row_group.first][meta.first]);
                downSizeFilterSurfDisplay.setLeafSize(downsample_rate, downsample_rate, downsample_rate);
                downSizeFilterSurfDisplay.filter(*down_sample_cloud);
                merge_point_cloud += *down_sample_cloud;
            }
        }
    }

    return merge_point_cloud;
}

Eigen::Matrix4d transform_teaser_method(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source_transform_teaser, pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target_transform_teaser,float scale_value){
  teaser::PLYReader reader;
  teaser::PointCloud src_cloud;
  teaser::PointCloud tgt_cloud2;

  for(int i=0;i<source_transform_teaser->points.size();i++){
    src_cloud.push_back(
        {static_cast<float>(source_transform_teaser->points[i].x), static_cast<float>(source_transform_teaser->points[i].y), static_cast<float>(source_transform_teaser->points[i].z)});
  }

  for(int i=0;i<target_transform_teaser->points.size();i++){
    tgt_cloud2.push_back(
        {static_cast<float>(target_transform_teaser->points[i].x), static_cast<float>(target_transform_teaser->points[i].y), static_cast<float>(target_transform_teaser->points[i].z)});
  }

  //auto status = reader.read("/home/kilox/test_source.pcd", src_cloud);
  //auto statu2s = reader.read("/home/kilox/test_target.pcd", tgt_cloud2);
  
  int N = src_cloud.size();
  int N_target = tgt_cloud2.size();
  // Convert the point cloud to Eigen
  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);

  Eigen::Matrix<double, 3, Eigen::Dynamic> src_rgt(3, N_target);
  for (size_t i = 0; i < N_target; ++i) {
    src_rgt.col(i) << tgt_cloud2[i].x, tgt_cloud2[i].y, tgt_cloud2[i].z;
  }

  for (size_t i = 0; i < N; ++i) {
    src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
  }

  // Homogeneous coordinates
  Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
  src_h.resize(4, src.cols());
  src_h.topRows(3) = src;
  src_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);


  Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h;
  tgt_h.resize(4, src_rgt.cols());
  tgt_h.topRows(3) = src_rgt;
  tgt_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N_target);

  // Apply an arbitrary SE(3) transformation
  Eigen::Matrix4d T;
  // clang-format off
  T << 9.96926560e-01,  6.68735757e-02, -4.06664421e-02, -1.15576939e-01,
      -6.61289946e-02, 9.97617877e-01,  1.94008687e-02, -3.87705398e-02,
      4.18675510e-02, -1.66517807e-02,  9.98977765e-01, 1.14874890,
      0,              0,                0,              1;
  // clang-format on
  // Apply transformation
  //Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h = src_h_tgt;
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = tgt_h.topRows(3);

  // Add some noise & outliers
  //addNoiseAndOutliers(tgt);

  // Convert to teaser point cloud
  teaser::PointCloud tgt_cloud;
  for (size_t i = 0; i < tgt.cols(); ++i) {
    tgt_cloud.push_back({static_cast<float>(tgt(0, i)), static_cast<float>(tgt(1, i)),
                         static_cast<float>(tgt(2, i))});
  }

  // Compute FPFH
  teaser::FPFHEstimation fpfh;
  //float scale_value = 10;//10 30
  auto obj_descriptors = fpfh.computeFPFHFeatures(src_cloud, 0.02*scale_value, 0.04*scale_value);
  auto scene_descriptors = fpfh.computeFPFHFeatures(tgt_cloud, 0.02*scale_value, 0.04*scale_value);

  teaser::Matcher matcher;
  auto correspondences = matcher.calculateCorrespondences(
      src_cloud, tgt_cloud, *obj_descriptors, *scene_descriptors, false, true, false, 0.95);

  // Run TEASER++ registration
  // Prepare solver parameters
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = NOISE_BOUND;
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 100;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm =
      teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = 0.005;

  // Solve with TEASER++
  teaser::RobustRegistrationSolver solver(params);
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  solver.solve(src_cloud, tgt_cloud, correspondences);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  auto solution = solver.getSolution();

  Eigen::Vector3d t_so = solution.translation;
  Eigen::Matrix3d R_so = solution.rotation;
  Eigen::Matrix4d transformationMatrix_ICP = Eigen::Matrix4d::Identity();
  transformationMatrix_ICP.block<3,3>(0,0) = R_so;
  transformationMatrix_ICP.block<3,1>(0,3) = t_so;

  return transformationMatrix_ICP;
}

Eigen::Matrix4d map_icp_teaser(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source_merge_cloud, pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target_merge_cloud,float scale_value){

    std::cout<<"source_merge_cloud size:"<<source_merge_cloud->points.size()<<std::endl;
    std::cout<<"target_merge_cloud size:"<<target_merge_cloud->points.size()<<std::endl;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_merge_cloud_downsample(new pcl::PointCloud<pcl::PointXYZINormal>);
    *source_merge_cloud_downsample = downsample_laser_cloud3<pcl::PointXYZINormal>(source_merge_cloud, 0.05);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_merge_cloud_downsample(new pcl::PointCloud<pcl::PointXYZINormal>);
    *target_merge_cloud_downsample = downsample_laser_cloud3<pcl::PointXYZINormal>(target_merge_cloud, 0.05);
    //pcl::io::savePCDFileBinary("/home/kilox/catkin_r3live/src/convertpointcloud/build/total.pcd", *merge_cloud_ptr);

    Eigen::Matrix4d transformationMatrix_ICP = Eigen::Matrix4d::Identity();
    if(source_merge_cloud_downsample->points.size()<=0 || target_merge_cloud_downsample->points.size()<=0){
        std::cout<<"source_merge_cloud_downsample size:"<<source_merge_cloud_downsample->points.size()<<" target_merge_cloud_downsample points size:"<<target_merge_cloud_downsample->points.size()<<std::endl;
        return transformationMatrix_ICP;
    }
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> gicp;
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZINormal>);
    tree1->setInputCloud(source_merge_cloud_downsample);
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZINormal>);
    tree2->setInputCloud(target_merge_cloud_downsample);

    gicp.setSearchMethodSource(tree1);
    gicp.setSearchMethodTarget(tree2);
    //gicp.reset(new GICPType);
    gicp.setMaxCorrespondenceDistance(5);

    gicp.setMaximumIterations(100);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setRANSACIterations(1);

    //float scale_value = 10
    Eigen::Matrix4d gh_icp_result = transform_teaser_method(source_merge_cloud_downsample,target_merge_cloud_downsample,scale_value);
    //gh_icp_result(0,3) = 0;
    //gh_icp_result(1,3) = 0;
    //gh_icp_result(2,3) = 0;
    //========test start=======
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_down_display_d(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_down_display_d2(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::transformPointCloud (*source_merge_cloud_downsample, *cloud_source_down_display_d, gh_icp_result);


    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZINormal>);
    //gicp.setInputSource(source_merge_cloud_downsample);  //源点云
    gicp.setInputSource(cloud_source_down_display_d);  //源点云
    gicp.setInputTarget(target_merge_cloud_downsample);  //目标点云
    gicp.align (*cloud_icp);
    
    
    
    //========test end=======
    //Transform out;
    if (gicp.hasConverged ())
    {
        //printf ("\033[11A");  // Go up 11 lines in terminal output.
        std::cout << "\nGICP has converged, score is: " << gicp.getFitnessScore() << std::endl;
        transformationMatrix_ICP = gicp.getFinalTransformation ().cast<double>();
        std::cout<<"gicp result:"<<std::endl<<transformationMatrix_ICP<<std::endl;
        EigenPoseTeaser s2_target;
        s2_target.R_ = transformationMatrix_ICP.block<3,3>(0,0);
        s2_target.t_ = transformationMatrix_ICP.block<3,1>(0,3);
        
        EigenPoseTeaser s1_s2_ep;
        s1_s2_ep.R_ = gh_icp_result.block<3,3>(0,0);
        s1_s2_ep.t_ = gh_icp_result.block<3,1>(0,3);

        EigenPoseTeaser s1_target = s2_target*s1_s2_ep;
        transformationMatrix_ICP.block<3,3>(0,0) = s1_target.R_;
        transformationMatrix_ICP.block<3,1>(0,3) = s1_target.t_;

        //transformationMatrix_ICP = gh_icp_result;
        //transformationMatrix_ICP = gh_icp_result;
        pcl::transformPointCloud (*source_merge_cloud_downsample, *cloud_source_down_display_d, transformationMatrix_ICP);
        
        
        pcl::io::savePCDFileBinary("/home/kilox/wutong/loop_detect/test2_transform.pcd", *cloud_source_down_display_d);
        pcl::io::savePCDFileBinary("/home/kilox/wutong/loop_detect/test2_source.pcd", *source_merge_cloud_downsample);
        pcl::io::savePCDFileBinary("/home/kilox/wutong/loop_detect/test2_target.pcd", *target_merge_cloud_downsample);
        

        std::cout<<"transformationMatrix_ICP:"<<transformationMatrix_ICP<<std::endl;
        Eigen::Matrix3d rotate_m;
        Eigen::Vector3d ICP_t;
        rotate_m(0,0) = transformationMatrix_ICP(0,0);
        rotate_m(0,1) = transformationMatrix_ICP(0,1);
        rotate_m(0,2) = transformationMatrix_ICP(0,2);

        rotate_m(1,0) = transformationMatrix_ICP(1,0);
        rotate_m(1,1) = transformationMatrix_ICP(1,1);
        rotate_m(1,2)=  transformationMatrix_ICP(1,2);

        rotate_m(2,0) = transformationMatrix_ICP(2,0);
        rotate_m(2,1) = transformationMatrix_ICP(2,1);
        rotate_m(2,2) = transformationMatrix_ICP(2,2);

        ICP_t[0] = transformationMatrix_ICP(0,3);
        ICP_t[1] = transformationMatrix_ICP(1,3);
        ICP_t[2] = transformationMatrix_ICP(2,3);
        Eigen::Quaterniond q_rotate(rotate_m);

    }

    return transformationMatrix_ICP;
}
int main2() {
  // Load the .ply file
  teaser::PLYReader reader;
  teaser::PointCloud src_cloud;
  teaser::PointCloud tgt_cloud2;
  auto status = reader.read("/home/kilox/test_source.pcd", src_cloud);
  auto statu2s = reader.read("/home/kilox/test_target.pcd", tgt_cloud2);
  
  int N = src_cloud.size();
  int N_target = tgt_cloud2.size();
  // Convert the point cloud to Eigen
  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);

  Eigen::Matrix<double, 3, Eigen::Dynamic> src_rgt(3, N_target);
  for (size_t i = 0; i < N_target; ++i) {
    src_rgt.col(i) << tgt_cloud2[i].x, tgt_cloud2[i].y, tgt_cloud2[i].z;
  }

  for (size_t i = 0; i < N; ++i) {
    src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
  }

  // Homogeneous coordinates
  Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
  src_h.resize(4, src.cols());
  src_h.topRows(3) = src;
  src_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);


  Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h;
  tgt_h.resize(4, src_rgt.cols());
  tgt_h.topRows(3) = src_rgt;
  tgt_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N_target);

  // Apply an arbitrary SE(3) transformation
  Eigen::Matrix4d T;
  // clang-format off
  T << 9.96926560e-01,  6.68735757e-02, -4.06664421e-02, -1.15576939e-01,
      -6.61289946e-02, 9.97617877e-01,  1.94008687e-02, -3.87705398e-02,
      4.18675510e-02, -1.66517807e-02,  9.98977765e-01, 1.14874890,
      0,              0,                0,              1;
  // clang-format on
  // Apply transformation
  //Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h = src_h_tgt;
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = tgt_h.topRows(3);

  // Add some noise & outliers
  //addNoiseAndOutliers(tgt);

  // Convert to teaser point cloud
  teaser::PointCloud tgt_cloud;
  for (size_t i = 0; i < tgt.cols(); ++i) {
    tgt_cloud.push_back({static_cast<float>(tgt(0, i)), static_cast<float>(tgt(1, i)),
                         static_cast<float>(tgt(2, i))});
  }

  // Compute FPFH
  teaser::FPFHEstimation fpfh;
  float scale_value = 10;
  auto obj_descriptors = fpfh.computeFPFHFeatures(src_cloud, 0.02*scale_value, 0.04*scale_value);
  auto scene_descriptors = fpfh.computeFPFHFeatures(tgt_cloud, 0.02*scale_value, 0.04*scale_value);

  teaser::Matcher matcher;
  auto correspondences = matcher.calculateCorrespondences(
      src_cloud, tgt_cloud, *obj_descriptors, *scene_descriptors, false, true, false, 0.95);

  // Run TEASER++ registration
  // Prepare solver parameters
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = NOISE_BOUND;
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 100;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm =
      teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = 0.005;

  // Solve with TEASER++
  teaser::RobustRegistrationSolver solver(params);
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  solver.solve(src_cloud, tgt_cloud, correspondences);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  auto solution = solver.getSolution();

  Eigen::Vector3d t_so = solution.translation;
  Eigen::Matrix3d R_so = solution.rotation;
  Eigen::Matrix4d transformationMatrix_ICP = Eigen::Matrix4d::Identity();
  transformationMatrix_ICP.block<3,3>(0,0) = R_so;
  transformationMatrix_ICP.block<3,1>(0,3) = t_so;

  
  pcl::PCDReader reader_pcd;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_read_source(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_read_source_transform(new pcl::PointCloud<pcl::PointXYZ>());
  reader_pcd.read<pcl::PointXYZ>("/home/kilox/test_source.pcd", *cloud_read_source);
  pcl::transformPointCloud(*cloud_read_source, *cloud_read_source_transform, transformationMatrix_ICP);
  pcl::io::savePCDFileBinary("/home/kilox/test_source_transform.pcd", *cloud_read_source_transform);

  // Compare results
  std::cout << "=====================================" << std::endl;
  std::cout << "          TEASER++ Results           " << std::endl;
  std::cout << "=====================================" << std::endl;
  std::cout << "Expected rotation: " << std::endl;
  std::cout << T.topLeftCorner(3, 3) << std::endl;
  std::cout << "Estimated rotation: " << std::endl;
  std::cout << solution.rotation << std::endl;
  std::cout << "Error (rad): " << getAngularError(T.topLeftCorner(3, 3), solution.rotation)
            << std::endl;
  std::cout << std::endl;
  std::cout << "Expected translation: " << std::endl;
  std::cout << T.topRightCorner(3, 1) << std::endl;
  std::cout << "Estimated translation: " << std::endl;
  std::cout << solution.translation << std::endl;
  std::cout << "Error (m): " << (T.topRightCorner(3, 1) - solution.translation).norm() << std::endl;
  std::cout << std::endl;
  std::cout << "Number of correspondences: " << N << std::endl;
  std::cout << "Number of outliers: " << N_OUTLIERS << std::endl;
  std::cout << "=====> Time taken (s): "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() /
                   1000000.0
            << std::endl;
}
