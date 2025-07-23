// An example showing TEASER++ registration with the Stanford bunny model
#include <chrono>
#include <iostream>
#include <random>

#include <Eigen/Core>

#include <teaser/ply_io.h>
#include <teaser/registration.h>

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
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
int main() {
  // Load the .ply file
  teaser::PLYReader reader;
  teaser::PointCloud src_cloud;
  teaser::PointCloud tgt_cloud;
  std::cout<<"line:"<<__LINE__<<std::endl;
  //auto status = reader.read("/home/kilox/workspace/workspace7/convertpointcloud/TEASER-plusplus/examples/example_data/bun_zipper_res3.ply", src_cloud);
  //auto status2 = reader.read("/home/kilox/workspace/workspace7/convertpointcloud/TEASER-plusplus/examples/example_data/bun_zipper_res3.ply", src_cloud);
  
  pcl::PCDReader reader2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>());
  reader2.read<pcl::PointXYZ>("/home/kilox/test_source.pcd", *cloud_source);
  reader2.read<pcl::PointXYZ>("/home/kilox/test_target.pcd", *cloud_target);

  std::cout<<"line:"<<__LINE__<<std::endl;
  int N = cloud_source->points.size();
  int N_target = cloud_target->points.size();
  std::cout<<"line:"<<__LINE__<<" N:"<<N<<std::endl;
  // Convert the point cloud to Eigen
  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_t(3, N_target);
  for (size_t i = 0; i < N; ++i) {
    src.col(i) << cloud_source->points[i].x, cloud_source->points[i].y, cloud_source->points[i].z;
  }

  for (size_t i = 0; i < N_target; ++i) {
    tgt_t.col(i) << cloud_target->points[i].x, cloud_target->points[i].y, cloud_target->points[i].z;
  }

  std::cout<<"cloud_target->points size:"<<cloud_target->points.size()<<std::endl;
  std::cout<<"cloud_source->points size:"<<cloud_source->points.size()<<std::endl;
  // Homogeneous coordinates
  Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;

  std::cout<<"line:"<<__LINE__<<std::endl;
  src_h.resize(4, src.cols());
  src_h.topRows(3) = src;
  src_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);
  std::cout<<"line:"<<__LINE__<<std::endl;

  Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h;
  std::cout<<"line:"<<__LINE__<<std::endl;
  tgt_h.resize(4, tgt_t.cols());
  tgt_h.topRows(3) = tgt_t;
  tgt_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N_target);

  std::cout<<"line:"<<__LINE__<<std::endl;
  // Apply an arbitrary SE(3) transformation
  Eigen::Matrix4d T;
  // clang-format off
  T << 9.96926560e-01,  6.68735757e-02, -4.06664421e-02, -1.15576939e-01,
      -6.61289946e-02, 9.97617877e-01,  1.94008687e-02, -3.87705398e-02,
      4.18675510e-02, -1.66517807e-02,  9.98977765e-01, 1.14874890e-01,
      0,              0,                0,              1;
  // clang-format on

  std::cout<<"line:"<<__LINE__<<std::endl;
  // Apply transformation
  //Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h = T * src_h;
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = tgt_h.topRows(3);

  // Add some noise & outliers
  //addNoiseAndOutliers(tgt);

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

  std::cout<<"line:"<<__LINE__<<std::endl;
  // Solve with TEASER++
  teaser::RobustRegistrationSolver solver(params);
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  solver.solve(src, tgt);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  std::cout<<"line:"<<__LINE__<<std::endl;
  auto solution = solver.getSolution();

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
  std::cout << "Time taken (s): "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() /
                   1000000.0
            << std::endl;
}
