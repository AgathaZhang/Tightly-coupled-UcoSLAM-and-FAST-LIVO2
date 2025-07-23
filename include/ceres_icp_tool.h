//
// Created by kilox on 2022/8/18.
//

#include <iostream>
#include <pcl/registration/ndt.h>
#include <stdio.h>
#include <vector>
#include <pcl/registration/icp.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

#include <ceres/ceres.h>

#include "ceres_icp.hpp"
#ifndef CATKIN_R3LIVE_CERES_ICP_TOOL_H
#define CATKIN_R3LIVE_CERES_ICP_TOOL_H

#define CORNER_MIN_MAP_NUM 0
#define SURFACE_MIN_MAP_NUM 50
#define BLUR_SCALE 1.0
#include <random>
namespace kilox{

    template < typename T >
    struct Random_generator_float
    {
        std::random_device                 random_device;
        std::mt19937                       m_random_engine;
        std::uniform_real_distribution< T > m_dist;
        std::normal_distribution< T > m_dist_normal;

        Random_generator_float(): m_random_engine( std::random_device{}() )
        {};
        ~Random_generator_float(){};

        T rand_uniform( T low = 0.0, T hight = 1.0 )
        {
            m_dist = std::uniform_real_distribution< T >( low, hight );
            return m_dist( m_random_engine );
        }

        T rand_normal( T mean = 0.0, T std = 100.0 )
        {
            m_dist_normal = std::normal_distribution< T >( mean, std );
            return m_dist_normal( m_random_engine );
        }

        T *rand_array_uniform( T low = 0.0, T hight = 1.0, size_t numbers= 100 )
        {
            T *res = new T[ numbers ];
            m_dist = std::uniform_real_distribution< T >( low, hight );
            for ( size_t i = 0; i < numbers; i++ )
            {
                res[ i ] = m_dist( m_random_engine );
            }
            return res;
        }
    };

    typedef pcl::PointXYZINormal PointType;
    typedef pcl::PointCloud<PointType> PointCloudXYZINormal;

   
class CeresIcpTool{

    public:
        double m_line_res;
        double m_plane_res;
        CeresIcpTool(double m_line_res_, double m_plane_res_){
            m_line_res = m_line_res_;
            m_plane_res = m_plane_res_;
            m_down_sample_filter_line_source.setLeafSize( m_line_res, m_line_res, m_line_res );
            m_down_sample_filter_surface_source.setLeafSize( m_plane_res, m_plane_res, m_plane_res );
            m_down_sample_filter_line_target.setLeafSize( m_line_res, m_line_res, m_line_res );
            m_down_sample_filter_surface_target.setLeafSize( m_plane_res, m_plane_res, m_plane_res );
        }

        CeresIcpTool(){
            m_line_res = 0.4;
            m_plane_res = 0.4;
            m_down_sample_filter_line_source.setLeafSize( m_line_res, m_line_res, m_line_res );
            m_down_sample_filter_surface_source.setLeafSize( m_plane_res, m_plane_res, m_plane_res );
            m_down_sample_filter_line_target.setLeafSize( m_line_res, m_line_res, m_line_res );
            m_down_sample_filter_surface_target.setLeafSize( m_plane_res, m_plane_res, m_plane_res );
        }

    public:

        pcl::PointCloud< PointType > sourece_pt_line_ds, sourece_pt_plane_ds; // Point cloud of downsampled
        pcl::PointCloud< PointType > target_pt_line_ds, target_pt_plane_ds;
        Eigen::Map<Eigen::Quaterniond> m_q_w_incre = Eigen::Map<Eigen::Quaterniond>( m_para_buffer_incremental );
        Eigen::Map<Eigen::Vector3d>    m_t_w_incre = Eigen::Map<Eigen::Vector3d>( m_para_buffer_incremental + 4 );

        pcl::KdTreeFLANN<PointType> m_kdtree_surf_from_map;
        pcl::KdTreeFLANN<PointType> m_kdtree_surf_from_map_last;

        ceres::Solver::Summary               summary;
        ceres::Solver::Summary               m_final_opt_summary;

        Random_generator_float<float> m_rand_float;

        bool gicp_has_conv = false;
        int line_search_num = 5;
        int IF_LINE_FEATURE_CHECK = 0;
        int plane_search_num = 5;
        int IF_PLANE_FEATURE_CHECK = 0;
        int ICP_PLANE = 1;
        int ICP_LINE= 1;

        double m_para_buffer_RT[ 7 ] = { 0, 0, 0, 1, 0, 0, 0 };
        double m_para_buffer_RT_last[ 7 ] = { 0, 0, 0, 1, 0, 0, 0 };
        double m_para_buffer_incremental[ 7 ] = { 0, 0, 0, 1, 0, 0, 0 };

        int m_current_frame_index = 0;
        int m_para_min_match_blur = 0.0;
        int m_para_max_match_blur = 0.3;
        int m_max_buffer_size = 50000000;

        int m_mapping_init_accumulate_frames = 100;
        int m_kmean_filter_count = 3;
        int m_kmean_filter_threshold = 2.0;

        double m_time_pc_corner_past = 0;
        double m_time_pc_surface_past = 0;
        double m_time_pc_full = 0;
        double m_time_odom = 0;
        double m_last_time_stamp = 0;
        double m_minimum_pt_time_stamp = 0;
        double m_maximum_pt_time_stamp = 1.0;
        float  m_last_max_blur = 0.0;

        int    m_odom_mode;
        int    m_matching_mode = 0;
        int    m_if_input_downsample_mode = 1;
        int    m_maximum_parallel_thread;
        int    m_maximum_mapping_buff_thread = 1; // Maximum number of thead for matching buffer update
        int    m_maximum_history_size = 100;
        int    m_para_threshold_cell_revisit = 0;
        float  m_para_max_angular_rate = 200.0 / 50.0; // max angular rate = 90.0 /50.0 deg/s
        float  m_max_final_cost = 100.0;
        int    m_para_icp_max_iterations = 20;
        int    m_para_cere_max_iterations = 100;
        int    m_para_optimization_maximum_residual_block = 1e5;
        double m_minimum_icp_R_diff = 0.01;
        double m_minimum_icp_T_diff = 0.01;

        int                                  m_maximum_allow_residual_block = 1e5;

        int                    surf_avail_num = 0;
        int                    corner_avail_num = 0;
        float                  minimize_cost = summary.final_cost ;
        PointType              pointOri, pointSel;
        int                    corner_rejection_num = 0;
        int                    surface_rejecetion_num = 0;
        int                    if_undistore_in_matching = 1;

        double                  m_angular_diff = 0;
        double                  m_t_diff = 0;
        double                  m_maximum_dis_plane_for_match = 50.0;
        double                  m_maximum_dis_line_for_match = 2.0;
        Eigen::Matrix<double, 3, 1> m_interpolatation_omega;
        Eigen::Matrix<double, 3, 3> m_interpolatation_omega_hat;
        Eigen::Matrix<double, 3, 3> m_interpolatation_omega_hat_sq2;

        int m_if_motion_deblur = 0 ;

        ceres::LinearSolverType slover_type = ceres::DENSE_SCHUR;
        int    m_para_cere_prerun_times = 2;
        float  m_para_max_speed = 100.0 / 50.0;

        double m_inliner_dis = 0.02;
        double m_inlier_ratio = 0.80;

        double m_interpolatation_theta = 0;

        double m_inlier_threshold;

        pcl::VoxelGrid< PointType > m_down_sample_filter_line_source, m_down_sample_filter_line_target;
        pcl::VoxelGrid< PointType > m_down_sample_filter_surface_source, m_down_sample_filter_surface_target;

        //当前的source点到世界的位姿
        Eigen::Quaterniond m_q_w_curr, m_q_w_last;
        Eigen::Vector3d m_t_w_curr, m_t_w_last;

        int                         m_maximum_icp_iteration = 10;
        float                       m_accepted_threshold = 0.2;

        int                         m_para_scene_alignments_maximum_residual_block = 5000;

    public:

        double compute_inlier_residual_threshold( std::vector< double > residuals, double ratio )
        {
            std::set< double > dis_vec;
            for ( size_t i = 0; i < ( size_t )( residuals.size() / 3 ); i++ )
            {
                dis_vec.insert( fabs( residuals[ 3 * i + 0 ] ) + fabs( residuals[ 3 * i + 1 ] ) + fabs( residuals[ 3 * i + 2 ] ) );
            }
            return *( std::next( dis_vec.begin(), ( int ) ( ( ratio ) * dis_vec.size() ) ) );
        }

        void set_ceres_solver_bound2( ceres::Problem &problem ,double * para_buffer_RT )
        {
            for ( unsigned int i = 0; i < 3; i++ )
            {

                problem.SetParameterLowerBound( para_buffer_RT + 4, i, -m_para_max_speed );
                problem.SetParameterUpperBound( para_buffer_RT + 4, i, +m_para_max_speed );
            }
        };

        Eigen::Matrix<double, 3, 1> pcl_pt_to_eigend( PointType &pt )
        {
            return Eigen::Matrix<double, 3, 1>( pt.x, pt.y, pt.z );
        }

    void pointAssociateToMap( PointType const *const pi, PointType *const po,
                              double interpolate_s = 1.0, int if_undistore = 0 )
    {
        Eigen::Vector3d point_curr( pi->x, pi->y, pi->z );
        Eigen::Vector3d point_w;

        point_w = m_q_w_curr * point_curr + m_t_w_curr;

        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
        //po->intensity = 1.0;
    }

        double find_tranfrom_of_two_mappings( pcl::PointCloud< PointType >& all_pt_a,
                                           pcl::PointCloud< PointType >& all_pt_b,
                                           int                               if_save = 1,
                                           std::string                       mapping_save_path = std::string( " " ) )
        {

            m_q_w_curr.setIdentity();
            m_q_w_last.setIdentity();
            m_t_w_last.setZero();
            m_para_icp_max_iterations = m_maximum_icp_iteration;
            m_para_cere_max_iterations = 10;
            m_para_cere_prerun_times = 2;
            m_maximum_allow_residual_block = m_para_scene_alignments_maximum_residual_block;

            //初始化参数

            m_max_final_cost = 20000;
            m_para_max_speed = 1000.0;
            m_para_max_angular_rate = 360 * 57.3;
            m_para_icp_max_iterations = 10;
            m_para_cere_max_iterations = 10;
            m_inliner_dis = 0.2;
            //pcl::PointCloud< PointType > sourece_pt_line = keyframe_a->extract_specify_points( Feature_type::e_feature_line );
            //pcl::PointCloud< PointType > sourece_pt_plane = keyframe_a->extract_specify_points(Feature_type::e_feature_plane );

            //pcl::PointCloud< PointType > all_pt_a = keyframe_a->get_all_pointcloud();
            //pcl::PointCloud< PointType > all_pt_a = keyframe_a->get_all_pointcloud();
            //pcl::PointCloud< PointType > all_pt_a = keyframe_a->m_accumulated_point_cloud;
            pcl::PointCloud< PointType > sourece_pt_line = all_pt_a;
            pcl::PointCloud< PointType > sourece_pt_plane = all_pt_a;

            //pcl::PointCloud< PointType > target_pt_line = keyframe_b->extract_specify_points(Feature_type::e_feature_line );
            //pcl::PointCloud< PointType > target_pt_plane = keyframe_b->extract_specify_points( Feature_type::e_feature_plane );

            //pcl::PointCloud< PointType > all_pt_b = keyframe_b->m_accumulated_point_cloud;
            //pcl::PointCloud< PointType > all_pt_b = keyframe_b->get_all_pointcloud();
            pcl::PointCloud< PointType > target_pt_line = all_pt_b;
            pcl::PointCloud< PointType > target_pt_plane = all_pt_b;



            

            //std::cout<<"a:"<<all_pt_a.size()<<" b:"<< all_pt_b.size()<<std::endl;

            m_down_sample_filter_line_source.setInputCloud( sourece_pt_line.makeShared() );
            m_down_sample_filter_surface_source.setInputCloud( sourece_pt_plane.makeShared() );
            m_down_sample_filter_line_target.setInputCloud( target_pt_line.makeShared() );
            m_down_sample_filter_surface_target.setInputCloud( target_pt_plane.makeShared() );

            
            // 推送过滤后的源点和目标点的点云

            Eigen::Matrix< double, 3, 1 > transform_T;
            Eigen::Quaterniond            transform_R = Eigen::Quaterniond::Identity();
            transform_T(0,0) = 0;
            transform_T(1,0) = 0;
            transform_T(2,0) = 0;
            m_t_w_incre = transform_T;
            m_t_w_curr = transform_T;

            gicp_has_conv = false;
            //std::cout<<"transform_T:"<< transform_T <<" transform_R:"<<transform_R<<std::endl;
            //Common_tools::Timer timer;
            //timer.tic("Total omp");
            //for ( int scale = 8; scale >= 0; scale-=4 )
            //从粗到细地进行icp
            for ( int scale = 8; scale >= 0; scale-=4 )
            {

                //timer.tic("Each omp");


                //m_line_res 0.4 plane_res 0.4
                float line_res = m_line_res*scale;
                float plane_res = m_plane_res*scale ;
                if ( line_res < m_line_res )
                {
                    line_res = m_line_res;
                }

                if ( plane_res < m_plane_res )
                {
                    plane_res = m_plane_res;
                    m_para_icp_max_iterations = m_maximum_icp_iteration*2;
                }

                //m_down_sample_filter_line_source.setLeafSize( line_res, line_res, line_res );
                m_down_sample_filter_surface_source.setLeafSize( plane_res, plane_res, plane_res );
                //m_down_sample_filter_line_target.setLeafSize( line_res, line_res, line_res );
                m_down_sample_filter_surface_target.setLeafSize( plane_res, plane_res, plane_res );

                //m_down_sample_filter_line_source.filter( sourece_pt_line_ds );
                m_down_sample_filter_surface_source.filter( sourece_pt_plane_ds );

                //m_down_sample_filter_line_target.filter( target_pt_line_ds );
                m_down_sample_filter_surface_target.filter( target_pt_plane_ds );

                gicp_has_conv = true;

                /**
                std::cout<< "Source pt line size = " << sourece_pt_line_ds.size() << " , plane size = " << sourece_pt_plane_ds.size()<<""<<std::endl;
                std::cout << "Target pt line size = " << target_pt_line_ds.size() << " , plane size = " << target_pt_plane_ds.size() << std::endl;

                if(scale == 8){
                    std::cout<<"-----------------------------------------------------------"<<std::endl;
                    std::cout<<"-------------> before transform -------------------------"<<std::endl;
                    std::cout<<"q:"<<m_q_w_curr.coeffs().transpose()<<std::endl;
                    std::cout<<"t:"<<m_t_w_curr.transpose()<<std::endl;
                }
                **/
                //todo: 地图点，原始点 这里进行了转换，如果不行得换回来
                //std::cout<<"===> ceres line:"<<__LINE__<<std::endl;
                find_out_incremental_transfrom(  target_pt_plane_ds.makeShared(), sourece_pt_plane_ds.makeShared() );
                //std::cout<<"===> ceres line:"<<__LINE__<<std::endl;
                /**
                if(scale == 0){
                    std::cout<<"------------->after transform -----------------------------"<<std::endl;
                    std::cout<<"q:"<<m_q_w_curr.coeffs().transpose()<<std::endl;
                    std::cout<<"t:"<<m_t_w_curr.transpose()<<std::endl;
                    std::cout<<"-----------------------------------------------------------"<<std::endl;
                }
                **/
                if(m_inlier_threshold > m_accepted_threshold*2)
                    break;

            }
            
            //std::cout<<"=============> total cost time:"<<std::endl<< timer.toc_string( "Total omp" ) << std::endl;


            target_pt_line_ds.clear();
            sourece_pt_plane_ds.clear();
            target_pt_line_ds.clear();
            target_pt_plane_ds.clear();

            return m_inlier_threshold;
        };

        int find_out_incremental_transfrom(
                                            pcl::PointCloud<PointType>::Ptr in_laser_cloud_surf_from_map,
                                            pcl::PointCloud<PointType>::Ptr laserCloudSurfStack)
        {


            pcl::PointCloud<PointType> laser_cloud_surf_from_map =  *in_laser_cloud_surf_from_map;

            if(laser_cloud_surf_from_map.points.size())
            {
                m_kdtree_surf_from_map.setInputCloud( laser_cloud_surf_from_map.makeShared() );
            }
            else
            {
                return 1;
            }
            return find_out_incremental_transfrom(  in_laser_cloud_surf_from_map, m_kdtree_surf_from_map, laserCloudSurfStack );

        }



        int find_out_incremental_transfrom(
                                            pcl::PointCloud< PointType >::Ptr& in_laser_cloud_surf_from_map,
                                            pcl::KdTreeFLANN< PointType > &   kdtree_surf_from_map,
                                            pcl::PointCloud< PointType >::Ptr& laserCloudSurfStack )
        {

            //std::cout<<"line:"<<__LINE__<<std::endl;
            Eigen::Map<Eigen::Quaterniond> q_w_incre = Eigen::Map<Eigen::Quaterniond>(m_para_buffer_incremental );
            Eigen::Map<Eigen::Vector3d> t_w_incre = Eigen::Map<Eigen::Vector3d>( m_para_buffer_incremental + 4 );

            m_kdtree_surf_from_map = kdtree_surf_from_map;

            //pcl::PointCloud<PointType> laser_cloud_surf_from_map =  *in_laser_cloud_surf_from_map;

            int laserCloudSurfFromMapNum = in_laser_cloud_surf_from_map->points.size();
            int laser_surface_pt_num = laserCloudSurfStack->points.size();


            int                    surf_avail_num = 0;
            float                  minimize_cost = summary.final_cost ;
            PointType              pointOri, pointSel;
            int                    surface_rejecetion_num = 0;
            int                    if_undistore_in_matching = 1;
            //printf_line;

            //std::cout<<"line:"<<__LINE__<<" laserCloudSurfFromMapNum:"<<laserCloudSurfFromMapNum<<" laser_surface_pt_num:"<<laser_surface_pt_num<<std::endl;
            if ( laserCloudSurfFromMapNum > SURFACE_MIN_MAP_NUM )
            {
                Eigen::Quaterniond q_last_optimize( 1.f, 0.f, 0.f, 0.f );
                Eigen::Vector3d    t_last_optimize( 0.f, 0.f, 0.f );
                int                iterCount = 0;

                std::vector<int>   m_point_search_Idx;
                std::vector<float> m_point_search_sq_dis;
                //std::cout<<"line:"<<__LINE__<<std::endl;
                for ( iterCount = 0; iterCount < m_para_icp_max_iterations; iterCount++ )
                {
                    m_point_search_Idx.clear();
                    m_point_search_sq_dis.clear();
                    surf_avail_num = 0;
                    surface_rejecetion_num = 0;

                    ceres::LossFunction *               loss_function = new ceres::HuberLoss( 0.1 );
                    ceres::LocalParameterization *      q_parameterization = new ceres::EigenQuaternionParameterization();
                    ceres::Problem::Options             problem_options;
                    ceres::ResidualBlockId              block_id;
                    ceres::Problem                      problem( problem_options );
                    std::vector<ceres::ResidualBlockId> residual_block_ids;

                    problem.AddParameterBlock( m_para_buffer_incremental, 4, q_parameterization );
                    problem.AddParameterBlock( m_para_buffer_incremental + 4, 3 );

                    for ( int i = 0; i < laser_surface_pt_num; i++ )
                    {

                        //std::cout<<"line:"<<__LINE__<<std::endl;
                        if ( laser_surface_pt_num > 2 * m_maximum_allow_residual_block )
                        {
                            if(m_rand_float.rand_uniform() * laser_surface_pt_num >  2 * m_maximum_allow_residual_block)
                            {
                                continue;
                            }
                        }

                        //std::cout<<"line:"<<__LINE__<<std::endl;
                        pointOri = laserCloudSurfStack->points[ i ];
                        int planeValid = true;
                        pointAssociateToMap( &pointOri, &pointSel, 1.0, if_undistore_in_matching );
                        //printf_line;
                        m_kdtree_surf_from_map.nearestKSearch( pointSel, plane_search_num, m_point_search_Idx, m_point_search_sq_dis );

                        //std::cout<<"line:"<<__LINE__<<std::endl;
                        if ( m_point_search_sq_dis[ plane_search_num - 1 ] < m_maximum_dis_plane_for_match )
                        {
                            std::vector<Eigen::Vector3d> nearCorners;
                            Eigen::Vector3d              center( 0, 0, 0 );
                            //std::cout<<"line:"<<__LINE__<<std::endl;
                            if ( IF_PLANE_FEATURE_CHECK )
                            {
                                for ( int j = 0; j < plane_search_num; j++ )
                                {
                                    //todo 这里可能会有问题 原来是laser_cloud_corner_from_map
                                    Eigen::Vector3d tmp( in_laser_cloud_surf_from_map->points[ m_point_search_Idx[ j ] ].x,
                                                         in_laser_cloud_surf_from_map->points[ m_point_search_Idx[ j ] ].y,
                                                         in_laser_cloud_surf_from_map->points[ m_point_search_Idx[ j ] ].z );
                                    center = center + tmp;
                                    nearCorners.push_back( tmp );
                                }

                                center = center / ( float ) ( plane_search_num );

                                Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();

                                for ( int j = 0; j < plane_search_num; j++ )
                                {
                                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[ j ] - center;
                                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                                }

                                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes( covMat );

                                if ( ( saes.eigenvalues()[ 2 ] > 3 * saes.eigenvalues()[ 0 ] ) &&
                                     ( saes.eigenvalues()[ 2 ] < 10 * saes.eigenvalues()[ 1 ] ) )
                                {
                                    planeValid = true;
                                }
                                else
                                {
                                    planeValid = false;
                                }
                            }

                            Eigen::Vector3d curr_point( pointOri.x, pointOri.y, pointOri.z );
                            //std::cout<<"line:"<<__LINE__<<std::endl;
                            if ( planeValid )
                            {
                                 //std::cout<<"line:"<<__LINE__<<std::endl;
                                if ( ICP_PLANE )
                                {
                                     //std::cout<<"line:"<<__LINE__<<std::endl;
                                    ceres::CostFunction *cost_function;
                                     //std::cout<<"line:"<<__LINE__<<std::endl;
                                    m_if_motion_deblur = false;
                                   // std::cout<<"m_point_search_Idx[ 0 ]:"<<m_point_search_Idx[ 0 ]<<std::endl;
                                    //std::cout<<"plane_search_num / 2:"<<plane_search_num / 2<<std::endl;
                                    //std::cout<<"plane_search_num - 1:"<<plane_search_num - 1<<std::endl;
                                    //std::cout<<"m_point_search_Idx[ plane_search_num / 2 ]:"<<m_point_search_Idx[ plane_search_num / 2 ]<<std::endl;
                                    //std::cout<<"m_point_search_Idx[ plane_search_num - 1 ]:"<<m_point_search_Idx[ plane_search_num - 1 ]<<std::endl;
                                    //std::cout<<"laser_cloud_surf_from_map.points size:"<<in_laser_cloud_surf_from_map->points.size()<<std::endl;
                                    cost_function = ceres_icp_point2plane<double>::Create(
                                                curr_point,
                                                pcl_pt_to_eigend( in_laser_cloud_surf_from_map->points[ m_point_search_Idx[ 0 ] ] ),
                                                pcl_pt_to_eigend( in_laser_cloud_surf_from_map->points[ m_point_search_Idx[ plane_search_num / 2 ] ] ),
                                                pcl_pt_to_eigend( in_laser_cloud_surf_from_map->points[ m_point_search_Idx[ plane_search_num - 1 ] ] ),
                                                Eigen::Matrix<double, 4, 1>( m_q_w_last.w(), m_q_w_last.x(), m_q_w_last.y(), m_q_w_last.z() ),
                                                m_t_w_last ); //pointOri.intensity );
                                     //std::cout<<"line:"<<__LINE__<<std::endl;
                                    block_id = problem.AddResidualBlock( cost_function, loss_function, m_para_buffer_incremental, m_para_buffer_incremental + 4 );
                                     //std::cout<<"line:"<<__LINE__<<std::endl;
                                    residual_block_ids.push_back( block_id );
                                     //std::cout<<"line:"<<__LINE__<<std::endl;
                                }
                                surf_avail_num++;
                                 //std::cout<<"line:"<<__LINE__<<std::endl;
                            }
                            else
                            {
                                surface_rejecetion_num++;
                            }
                            //std::cout<<"line:"<<__LINE__<<std::endl;
                        }
                    }

                    std::vector< ceres::ResidualBlockId > residual_block_ids_temp;
                    residual_block_ids_temp.reserve( residual_block_ids.size() );
                    //std::cout<<"line:"<<__LINE__<<std::endl;
                    // Drop some of the residual to guaruntee the real time performance.
                    if ( residual_block_ids.size() > (size_t) m_maximum_allow_residual_block )
                    {
                        residual_block_ids_temp.clear();

                        float  threshold_to_reserve = ( float ) m_maximum_allow_residual_block / ( float ) residual_block_ids.size();
                        float *probability_to_drop = m_rand_float.rand_array_uniform( 0, 1.0, residual_block_ids.size() );
                        for ( size_t i = 0; i < residual_block_ids.size(); i++ )
                        {
                            if ( probability_to_drop[ i ] > threshold_to_reserve )
                            {
                                problem.RemoveResidualBlock( residual_block_ids[ i ] );
                            }
                            else
                            {
                                residual_block_ids_temp.push_back( residual_block_ids[ i ] );
                            }
                        }
                        residual_block_ids = residual_block_ids_temp;
                        delete probability_to_drop;
                    }

                    ceres::Solver::Options options;
                    //std::cout<<"line:"<<__LINE__<<std::endl;
                    // If the number of residual block too Large, randomly drop some of them to guarentee the real-time perfromance.
                    for ( size_t ii = 0; ii < 1; ii++ )
                    {
                        options.linear_solver_type = slover_type;
                        options.max_num_iterations = m_para_cere_max_iterations;
                        options.max_num_iterations = m_para_cere_prerun_times;
                        options.minimizer_progress_to_stdout = false;
                        options.check_gradients = false;
                        options.gradient_check_relative_precision = 1e-10;
                        //options.function_tolerance = 1e-100; // default 1e-6

                        set_ceres_solver_bound2( problem, m_para_buffer_incremental );
                        ceres::Solve( options, &problem, &summary );

                        residual_block_ids_temp.clear();
                        ceres::Problem::EvaluateOptions eval_options;
                        eval_options.residual_blocks = residual_block_ids;
                        double              total_cost = 0.0;
                        std::vector<double> residuals;
                        problem.Evaluate( eval_options, &total_cost, &residuals, nullptr, nullptr );
                        //double avr_cost = total_cost / residual_block_ids.size();

                        double m_inliner_ratio_threshold = compute_inlier_residual_threshold( residuals, m_inlier_ratio );
                        m_inlier_threshold = std::max( m_inliner_dis, m_inliner_ratio_threshold );
                        //screen_out << "Inlier threshold is: " << m_inlier_final_threshold << endl;
                        for ( unsigned int i = 0; i < residual_block_ids.size(); i++ )
                        {
                            if ( ( fabs( residuals[ 3 * i + 0 ] ) + fabs( residuals[ 3 * i + 1 ] ) + fabs( residuals[ 3 * i + 2 ] ) ) > m_inlier_threshold ) // std::min( 1.0, 10 * avr_cost )
                            {
                                //screen_out << "Remove outliers, drop id = " << (void *)residual_block_ids[ i ] <<endl;
                                problem.RemoveResidualBlock( residual_block_ids[ i ] );
                            }
                            else
                            {
                                residual_block_ids_temp.push_back( residual_block_ids[ i ] );
                            }
                        }
                        residual_block_ids = residual_block_ids_temp;
                    }
                    //std::cout<<"line:"<<__LINE__<<std::endl;
                    options.linear_solver_type = slover_type;
                    options.max_num_iterations = m_para_cere_max_iterations;
                    options.minimizer_progress_to_stdout = false;
                    options.check_gradients = false;
                    options.gradient_check_relative_precision = 1e-10;

                    set_ceres_solver_bound2( problem, m_para_buffer_incremental );
                    ceres::Solve( options, &problem, &summary );

                    m_t_w_curr = m_q_w_last * t_w_incre + m_t_w_last;
                    m_q_w_curr = m_q_w_last * q_w_incre;

                    m_angular_diff = ( float ) m_q_w_curr.angularDistance( m_q_w_last ) * 57.3;
                    m_t_diff = ( m_t_w_curr - m_t_w_last ).norm();
                    minimize_cost = summary.final_cost  ;

                    if ( q_last_optimize.angularDistance( q_w_incre ) < 57.3 * m_minimum_icp_R_diff &&
                         ( t_last_optimize - t_w_incre ).norm() < m_minimum_icp_T_diff )
                    {
                        break;
                    }
                    else
                    {
                        q_last_optimize = q_w_incre;
                        t_last_optimize = t_w_incre;
                    }
                }

                //std::cout<<"line:"<<__LINE__<<std::endl;
                m_inlier_threshold = m_inlier_threshold* summary.final_cost/ summary.initial_cost; //
                 //std::cout<<"line:"<<__LINE__<<std::endl;
                if ( m_angular_diff > m_para_max_angular_rate || minimize_cost > m_max_final_cost )
                {

                    for ( int i = 0; i < 7; i++ )
                    {
                        m_para_buffer_RT[ i ] = m_para_buffer_RT_last[ i ];
                    }
                    m_last_time_stamp = m_minimum_pt_time_stamp;
                    m_q_w_curr = m_q_w_last;
                    m_t_w_curr = m_t_w_last;
                    //std::cout<<"line:"<<__LINE__<<std::endl;
                    return 0;
                }
                 //std::cout<<"line:"<<__LINE__<<std::endl;
                m_final_opt_summary = summary;
                 //std::cout<<"line:"<<__LINE__<<std::endl;
            }
            else
            {
                std::cout << "time Map corner and surf num are not enough" << std::endl;
            }
             //std::cout<<"line:"<<__LINE__<<std::endl;
            return 1;

        }




    };
}

#endif //CATKIN_R3LIVE_CERES_ICP_TOOL_H
