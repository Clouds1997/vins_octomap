// This file is part of dre_slam - Dynamic RGB-D Encoder SLAM for Differential-Drive Robot.
//
// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)
//
// dre_slam is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// dre_slam is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "sub_octomap_construction.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace dre_slam
{

// 这个节点是创建子图的节点
SubOctoMapConstruction::SubOctoMapConstruction ( OctoMapFusion* octomap_fusion,  Config* cfg ) : octomap_fusion_ ( octomap_fusion ), cfg_ ( cfg )
{
    sub_octomap_construction_thread_ = new std::thread ( &SubOctoMapConstruction::processing, this );
    nkf_passed_ = 0;
} // SubOctomapping


void SubOctoMapConstruction::processing()
{
    // std::cout << "SubOctoMapConstruction is ok !!!!" << std::endl;
    while ( true ) {
        // 这里就是将关键帧序列给全部弹完为止
        if ( checkNewFrame() == true ) {
            // std::cout << "map is constructing !!!!" << std::endl;
            // 从关键帧序列中弹出一个位于栈顶的关键帧
            KeyFrame* kf = getNewKeyFrame();

			/*std::unique_lock<mutex> lock ( mutex_all_data_ );*/ 
			
            // construct sub map.
            if ( nkf_passed_ == 0 ) {
                cur_sub_map_ = new SubOctomap ( cfg_ );
            }

            // Construct 3D point cloud in the camera frame.
            octomap::Pointcloud point_cloud_c;
            //createCleanCloud ( kf, point_cloud_c ); // voxel_grid filtered   这个就是将关键帧的图像转换为点云数据，这里的点云数据还是在相机坐标系下面
			createPointCloud( kf, point_cloud_c );

			// Insert one scan to full map.   这里是将相机坐标下面的点云转换到世界坐标系下面
			octomap_fusion_->insertOneScan2FullMapAndPub ( kf, point_cloud_c);
             
			// // Insert one scan to cur sub map.  就是将这一帧的点云转到base（第一帧点云）之下 就是差不多创建一个局部的小地图 
            // cur_sub_map_->insertKeyFrame ( kf, point_cloud_c );

            // // Check if need to construct new submap.  如果当前子图的关键帧数量已经达到阈值，则创建新的子图
<<<<<<< HEAD
            ++ nkf_passed_;
            if ( nkf_passed_ > cfg_->oc_submap_size_ ) {
                nkf_passed_ = 0;
                SubOctomap* new_sub_map = cur_sub_map_;
=======
            // ++ nkf_passed_;
            // if ( nkf_passed_ > cfg_->oc_submap_size_ ) {
            //     nkf_passed_ = 0;
            //     SubOctomap* new_sub_map = cur_sub_map_;
>>>>>>> 4efa7513f48053610bc0e35b228a8b9cff65430e
				
			// 	// insert one submap to fullmap.
			// 	octomap_fusion_->insertSubMap ( new_sub_map );
            // } // if have to insert submap.

        } // if have new keyframe.
        usleep ( 5000 );
    }// while true.
} // processing new keyframe.

// 关键帧从这里插入
void SubOctoMapConstruction::insertKeyFrame ( KeyFrame* kf )
{
    // std::cout << "insertKeyFrame !!!!!!!"<<std::endl; 
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    kfs_queue_.push ( kf );
}

// 这个函数就是来检查是否有关键帧
bool SubOctoMapConstruction::checkNewFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    return ( !kfs_queue_.empty() );
} // checkNewFrame

KeyFrame* SubOctoMapConstruction::getNewKeyFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    KeyFrame* kf = kfs_queue_.front();
    kfs_queue_.pop();
    return  kf;
} // getNewKeyFrame

//这个可以重写
void SubOctoMapConstruction::createPointCloud (KeyFrame* cur_kf, octomap::Pointcloud& point_cloud)
{
    for (unsigned int i = 0; i < cur_kf->point_3d_depth.size(); i++)
    {
        cv::Point3f pcl = cur_kf->point_3d_depth[i];
        point_cloud.push_back(pcl.x , pcl.y, pcl.z);
    }

} // createPointCloud

} // namespace dre_slam
