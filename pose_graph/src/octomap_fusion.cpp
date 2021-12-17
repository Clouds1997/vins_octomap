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

#include "octomap_fusion.h"
#include "sub_octomap_construction.h"

namespace dre_slam
{
OctoMapFusion::OctoMapFusion (RosPuber* ros_puber) : ros_puber_(ros_puber)
{
    std::cout << "OctoMapFusion is ok!" << std::endl;
    //首先先创建一个新的进程进行地图拼接的工作
    // th_octomap_fusion_ = new std::thread ( &OctoMapFusion::processing, this );
    full_map_ = new octomap::OcTree ( 0.1 );
    full_map_->setOccupancyThres (0.61 );
    full_map_->setProbHit ( 0.6 );
    full_map_->setProbMiss ( 0.45);

    clamping_max_ = full_map_->getClampingThresMaxLog();
    clamping_min_ = full_map_->getClampingThresMinLog();
} // OctomapMerging

// 这个一看就是插入一个子图
void OctoMapFusion::insertSubMap ( SubOctomap* submap )
{
    std::cout << "pub a new submap" << std::endl;
    // publish OctoMap.
	ros_puber_->pubsubMap(submap->sub_octree_);
    std::unique_lock<mutex> lock ( mutex_submaps_ );
    submaps_.push_back ( submap );
} // insertSubMap

void OctoMapFusion::insertOneScan2FullMapAndPub ( KeyFrame* kf, octomap::Pointcloud& point_cloud_c)
{
    Eigen::Vector3d P = kf->P;
    Eigen::Matrix3d R = kf->R;
    // Convert point cloud to world coordinate.
    octomap::Pointcloud point_cloud_w;
    // 这里是相机和世界坐标之间的转换
    for ( size_t i = 0; i < point_cloud_c.size(); i ++ ) {
        octomap::point3d& pt = point_cloud_c[i];
        Eigen::Vector3d ptc ( pt.x(), pt.y(), pt.z() );
        Eigen::Vector3d w_pts_i = R * (qi_d * ptc + ti_d) + P;
        // Delete error points. 
		if(w_pts_i[2] > 6.0 || w_pts_i[2] < -2.0)
			continue;
		
        point_cloud_w.push_back ( octomap::point3d ( w_pts_i[0], w_pts_i[1], w_pts_i[2] ) );
    }

    std::unique_lock<mutex> lock ( mutex_full_map_ );
    // update the map.
    full_map_->insertPointCloud ( point_cloud_w, octomap::point3d (0,0,0 ), -1, true, true );
    full_map_->updateInnerOccupancy();
	
	// publish OctoMap.
	ros_puber_->pubOctoMap(full_map_);
} // insertOneScan2FullMap

}


