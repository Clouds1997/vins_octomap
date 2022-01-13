// This file is part of rio_slam - Dynamic RGB-D Encoder SLAM for Differential-Drive Robot.
//
// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)
//
// rio_slam is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// rio_slam is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef OCTOMAP_FUSION
#define OCTOMAP_FUSION


#include <thread>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<pcl/filters/passthrough.h>  //直通滤波器头文件
#include<pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include<pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件
#include <sophus/se3.h>
#include "config.h"
#include "ros_puber.h"
#include "keyframe.h"
#include "sub_octomap.h"

namespace rio_slam {
class SubOctoMapConstruction;

class OctoMapFusion{
public:
	OctoMapFusion(RosPuber *ros_puber, Config* cfg);
	// ~OctoMapFusion();
	void insertSubMap( SubOctomap* submap );
	void insertOneScan2FullMapAndPub ( KeyFrame* kf, octomap::Pointcloud& point_cloud_c);
	void fusionAndPub();  
	void insertSubMap2NewTree(SubOctomap* submap, octomap::ColorOcTree* new_tree);
	void processing();
	void setLoopFlag();
	bool getLoopFlag();
	void saveOctoMap( const std::string& dir );
	
private: 
	void transformTree(octomap::ColorOcTree* src_tree, Sophus::SE3& Twc, octomap::ColorOcTree* dst_tree);

	RosPuber* ros_puber_;	
	Config* cfg_;
	octomap::ColorOcTree* full_map_;
	std::list<SubOctomap*> submaps_;
	
	std::thread* th_octomap_fusion_;
	std::mutex mutex_submaps_;
	std::mutex mutex_full_map_;
	
	std::mutex mutex_loop_flag_;
	bool loop_flag_ = false;
	KeyFrame* ckf_; 
	
	double clamping_max_;
	double clamping_min_;

}; // class OctoMapFusion
	
} // namespace rio_slam

#endif