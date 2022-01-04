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

#ifndef ROS_PUBER_H
#define ROS_PUBER_H

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>

namespace dre_slam{
class RosPuber{
public:
	RosPuber( ros::NodeHandle& nh );
	void pubOctoMap( octomap::ColorOcTree* octree );
	void pubsubMap( octomap::ColorOcTree* octree );
	
private:
	ros::NodeHandle nh_;

	// OctoMap.
	ros::Publisher puber_octomap_;
	ros::Publisher puber_submap_;
}; // RosPuber
}
 

	


#endif
