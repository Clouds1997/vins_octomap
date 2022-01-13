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

#ifndef SUB_OCTOMAP_H
#define SUB_OCTOMAP_H

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <sophus/se3.h>
#include "keyframe.h"
#include "config.h"
#include "ros_puber.h"

namespace rio_slam{
	
class SubOctomap{
public:
	SubOctomap( RosPuber *ros_puber, Config* cfg);
	void insertKeyFrame( KeyFrame* kf, octomap::Pointcloud& point_cloud_c );
	bool isContainKeyframe(KeyFrame* kf);
	
	~SubOctomap()
	{
		delete sub_octree_;
	}
	
	// main contant.
	RosPuber* ros_puber_;	
	Config* cfg_;
	octomap::ColorOcTree* sub_octree_;
	std::set<KeyFrame*> kfs_;
	KeyFrame* kf_base_;
};// class SubOctomap
	
} //namespace rio_slam


#endif
