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

#include "config.h"

namespace dre_slam{
	
Config::Config ( const std::string& cfg_dir)
{

	cv::FileStorage fs ( cfg_dir, cv::FileStorage::READ );
	
	/**** OctoMap Construction ****/
	fs["oc_voxel_size_"] >> oc_voxel_size_;   // Voxel size of the OctoMap (m).
	fs["oc_submap_size_"] >> oc_submap_size_; // Sub-OctoMap size (KFs)
	
	fs.release();
} // Config

} // namespace dre_slam
