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


#include "sub_octomap.h"

namespace dre_slam
{

SubOctomap::SubOctomap ( Config* cfg) :cfg_ ( cfg )
{
    sub_octree_ = new octomap::ColorOcTree ( cfg->oc_voxel_size_ );
    sub_octree_->setOccupancyThres ( cfg->oc_occ_th_ );
    sub_octree_->setProbHit ( cfg->oc_prob_hit_ );
    sub_octree_->setProbMiss ( cfg->oc_prob_miss_ );
} // constructor

void SubOctomap::insertKeyFrame ( KeyFrame* kf, octomap::Pointcloud& point_cloud_c )
{
    // std::cout << "insertKeyFrame in sub_octomap !!!!!!" << std::endl;
    if ( kfs_.size() == 0 ) {
        kf_base_ = kf;
        
    } // the first is set as base.
    
    kfs_.insert ( kf );
	Eigen::Vector3d _T_w_i_b;
    Eigen::Matrix3d _R_w_i_b;
    kf_base_->getVioPose(_T_w_i_b, _R_w_i_b);

    Eigen::Vector3d _T_w_i_c;
    Eigen::Matrix3d _R_w_i_c;
    kf->getVioPose(_T_w_i_c, _R_w_i_c);

    Eigen::Vector3d _T_w_i_bc;
    Eigen::Matrix3d _R_w_i_bc;

    _T_w_i_bc = _T_w_i_c - _T_w_i_b;
    _R_w_i_bc = _R_w_i_b.inverse() * _R_w_i_c;

    // Sophus::SE3 Tbc(_R_w_i_bc,_T_w_i_bc);

    // rotate the point cloud to base frame.
    octomap::Pointcloud point_cloud_b;
    for ( size_t i = 0; i < point_cloud_c.size(); i ++ ) {
        octomap::point3d& pt = point_cloud_c[i];
        Eigen::Vector3d ptc ( pt.x(), pt.y(), pt.z() );
        Eigen::Vector3d ptb =_R_w_i_bc * (qi_d * ptc + ti_d) + _T_w_i_bc;
		
		// Delete error points.
		if(ptb[2] > 6.0 || ptb[2] < -2.0)
			continue;
		
		
        // point_cloud_b.push_back ( octomap::point3d ( ptb[0], ptb[1], ptb[2] ) );


        unsigned int B = kf->point_3d_color[i].x;
        unsigned int G = kf->point_3d_color[i].y;
        unsigned int R = kf->point_3d_color[i].z;

        // cout << "the color is ::"<<R << " "<< G << " "<< B << endl;
        sub_octree_->updateNode(octomap::point3d(ptb[0], ptb[1], ptb[2] ), true);

        // octomap::OcTreeKey key;
        // if (!full_map_->coordToKeyChecked(octomap::point3d(w_pts_i[0],w_pts_i[1],w_pts_i[2]), key))
        //     cout << "the test is NULL" << endl;
        sub_octree_->setNodeColor(ptb[0], ptb[1], ptb[2] ,R,G,B);

    }

    // Eigen::Vector3d& pt_o = Tbc.translation();
    // std::cout <<  pt_o[0] << " " << pt_o[1] << " " << pt_o[2] << "!!!!!!!!!!!!!!!!!11" << std::endl;
    // sub_octree_->insertPointCloud ( point_cloud_b, octomap::point3d ( pt_o[0],pt_o[1],pt_o[2] ), -1, true, true );
    // sub_octree_->insertPointCloud ( point_cloud_b, octomap::point3d ( 0,0,0), -1, true, true );
    sub_octree_->updateInnerOccupancy();

} // insertKeyFrame


bool SubOctomap::isContainKeyframe ( KeyFrame* kf )
{
    std::set<KeyFrame*>::iterator iter = kfs_.find ( kf );
    if ( iter == kfs_.end() ) {
        return false;
    }
    return true;
} // isContainKeyframe


} // namespace dre_slam

