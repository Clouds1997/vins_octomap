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

#include "ros_puber.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "run_timer.h"

namespace dre_slam
{
RosPuber::RosPuber ( ros::NodeHandle& nh )
{
	
    // Current frame.
    // puber_robot_pose_ = nh.advertise<geometry_msgs::PoseStamped> ( "dre_slam/cur_robot_pose", 1, true );
    // image_transport::ImageTransport it ( nh );
    // puber_img_match_ = it.advertise ( "dre_slam/cur_img_match", 1, true );

    // // Dynamic pixel culling.
    // puber_dpc_img_objects_ = it.advertise ( "dre_slam/dpc_img_objects", 1, true );
    // puber_dpc_img_clusters_ = it.advertise ( "dre_slam/dpc_img_clusters", 1, true );
    // puber_dpc_img_mask_ = it.advertise ( "dre_slam/dpc_img_mask", 1, true );

    // KFs and pose graph. Sparse Map
    // puber_mappoints_ = nh.advertise<sensor_msgs::PointCloud> ( "dre_slam/mappoints", 1, true );
    // puber_kfs_puber_ = nh.advertise<geometry_msgs::PoseArray> ( "dre_slam/keyframes", 1, true );
    // puber_encoder_graph_ = nh.advertise<visualization_msgs::Marker> ( "dre_slam/encoder_graph", 1, true );
    // puber_loop_graph_ = nh.advertise<visualization_msgs::Marker> ( "dre_slam/loop_graph", 1, true );
    // puber_visual_graph_ = nh.advertise<visualization_msgs::Marker> ( "dre_slam/visual_graph", 1, true );

    // OctoMap
    std::cout << "rospub is ok !!!!"<<std::endl;
    puber_octomap_ = nh.advertise<octomap_msgs::Octomap> ( "vins_rbgd/octomap", 1, true );
    puber_submap_ = nh.advertise<octomap_msgs::Octomap> ( "vins_rbgd/submap", 1, true );
} // RosPuber


void RosPuber::pubOctoMap ( octomap::ColorOcTree* octree )
{
    // std::cout << "pubOctoMap!!!!"<<std::endl;
    RunTimer t; t.start();
    if ( puber_octomap_.getNumSubscribers() == 0 ) {
        return;
    }
    octree->prune();
    octomap_msgs::Octomap map_msg;
    octomap_msgs::fullMapToMsg( *octree, map_msg );
    map_msg.header.frame_id = "world";
    map_msg.header.stamp = ros::Time::now();
    puber_octomap_.publish ( map_msg );
	// std::cout << "Puber OctoMap time cost: " << t.duration() << "\n";
    t.stop();
} // pubOctoMap

void RosPuber::pubsubMap ( octomap::ColorOcTree* octree )
{
    // std::cout << "pubsubMap!!!!"<<std::endl;
    RunTimer t; t.start();
    if ( puber_submap_.getNumSubscribers() == 0 ) {
        return;
    }

    octomap_msgs::Octomap map_msg;
    octomap_msgs::fullMapToMsg( *octree, map_msg );
    map_msg.header.frame_id = "world";
    map_msg.header.stamp = ros::Time::now();
    puber_submap_.publish ( map_msg );
	// std::cout << "Puber OctoMap time cost: " << t.duration() << "\n";
    t.stop();
} // pubOctoMap

}
