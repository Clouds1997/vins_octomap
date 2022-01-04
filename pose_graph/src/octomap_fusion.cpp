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
    OctoMapFusion::OctoMapFusion(RosPuber *ros_puber, Config *cfg) : ros_puber_(ros_puber), cfg_(cfg)
    {
        // std::cout << "OctoMapFusion is ok!" << std::endl;
        //首先先创建一个新的进程进行地图拼接的工作
        th_octomap_fusion_ = new std::thread(&OctoMapFusion::processing, this);
        full_map_ = new octomap::ColorOcTree(cfg_->oc_voxel_size_);
        full_map_->setOccupancyThres(cfg_->oc_occ_th_);
        full_map_->setProbHit(cfg_->oc_prob_hit_);
        full_map_->setProbMiss(cfg_->oc_prob_miss_);

        clamping_max_ = full_map_->getClampingThresMaxLog();
        clamping_min_ = full_map_->getClampingThresMinLog();
    } // OctomapMerging

    // 这个一看就是插入一个子图
    void OctoMapFusion::insertSubMap(SubOctomap *submap)
    {
        // std::cout << "pub a new submap" << std::endl;
        // // publish OctoMap.
        // ros_puber_->pubsubMap(submap->sub_octree_);
        std::unique_lock<mutex> lock(mutex_submaps_);
        submaps_.push_back(submap);
    } // insertSubMap

    void OctoMapFusion::insertOneScan2FullMapAndPub(KeyFrame *kf, octomap::Pointcloud &point_cloud_c)
    {
        Eigen::Vector3d P = kf->P;
        Eigen::Matrix3d R = kf->R;
        // Convert point cloud to world coordinate.
        octomap::Pointcloud point_cloud_w;
        // 这里是相机和世界坐标之间的转换
        for (size_t i = 0; i < point_cloud_c.size(); i++)
        {
            octomap::point3d &pt = point_cloud_c[i];
            Eigen::Vector3d ptc(pt.x(), pt.y(), pt.z());
            Eigen::Vector3d w_pts_i = R * (qi_d * ptc + ti_d) + P;
            // Delete error points.
            if (w_pts_i[2] > 6.0 || w_pts_i[2] < -2.0)
                continue;

            unsigned int B = kf->point_3d_color[i].x;
            unsigned int G = kf->point_3d_color[i].y;
            unsigned int R = kf->point_3d_color[i].z;

            // cout << "the color is ::"<<R << " "<< G << " "<< B << endl;

            point_cloud_w.push_back(octomap::point3d(w_pts_i[0], w_pts_i[1], w_pts_i[2]));
            full_map_->updateNode(octomap::point3d(w_pts_i[0], w_pts_i[1], w_pts_i[2]), true);

            // octomap::OcTreeKey key;
            // if (!full_map_->coordToKeyChecked(octomap::point3d(w_pts_i[0],w_pts_i[1],w_pts_i[2]), key))
            //     cout << "the test is NULL" << endl;
            full_map_->setNodeColor(w_pts_i[0], w_pts_i[1], w_pts_i[2],R,G,B);

            // full_map_->setNodeColor(w_pts_i[0], w_pts_i[1], w_pts_i[2], 0, 0, 0);
        }

        std::unique_lock<mutex> lock(mutex_full_map_);
        // update the map.
        // full_map_->insertPointCloud(point_cloud_w, octomap::point3d(0, 0, 0), -1, true, true);
        full_map_->updateInnerOccupancy();

        // publish OctoMap.
        ros_puber_->pubOctoMap(full_map_);
    } // insertOneScan2FullMap

    void OctoMapFusion::fusionAndPub()
    {
        std::unique_lock<mutex> lock_sub(mutex_submaps_);
        std::list<SubOctomap *> tmp_submaps = submaps_;

        if (tmp_submaps.size() == 0)
        {
            return;
        }

        // Create new tree.  这一步是初始化一个新的全局地图
        octomap::ColorOcTree *new_tree = new octomap::ColorOcTree(cfg_->oc_voxel_size_);
        new_tree->setOccupancyThres(cfg_->oc_occ_th_);
        new_tree->setProbHit(cfg_->oc_prob_hit_);
        new_tree->setProbMiss(cfg_->oc_prob_miss_);

        // insert sub tree to new tree.  遍历所有的子图，然后将这些小子图一个个的插入进行
        for (SubOctomap *submap : tmp_submaps)
        {
            insertSubMap2NewTree(submap, new_tree);
        }

        // 执行完成上面的步骤以后，就可以的得到一个新的在进行了回环的结果后的地图了
        std::unique_lock<mutex> lock_full(mutex_full_map_);
        delete full_map_;
        full_map_ = new_tree;

        // publish OctoMap.
        // std::cout << "loop ***************************************pub"<<endl;
        ros_puber_->pubOctoMap(full_map_);
    } // fuse the sub octrees.

    // 这一个是将小的子图，插入到全局地图里面
    void OctoMapFusion::insertSubMap2NewTree(SubOctomap *submap, octomap::ColorOcTree *new_tree)
    {
        // 这个接下去有一大段。不想看了，反正就是得到子图的一个位姿
        Eigen::Vector3d _T_w_i_b;
        Eigen::Matrix3d _R_w_i_b;
        submap->kf_base_->getVioPose(_T_w_i_b, _R_w_i_b);
        Sophus::SE3 Twc(_R_w_i_b, _T_w_i_b);
        transformTree(submap->sub_octree_, Twc, new_tree);
    } // insertSubMap2NewTree

    void OctoMapFusion::transformTree(octomap::ColorOcTree *src_tree, Sophus::SE3 &Twc, octomap::ColorOcTree *dst_tree)
    {
        // 这一步遍历了子图里面的每个体素，并且将他们都做了变换
        for (octomap::ColorOcTree::leaf_iterator it = src_tree->begin_leafs(); it != src_tree->end_leafs(); ++it)
        {
            // src.   获取到每个体素的坐标
            Eigen::Vector3d pt_src(it.getX(), it.getY(), it.getZ());

            octomap::ColorOcTreeNode::Color color = it->getColor();
            // cout << "the color is "<<(int)color.r << (int)color.g << (int)color.b << " "<<endl;
            // 然后又反过去把这个节点找出来？   是的 在这里换了数据的类型
            octomap::OcTreeNode *node_src = src_tree->search(pt_src(0), pt_src(1), pt_src(2));
            if (node_src == NULL)
            {
                continue;
            }
            // 获取这个体素节点的占据概率
            double prob_log = node_src->getLogOdds();

            // dest.   进行坐标的转换
            Eigen::Vector3d pt_dst = Twc * pt_src;

            octomap::OcTreeNode *node_dst = dst_tree->search(pt_dst(0), pt_dst(1), pt_dst(2));
            if (node_dst == NULL)
            {
                // 如果转换完成后，这个地方是空的，那就直接插入
                node_dst = dst_tree->updateNode(pt_dst(0), pt_dst(1), pt_dst(2), true);
                prob_log = std::max<double>(std::min<double>(prob_log, clamping_max_), clamping_min_);

                node_dst->setLogOdds(prob_log);
            }
            else
            {
                // 如果这个地方已经有其他体素占据了，就取两个概率最高的那一个
                double prob_log_dst = node_dst->getLogOdds();
                double sum_log = prob_log_dst + prob_log;

                sum_log = std::max<double>(std::min<double>(sum_log, clamping_max_), clamping_min_);
                node_dst->setLogOdds(sum_log);
            }
            dst_tree->setNodeColor(pt_dst(0), pt_dst(1), pt_dst(2) ,(int)color.r , (int)color.g , (int)color.b);
        } // for all leafs.  最后总体的来更新一下
        dst_tree->updateInnerOccupancy();
    } // transformTree

    void OctoMapFusion::processing()
    {
        static long int kf_num = 0;
        while (true)
        {
            if (getLoopFlag() == true)
            {
                // fusion & publish octomap.   这个如果得到了回环的标志，就进行地图的拼接
                // 关于这个标志位，则是下面的setLoopFlag函数进行置位
                fusionAndPub();
            }
            usleep(5000); // sleep 5 ms.
        }
    } // processing

    bool OctoMapFusion::getLoopFlag()
    {
        std::unique_lock<mutex> lock(mutex_loop_flag_);
        if (loop_flag_ == true)
        {
            loop_flag_ = false;
            return true;
        }
        return false;
    }

    // 在回环进程中被调用
    void OctoMapFusion::setLoopFlag()
    {
        std::unique_lock<mutex> lock(mutex_loop_flag_);
        loop_flag_ = true;
    }

    void OctoMapFusion::saveOctoMap(const string &dir)
    {
        std::unique_lock<mutex> lock_full(mutex_full_map_);
        full_map_->writeBinary(dir);
    } // saveOctoMap

}
