#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#include "config.h"
#include <image_obj_msgs/ImageBox.h>
#include <image_obj_msgs/ImageObj.h>

#include <KMCUDA/kmcuda.h>
// #include <DBoW2/DBoW2/FORB.h>


#include <sys/time.h>

#define SKIP_FIRST_CNT 10
using namespace std;

queue<sensor_msgs::ImageConstPtr> image_buf,depth_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
// queue< image_feature_msgs::ImageFeaturesConstPtr> imagefeature_buf;
queue< image_obj_msgs::ImageObjConstPtr> imageobj_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

float PCL_MAX_DIST, PCL_MIN_DIST, RESOLUTION;
int U_BOUNDARY, D_BOUNDARY, L_BOUNDARY, R_BOUNDARY;
int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int PCL_DIST;
int DEBUG_IMAGE;
int VISUALIZE_IMU_FORWARD;
int LOOP_CLOSURE;
int FAST_RELOCALIZATION;


camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;

Eigen::Matrix<double, 3, 1> ti_d;
Eigen::Matrix<double, 3, 3> qi_d;

ros::Publisher pub_match_img;
ros::Publisher pub_match_points;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_key_odometrys;
ros::Publisher pub_vio_path;
nav_msgs::Path no_loop_path;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

//not used in my case, just ignore sequence 1-5
void new_sequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!depth_buf.empty())
        depth_buf.pop();
    // while(!imagefeature_buf.empty())
    //     imagefeature_buf.pop();
    while(!imageobj_buf.empty())
        imageobj_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
    //ROS_WARN("image_callback!1");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    image_buf.push(image_msg);
    depth_buf.push(depth_msg);
    m_buf.unlock();
    //printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 || image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_msg->header.stamp.toSec();
}

void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    //ROS_INFO("point_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x,
                                                     point_msg->points[i].y,
                                                     point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],
                                                     point_msg->channels[i].values[1]);
    }
    */
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("pose_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}

// not used
void imu_forward_callback(const nav_msgs::Odometry::ConstPtr &forward_msg)
{
    if (VISUALIZE_IMU_FORWARD)
    {
        Vector3d vio_t(forward_msg->pose.pose.position.x, forward_msg->pose.pose.position.y, forward_msg->pose.pose.position.z);
        Quaterniond vio_q;
        vio_q.w() = forward_msg->pose.pose.orientation.w;
        vio_q.x() = forward_msg->pose.pose.orientation.x;
        vio_q.y() = forward_msg->pose.pose.orientation.y;
        vio_q.z() = forward_msg->pose.pose.orientation.z;

        vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
        vio_q = posegraph.w_r_vio *  vio_q;

        vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
        vio_q = posegraph.r_drift * vio_q;

        Vector3d vio_t_cam;
        Quaterniond vio_q_cam;
        vio_t_cam = vio_t + vio_q * tic;
        vio_q_cam = vio_q * qic;

        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, forward_msg->header);
    }
}
void relo_relative_pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    Vector3d relative_t = Vector3d(pose_msg->pose.pose.position.x,
                                   pose_msg->pose.pose.position.y,
                                   pose_msg->pose.pose.position.z);
    Quaterniond relative_q;
    relative_q.w() = pose_msg->pose.pose.orientation.w;
    relative_q.x() = pose_msg->pose.pose.orientation.x;
    relative_q.y() = pose_msg->pose.pose.orientation.y;
    relative_q.z() = pose_msg->pose.pose.orientation.z;
    double relative_yaw = pose_msg->twist.twist.linear.x;
    int index = pose_msg->twist.twist.linear.y;
    //printf("receive index %d \n", index );
    Eigen::Matrix<double, 8, 1 > loop_info;
    loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
                 relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                 relative_yaw;
    posegraph.updateKeyFrameLoop(index, loop_info);

}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio *  vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;

    if (!VISUALIZE_IMU_FORWARD)
    {
        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
    }

    odometry_buf.push(vio_t_cam);
    if (odometry_buf.size() > 10)
    {
        odometry_buf.pop();
    }

    visualization_msgs::Marker key_odometrys;
    key_odometrys.header = pose_msg->header;
    key_odometrys.header.frame_id = "world";
    key_odometrys.ns = "key_odometrys";
    key_odometrys.type = visualization_msgs::Marker::SPHERE_LIST;
    key_odometrys.action = visualization_msgs::Marker::ADD;
    key_odometrys.pose.orientation.w = 1.0;
    key_odometrys.lifetime = ros::Duration();

    //static int key_odometrys_id = 0;
    key_odometrys.id = 0; //key_odometrys_id++;
    key_odometrys.scale.x = 0.1;
    key_odometrys.scale.y = 0.1;
    key_odometrys.scale.z = 0.1;
    key_odometrys.color.r = 1.0;
    key_odometrys.color.a = 1.0;

    for (unsigned int i = 0; i < odometry_buf.size(); i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d vio_t;
        vio_t = odometry_buf.front();
        odometry_buf.pop();
        pose_marker.x = vio_t.x();
        pose_marker.y = vio_t.y();
        pose_marker.z = vio_t.z();
        key_odometrys.points.push_back(pose_marker);
        odometry_buf.push(vio_t);
    }
    pub_key_odometrys.publish(key_odometrys);

    // not used
    if (!LOOP_CLOSURE)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = pose_msg->header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = vio_t.x();
        pose_stamped.pose.position.y = vio_t.y();
        pose_stamped.pose.position.z = vio_t.z();
        no_loop_path.header = pose_msg->header;
        no_loop_path.header.frame_id = "world";
        no_loop_path.poses.push_back(pose_stamped);
        pub_vio_path.publish(no_loop_path);
    }
}

void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}

// void featureCallback(const image_feature_msgs::ImageFeaturesConstPtr &msg)
// {
//     if(!LOOP_CLOSURE)
//         return;
//      m_buf.lock();
//       cout <<fixed<<setprecision(6)<< "image_feature_msgs "<<msg->header.stamp.toSec()<<endl;
//      imagefeature_buf.push(msg);
//      m_buf.unlock();
// }

void image_objCallback(const  image_obj_msgs::ImageObjConstPtr &msg)
{
    if(!LOOP_CLOSURE)
        return;

        
        //  cout << msg->header.stamp.toSec()<<endl;
     m_buf.lock();
     imageobj_buf.push(msg);
     m_buf.unlock();
}


// void segmentPointCloudByKmeans ( const vector< Eigen::Vector2d >& pts2d, const vector< Eigen::Vector3d >& pts3d, const int n_clusters, vector< vector< Eigen::Vector2d > >& clusters_2d, vector< vector< Eigen::Vector3d > >& clusters_3d )
// {
//     // Convert
//     cv::Mat points ( pts3d.size(), 3, CV_32F, cv::Scalar ( 0,0,0 ) );
//     cv::Mat centers ( n_clusters, 1, points.type() );

//     uint32_t size = pts3d.size();

//     float samples[size * 3];

//     // Convert to opencv type
//     for ( size_t i = 0; i < pts3d.size(); i ++ ) {
//         const Eigen::Vector3d& ept = pts3d.at ( i );
//         points.at<float> ( i, 0 ) = ept[0];
//         points.at<float> ( i, 1 ) = ept[1];
//         points.at<float> ( i, 2 ) = ept[2];
//         samples[i * 3 + 0] = ept[0];
//         samples[i * 3 + 1] = ept[1];
//         samples[i * 3 + 2] = ept[2];
//     } // for all points


//     // Do Kmeans
//     // cv::Mat labels;
//     // cv::TermCriteria criteria = cv::TermCriteria ( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 5, 2.0 );
//     // cv::kmeans ( points, n_clusters, labels, criteria, 3, cv::KMEANS_PP_CENTERS, centers );

//     // uint32_t assignments[size];
//     // float centroids[size];

//     // KMCUDAResult result = kmeans_cuda(
//     // kmcudaInitMethodPlusPlus, NULL,  // kmeans++ centroids initialization
//     // 0.1,                            // less than 1% of the samples are reassigned in the end
//     // 0.1,                             // activate Yinyang refinement with 0.1 threshold
//     // kmcudaDistanceMetricL2,          // Euclidean distance
//     // size , 3, n_clusters,
//     // 0xDEADBEEF,                      // random generator seed
//     // 1,                               // use all available CUDA devices
//     // -1,                              // samples are supplied from host
//     // 0,                               // not in float16x2 mode
//     // 1,                               // moderate verbosity
//     // samples, centroids, assignments,nullptr);

//     // Collect clusters.
//     clusters_2d.clear();
//     clusters_3d.clear();

//     clusters_2d.resize ( n_clusters );
//     clusters_3d.resize ( n_clusters );

//     for ( size_t i = 0; i < pts3d.size(); i ++ ) {
//         // int label_idx = assignments[i];
//         // int label_idx = labels.at<int> ( i, 0 );
//         clusters_2d.at ( label_idx ).push_back ( pts2d.at ( i ) );
//         clusters_3d.at ( label_idx ).push_back ( pts3d.at ( i ) );
//     }

// } // segmentPointCloudByKmeans

void drawClustersOnImage ( cv::Mat& io_img, const vector< vector< Eigen::Vector2d > >& clusters_2d, const std::vector<uint8_t>& colors )
{

    for ( size_t i = 0; i < clusters_2d.size(); i ++ ) {
        uint8_t r = colors.at ( i * 3 );
        uint8_t g = colors.at ( i * 3 + 1 );
        uint8_t b = colors.at ( i * 3 + 2 );

        // draw
        const std::vector<Eigen::Vector2d>& pts = clusters_2d.at ( i );
        for ( size_t j = 0; j < pts.size(); j ++ ) {
            const Eigen::Vector2d& pt = pts.at ( j );
            io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [0] = r;
            io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [1] = g;
            io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [2] = b;
        }
    }
} // drawClustersOnImages


void process()
{
    if (!LOOP_CLOSURE)
        return;
    while (true)
    {
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::ImageConstPtr depth_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;
        // image_feature_msgs::ImageFeaturesConstPtr  imagefeature_msg = NULL;
        image_obj_msgs::ImageObjConstPtr imageobj_msg = NULL;
        // find out the messages with same time stamp

        m_buf.lock();
        //get image_msg, pose_msg and point_msg
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty() && !imageobj_buf.empty())
        {
            // cout <<fixed<<setprecision(6)<< "===============image_msg "<<image_buf.front()->header.stamp.toSec()<<endl;
            // cout <<fixed<<setprecision(6)<< "!!!!!!!!!!!!!!!!!!!!!!!!!!imageobj_msg "<<imageobj_buf.back()->header.stamp.toSec()<<endl;
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            // else if (image_buf.front()->header.stamp.toSec() > imagefeature_buf.front()->header.stamp.toSec())
            // {
            //     imagefeature_buf.pop();
            //     printf("throw imagefeature_buf at beginning\n");
            // }  
             else if ( image_buf.front()->header.stamp.toSec() > imageobj_buf.front()->header.stamp.toSec() )
            {
                // cout <<fixed<<setprecision(6)<<"imageobj_buf.back()"<< imageobj_buf.back()->header.stamp.toSec()<<endl;
                //   cout <<"imageobj_buf size"<< imageobj_buf.size()<<endl;
                imageobj_buf.pop();
                printf("throw imageobj_buf at beginning\n");
            }  
             else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec()
                && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec() &&  !imageobj_buf.empty()
                )
            {
                // cout <<fixed<<setprecision(6)<<"imageobj_buf.size() "<< imageobj_buf.size()<<endl;

                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                {
                    image_buf.pop();
                    depth_buf.pop();
                }
                image_msg = image_buf.front();
                depth_msg = depth_buf.front();
                image_buf.pop();
                depth_buf.pop();
               
            //    while (imagefeature_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
            //         imagefeature_buf.pop();   
            //     imagefeature_msg = imagefeature_buf.front();
            //     imagefeature_buf.pop();

                // cout <<"====================================================="<<endl;
                //  cout <<fixed<<setprecision(6)<<"imageobj_buf.front() "<< imageobj_buf.front()->header.stamp.toSec()<<endl;
                //  cout <<fixed<<setprecision(6)<<"imageobj_buf.back()"<< imageobj_buf.back()->header.stamp.toSec()<<endl;
                //  cout <<fixed<<setprecision(6)<<"pose_buf.front() "<<pose_msg->header.stamp.toSec()<<endl;
                if ( imageobj_buf.back()->header.stamp.toSec() >= pose_msg->header.stamp.toSec()){
                     while (imageobj_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec() )
                            imageobj_buf.pop();   
                    // cout << "**************box******************"<<endl;
                    if ( !imageobj_buf.empty()){
                        imageobj_msg = imageobj_buf.front();
                    imageobj_buf.pop();
                    }
                }
              
                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();
        
        if (pose_msg != NULL)
        {
            //printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            //printf(" point time %f \n", point_msg->header.stamp.toSec());
            //printf(" image time %f \n", image_msg->header.stamp.toSec());
            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            cv_bridge::CvImageConstPtr ptr;
            cv_bridge::CvImageConstPtr ptr_rgb;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr_rgb = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
                // ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

            //depth has encoding TYPE_16UC1
            cv_bridge::CvImageConstPtr depth_ptr;
            // debug use     std::cout<<depth_msg->encoding<<std::endl;
            {
                sensor_msgs::Image img;
                img.header = depth_msg->header;
                img.height = depth_msg->height;
                img.width = depth_msg->width;
                img.is_bigendian = depth_msg->is_bigendian;
                img.step = depth_msg->step;
                img.data = depth_msg->data;
                img.encoding = sensor_msgs::image_encodings::MONO16;
                depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
            }

            cv::Mat image;
            cv::Mat image_rgb;
            
            cv::cvtColor(ptr_rgb->image,image_rgb,  CV_BGR2RGB);
            cv::cvtColor(ptr_rgb->image,image,  CV_BGR2GRAY);

            // cv::imshow("rgb", image_rgb);
            // cv::waitKey(1);

            
            cv::Mat depth = depth_ptr->image;
            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d;
                vector<cv::Point2f> point_2d_uv;
                vector<cv::Point2f> point_2d_normal;
                vector<cv::Point3f> point_3d_depth;
                vector<cv::Point3i> point_3d_color;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }

                // 获取动态锚框
                vector<Object_t> objects;
                cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
                //debug: ROS_WARN("Depth points count: %d", count_);
                if (imageobj_msg!= NULL){
                    // cv::Mat show_img;
                    // cv::cvtColor(ptr->image,show_img,  CV_GRAY2RGB);
                     cout << "**************box******************"<< imageobj_msg-> boxes.size() <<endl;
                    for ( auto &box : imageobj_msg-> boxes ){       
                        Object_t obj;
                        obj.id = box.box[6];
                        obj.tl.x = box.box[0];
                        obj.tl.y = box.box[1];
                        obj.br.x = box.box[2];
                        obj.br.y = box.box[3];
                        obj.score = box.box[4];
                        cv::Rect rect(box.box[0],box.box[1], box.box[2] - box.box[0], box.box[3] -  box.box[1]);
                        mask(rect).setTo(255);
                        // cv::rectangle(show_img,obj.tl,obj.br, cv::Scalar(0,255,255) ,1);
                        objects.push_back(obj);
                        // cout << box.box.size()<<endl;
                    }
                    // cv::imshow("rgb", show_img);
                    // cv::waitKey(1);
                }
                // cv::imshow("mask", mask);
                // cv::waitKey(1);
                // cv::imshow("ori", image);


                // ROW: 480 y  COL: 640 x
                // //debug: int count_ = 0;
                // std::vector<Eigen::Vector2d> pts2d;
                // std::vector<Eigen::Vector3d> pts3d;

                for (int i = L_BOUNDARY; i < COL - R_BOUNDARY; i += PCL_DIST)
                {
                    for (int j = U_BOUNDARY; j < ROW - D_BOUNDARY; j += PCL_DIST)
                    {
                        Eigen::Vector2d a(i, j);
                        Eigen::Vector3d b;
						//depth is aligned
                        m_camera->liftProjective(a, b);
                        float depth_val = ((float)depth.at<unsigned short>(j, i)) / 1000.0;
                        //  && mask.at<uchar>(i,j) == 0 去除锚框中的点   && static_cast<int>(mask.ptr<uchar>(j)[i]) == 0
                        // cout << mask.size() << endl;
                        if (depth_val > PCL_MIN_DIST && depth_val < PCL_MAX_DIST&& (int)(mask.at<unsigned char>(j,i)) == 0)
                        {
                            //debug: ++count_;

                            cv::Vec3b rgb=image_rgb.at<cv::Vec3b>(j,i);

                            int R = static_cast<int>(rgb[0]);
                            int G = static_cast<int>(rgb[1]);
                            int B = static_cast<int>(rgb[2]);

                            // R = R < 100 ? 50 : R < 200 ? 150 : 225;
                            // G = G < 100 ? 50 : G < 200 ? 150 : 225;
                            // B = B < 100 ? 50 : B < 200 ? 150 : 225;
                            // std::cout << (int)mask.at<unsigned char>(j,i) <<" ";
                            point_3d_depth.push_back(cv::Point3f(b.x() * depth_val, b.y() * depth_val, depth_val));
                            point_3d_color.push_back(cv::Point3i(R,G,B));
                            // point_3d_color.push_back(cv::Point3i(255,255,255));
                            // pts3d.push_back(Eigen::Vector3d(b.x() * depth_val, b.y() * depth_val, depth_val));
                            // pts2d.push_back ( Eigen::Vector2d ( i, j) );
                        }
                    }
                }
		        // cout << point_3d_color.size() << endl;
                // cout << endl;

                // struct timeval timeA;
	            // gettimeofday(&timeA, NULL);

                // if(pts2d.size()> 0){
                //     // Calc Number of clusters.
                //     int K_clusters = std::ceil ( ( double ) pts2d.size() / ( double ) 12000);

                //     std::vector<std::vector<Eigen::Vector2d>> clusters_2d;
                //     std::vector<std::vector<Eigen::Vector3d>> clusters_3d;
                //     segmentPointCloudByKmeans ( pts2d, pts3d, K_clusters, clusters_2d, clusters_3d );

                //     cv::Mat clusters = cv::Mat (image.size(), CV_8UC3, cv::Scalar ( 0, 0, 0 ) );
                    
                //     struct timeval timeB;
                //     gettimeofday(&timeB, NULL);
                //     cout << (timeB.tv_usec - timeA.tv_usec)/1000<<"ms to cluster" << endl;
                //     std::vector<uint8_t> colors_ = {213,0,0,197,17,98,170,0,255,98,0,234,48,79,254,41,98,255,0,145,234,0,184,212,0,191,165,0,200,83,100,221,23,174,234,0,255,214,0,255,171,0,255,109,0,221,44,0,62,39,35,33,33,33,38,50,56,144,164,174,224,224,224,161,136,127,255,112,67,255,152,0,255,193,7,255,235,59,192,202,51,139,195,74,67,160,71,0,150,136,0,172,193,3,169,244,100,181,246,63,81,181,103,58,183,171,71,188,236,64,122,239,83,80, 213,0,0,197,17,98,170,0,255,98,0,234,48,79,254,41,98,255,0,145,234,0,184,212,0,191,165,0,200,83,100,221,23,174,234,0,255,214,0,255,171,0,255,109,0,221,44,0,62,39,35,33,33,33,38,50,56,144,164,174,224,224,224,161,136,127,255,112,67,255,152,0,255,193,7,255,235,59,192,202,51,139,195,74,67,160,71,0,150,136,0,172,193,3,169,244,100,181,246,63,81,181,103,58,183,171,71,188,236,64,122,239,83,80};
                //     // draw
                //     drawClustersOnImage (clusters,clusters_2d, colors_ );

                //     cv::imshow("clusters", clusters);
                //     cv::waitKey(1);
                // }
            
                

                // 通过frame_index标记对应帧
                // add sparse depth img to this class point_3d_color,
                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image, point_3d_depth, point_3d_color,
                                   point_3d, point_2d_uv, point_2d_normal, point_id,sequence);
                m_process.lock();
                start_flag = 1;
                posegraph.addKeyFrame(keyframe, 1);
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void command()
{
    if (!LOOP_CLOSURE)
        return;
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            printf("program shutting down...\n");
            ros::shutdown();
        }
        if (c == 'n')
            new_sequence();

        if (c == 'p')
        {
            TicToc t_filter;
            posegraph.pclFilter(false);
            printf("pclFilter time: %f", t_filter.toc());
        }
        if (c == 'd')
        {
            TicToc t_pcdfile;
            posegraph.save_cloud->width = posegraph.save_cloud->points.size();
            posegraph.save_cloud->height = 1;
	        pcl::io::savePCDFileASCII("/home/riger/pcd_file_"+to_string(frame_index)+"keyframes.pcd", *(posegraph.save_cloud));
            printf("Save pcd file done! Time cost: %f", t_pcdfile.toc());
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_graph");
    ros::NodeHandle n("~");
    std::string octomap_cfg_dir;
    if ( ! n.getParam ( "octomap_cfg_dir", octomap_cfg_dir ) ) {
		std::cout << "Read octomap_cfg_dir failure !\n";
        return -1;
    }
	// Load system configure.
	dre_slam::Config* cfg = new dre_slam::Config( octomap_cfg_dir );
    posegraph.registerPub(n, cfg);
    cout <<"there is ok"<<endl;
    // read param
    n.getParam("visualization_shift_x", VISUALIZATION_SHIFT_X);
    n.getParam("visualization_shift_y", VISUALIZATION_SHIFT_Y);
    n.getParam("skip_cnt", SKIP_CNT);
    n.getParam("skip_dis", SKIP_DIS);
    std::string config_file;
    n.getParam("config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    double camera_visual_size = fsSettings["visualize_camera_size"];
    cameraposevisual.setScale(camera_visual_size);
    cameraposevisual.setLineWidth(camera_visual_size / 10.0);


    LOOP_CLOSURE = fsSettings["loop_closure"];
    std::string IMAGE_TOPIC,DEPTH_TOPIC;
    int LOAD_PREVIOUS_POSE_GRAPH;
    // prepare for loop closure (load vocabulary, set topic, etc)
    if (LOOP_CLOSURE)
    {
        ROW = fsSettings["image_height"];
        COL = fsSettings["image_width"];
        PCL_DIST = fsSettings["pcl_dist"];
        U_BOUNDARY = fsSettings["u_boundary"];
        D_BOUNDARY = fsSettings["d_boundary"];
        L_BOUNDARY = fsSettings["l_boundary"];
        R_BOUNDARY = fsSettings["r_boundary"];
        PCL_MIN_DIST = fsSettings["pcl_min_dist"];
        PCL_MAX_DIST = fsSettings["pcl_max_dist"];
		RESOLUTION = fsSettings["resolution"];
        //OctreePointCloudDensity has no ::Ptr
		posegraph.octree = new pcl::octree::OctreePointCloudDensity<pcl::PointXYZ>(RESOLUTION);
	    posegraph.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        posegraph.save_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
		posegraph.octree->setInputCloud(posegraph.cloud);
        posegraph.octree->addPointsFromInputCloud();
		// in pcl 1.8.0+, need to set bbox (isVoxelOccupiedAtPoint will check bbox)
		posegraph.octree->defineBoundingBox(-100, -100, -100, 100, 100, 100);
        std::string pkg_path = ros::package::getPath("pose_graph");
        string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
        cout << "vocabulary_file" << vocabulary_file << endl;
        posegraph.loadVocabulary(vocabulary_file);

        posegraph.loadNetVlad();

        BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
        cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

        fsSettings["image_topic"] >> IMAGE_TOPIC;
        fsSettings["depth_topic"] >> DEPTH_TOPIC;
        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        fsSettings["output_path"] >> VINS_RESULT_PATH;
        fsSettings["save_image"] >> DEBUG_IMAGE;

        cv::Mat cv_qid, cv_tid;
        fsSettings["extrinsicRotation"] >> cv_qid;
        fsSettings["extrinsicTranslation"] >> cv_tid;
        cv::cv2eigen(cv_qid, qi_d);
        cv::cv2eigen(cv_tid, ti_d);

        VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];
        LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
        FAST_RELOCALIZATION = fsSettings["fast_relocalization"];
        VINS_RESULT_PATH = VINS_RESULT_PATH + "/vins_result_loop.csv";
        std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
        fout.close();
        fsSettings.release();
        //not used
        if (LOAD_PREVIOUS_POSE_GRAPH)
        {
            printf("load pose graph\n");
            m_process.lock();
            posegraph.loadPoseGraph();
            m_process.unlock();
            printf("load pose graph finish\n");
            load_flag = 1;
        }
        else
        {
            printf("no previous pose graph\n");
            load_flag = 1;
        }
    }

    fsSettings.release();
    // publish camera pose by imu propagate and odometry (Ps and Rs of curr frame)
    // not important
    ros::Subscriber sub_imu_forward = n.subscribe("/vins_estimator/imu_propagate", 2000, imu_forward_callback);
    // odometry_buf
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 2000, vio_callback);

    //get image msg, store in image_buf
    //ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 2000, image_callback);
    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, IMAGE_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(n, DEPTH_TOPIC, 1);
    //message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub_image, sub_depth, 2000);
    // fit fisheye camera
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_image, sub_depth);
    sync.registerCallback(boost::bind(&image_callback, _1, _2));

    //get keyframe_pose(Ps and Rs), store in pose_buf (marginalization_flag == 0)
    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
    //get extrinsic (ric qic  odometry.pose.pose.position and odometry.pose.pose.orientation)
    //update tic and qic real-time
    ros::Subscriber sub_extrinsic = n.subscribe("/vins_estimator/extrinsic", 2000, extrinsic_callback);
    //get keyframe_point(pointclude), store in point_buf (marginalization_flag == 0)
    ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point", 2000, point_callback);
    
    // ros::Subscriber sub_feature_ = n.subscribe("/d400/features", 2000, featureCallback);//netvlad

    ros::Subscriber sub_obj = n.subscribe("/image/detected_obj", 2000, image_objCallback,ros::TransportHints().tcpNoDelay()); //yolov4 box

    // do relocalization here.
    // pose_graph publish match_points to vins_estimator, estimator then publish relo_relative_pose
    ros::Subscriber sub_relo_relative_pose = n.subscribe("/vins_estimator/relo_relative_pose", 2000, relo_relative_pose_callback);

    pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_key_odometrys = n.advertise<visualization_msgs::Marker>("key_odometrys", 1000);
    //not used
    pub_vio_path = n.advertise<nav_msgs::Path>("no_loop_path", 1000);
    pub_match_points = n.advertise<sensor_msgs::PointCloud>("match_points", 100);

    std::thread measurement_process;
    std::thread keyboard_command_process;
    // main thread
    measurement_process = std::thread(process);
    // not used
    keyboard_command_process = std::thread(command);


    ros::spin();

    return 0;
}
