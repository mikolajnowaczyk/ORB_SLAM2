/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace ORB_SLAM2;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

tf::Transform TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);

  tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}

/*void PublishPositionAsTransform (cv::Mat position, std::string map_frame_id_param, std::string camera_frame_id_param, ros::Time current_frame_time;) {
  tf::Transform transform = TransformFromMat (position);
  static tf::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, current_frame_time, map_frame_id_param, camera_frame_id_param));
}*/

void PublishPositionAsPoseStamped (cv::Mat position, ros::Publisher pose_publisher,ros::Time current_frame_time, std::string map_frame_id_param) {
  tf::Transform grasp_tf = TransformFromMat(position);
  tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time, map_frame_id_param);
  geometry_msgs::PoseStamped pose_msg;
  tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg);
  pose_publisher.publish(pose_msg);
}

void PublishGraphAsPoseStamped(vector<KeyFrame*> KFGraph, ros::Publisher kf_graph_publisher,ros::Time current_frame_time, std::string map_frame_id_param) {
  geometry_msgs::PoseArray pose_array_msg;
  geometry_msgs::PoseStamped pose_stamped_msg;
  geometry_msgs::Pose pose_msg;
  cv::Mat position;
  tf::Transform grasp_tf;
  for(unsigned int i = 0; i < KFGraph.size(); i++)
  {
      position = KFGraph[i]->GetPose();
      grasp_tf = TransformFromMat(position);
      tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time, map_frame_id_param);
      tf::poseStampedTFToMsg (grasp_tf_pose, pose_stamped_msg);
      pose_msg = pose_stamped_msg.pose;
      pose_array_msg.poses.push_back(pose_msg);
      //KFGraph.push_back(pose_msg);
      //kf_graph_publisher.publish(pose_msg);
  }
  pose_array_msg.header.frame_id = "map";
  kf_graph_publisher.publish(pose_array_msg);
  cout<<KFGraph.size()<<" KeyFrames was sent!"<<endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Modified");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    std::string name_of_node = ros::this_node::getName();

    std::string map_frame_id_param = "map";
    std::string camera_frame_id_param;
    ros::Time current_frame_time;

    nh.param<std::string>(name_of_node + "/pointcloud_frame_id", map_frame_id_param, "map");
    nh.param<std::string>(name_of_node + "/camera_frame_id", camera_frame_id_param, "camera_link");
    // Define all subscriers
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::Publisher map_points_publisher;
    ros::Publisher pose_publisher;
    ros::Publisher pose_number_publisher;
    ros::Publisher kf_graph_publisher;
    ros::Publisher covisibility_graph_publisher;

    pose_publisher = nh.advertise<geometry_msgs::PoseStamped> (name_of_node + "/pose", 1);
    kf_graph_publisher = nh.advertise<geometry_msgs::PoseArray> (name_of_node + "/kf_graph", 100);

    ros::Rate loop_rate(30);
    while (ros::ok()) //main loop
    {
        cv::Mat position = SLAM.GetCurrentPosition();
        vector<KeyFrame*> KFGraph = SLAM.GetMap()->GetAllKeyFrames();
        if (!position.empty())
        {
            //PublishPositionAsTransform (position);
            PublishPositionAsPoseStamped(position, pose_publisher, current_frame_time, map_frame_id_param);
        }
        if(KFGraph.size()!=0)
        {
            PublishGraphAsPoseStamped(KFGraph,kf_graph_publisher,current_frame_time,map_frame_id_param);
            cout<<"hello from while :)"<<endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }//end while

    ros::spin();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}

//map -> std::vector<MapPoint*> GetAllMapPoints();
