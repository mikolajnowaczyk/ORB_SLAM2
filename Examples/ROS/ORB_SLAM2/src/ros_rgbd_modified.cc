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
#include<string>

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
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

using namespace std;
using namespace ORB_SLAM2;

bool compareKFs(KeyFrame* KF1, KeyFrame* KF2)
{
    return (KF1->mnId < KF2->mnId);
}

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

bool matIsEqual(const cv::Mat mat1, const cv::Mat mat2){
   // treat two empty mat as identical as well
   if (mat1.empty() && mat2.empty()) {
       return true;
   }
   // if dimensionality of two mat is not identical, these two mat is not identical
   if (mat1.cols != mat2.cols || mat1.rows != mat2.rows || mat1.dims != mat2.dims) {
       return false;
   }
   cv::Mat diff;
   cv::compare(mat1, mat2, diff, cv::CMP_NE);
   int nz = cv::countNonZero(diff);
   return nz==0;
}

tf::Transform TransformFromMat(cv::Mat position_mat)
{
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
  /*const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);*/

  const tf::Matrix3x3 tf_orb_to_ros (1, 0, 0,
                                     0, 1, 0,
                                     0, 0, 1);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
//  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
//  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
//  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
//  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}

void PublishPositionAsTransform(cv::Mat position, std::string map_frame_id_param, std::string camera_frame_id_param, ros::Time current_frame_time)
{
  tf::Transform transform = TransformFromMat (position);
  static tf::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, current_frame_time, map_frame_id_param, camera_frame_id_param));
}

void PublishPositionAsPoseStamped (cv::Mat position, ros::Publisher pose_publisher,ros::Time current_frame_time, std::string map_frame_id_param)
{
  tf::Transform grasp_tf = TransformFromMat(position);
  tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time, map_frame_id_param);//, current_frame_time, map_frame_id_param);
  geometry_msgs::PoseStamped pose_msg;
  tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg);
  pose_publisher.publish(pose_msg);
}

void PublishGraphAsPoseStamped(vector<KeyFrame*> KFGraph, ros::Publisher kf_graph_publisher, ros::Time current_frame_time, std::string map_frame_id_param)
{
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
  //cout<<KFGraph.size()<<" KeyFrames was sent!"<<endl;
}

void PublishGraphAsPosesStamped(vector<KeyFrame*> KFGraph, ros::Publisher kf_graph_publisher,ros::Publisher kf_graph_indexes_publisher , ros::Time current_frame_time, std::string map_frame_id_param)
{
  geometry_msgs::PoseStamped pose_stamped_msg;
  geometry_msgs::Pose pose_msg;
  cv::Mat position;
  tf::Transform grasp_tf;
  std_msgs::Int32MultiArray indexes;
  for(unsigned int i = 0; i < KFGraph.size(); i++)
  {
      position = KFGraph[i]->GetPose();
      grasp_tf = TransformFromMat(position);
      tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time, map_frame_id_param);
      tf::poseStampedTFToMsg (grasp_tf_pose, pose_stamped_msg);
      //pose_msg = pose_stamped_msg.pose;
      pose_stamped_msg.header.frame_id = to_string(KFGraph[i]->mnId);
      kf_graph_publisher.publish(pose_stamped_msg);
      indexes.data.push_back(KFGraph[i]->mnId);
  }
  kf_graph_indexes_publisher.publish(indexes);
}

sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points, std::string map_frame_id_param, int min_observations_per_point)
{
  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  //cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++)
  {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

        unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i=0; i<cloud.width; i++)
  {
    if (map_points.at(i)->nObs >= min_observations_per_point)
    {
      data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
    }
  }

  return cloud;
}

void PublishMapPoints(std::vector<ORB_SLAM2::MapPoint*> map_points, ros::Publisher map_points_publisher, std::string map_frame_id_param, int min_observations_per_point)
{
  sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud(map_points, map_frame_id_param, min_observations_per_point);
  map_points_publisher.publish(cloud);
}

std_msgs::Int32MultiArray PublishCovisibility(vector<KeyFrame*> KFGraph, ros::Publisher covisibility_publisher)
{
  std_msgs::Int32MultiArray covisibility_msg;
  for(unsigned int i = 0; i < KFGraph.size(); i++)
  {
     for(unsigned int j = i+1; j < KFGraph.size(); j++)
     {
         covisibility_msg.data.push_back(i);
         covisibility_msg.data.push_back(j);
         covisibility_msg.data.push_back(KFGraph[i]->GetWeight(KFGraph[j]));
     }
  }
  cout<<"PublishCovisibility: VectorSize() = "<< covisibility_msg.data.size()<<endl;
  covisibility_publisher.publish(covisibility_msg);
  return covisibility_msg;
}

void PublishPoseNumber(vector<KeyFrame*> KFGraph, KeyFrame* ReferenceKF, ros::Publisher pose_number_publisher)
{
  std_msgs::Int32 number_msg;
  //int number;
  for(unsigned int i = 0; i < KFGraph.size(); i++)
  {
    if(matIsEqual(KFGraph[i]->GetPose(),ReferenceKF->GetPose()))
    {
        number_msg.data = i;
        break;
    }
  }
  cout<<"PublishPoseNumber function debug:"<<endl;
  for(unsigned int i = 0; i < KFGraph.size(); i++)
  {
    cout<<i<<" "<<KFGraph[i]->mnId <<" and pointer: "<<&KFGraph[i]<<endl;
  }
  cout<<endl;

  cout<<"Current number: "<<number_msg.data<<" of total "<<KFGraph.size()<<endl;
  pose_number_publisher.publish(number_msg);
}

void PublishGraphVisualization(vector<KeyFrame*> KFGraph, std_msgs::Int32MultiArray covisibility_msg, ros::Publisher visualization_publisher)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    //marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.0002;
    marker.scale.y = 0.0002;
    marker.scale.z = 0.0002;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

    geometry_msgs::PoseStamped pose_stamped_msg;
    cv::Mat position;
    tf::Transform grasp_tf;
    ros::Time current_frame_time;
    std::string map_frame_id_param = "map";
    //points from graph
    for(unsigned int i = 0; i < (covisibility_msg.data.size()/3); i++)
    {
        if(covisibility_msg.data[3*i+2] >= 10)
        {
            position = KFGraph[covisibility_msg.data[3*i+0]]->GetPose();
            grasp_tf = TransformFromMat(position);
            tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time, map_frame_id_param);
            tf::poseStampedTFToMsg (grasp_tf_pose, pose_stamped_msg);

            geometry_msgs::Point p;
            p.x = pose_stamped_msg.pose.position.x;
            p.y = pose_stamped_msg.pose.position.y;
            p.z = pose_stamped_msg.pose.position.z;

            marker.points.push_back(p);

            position = KFGraph[covisibility_msg.data[3*i+1]]->GetPose();
            grasp_tf = TransformFromMat(position);
            tf::Stamped<tf::Pose> grasp_tf_pose2(grasp_tf, current_frame_time, map_frame_id_param);
            tf::poseStampedTFToMsg (grasp_tf_pose2, pose_stamped_msg);

            p.x = pose_stamped_msg.pose.position.x;
            p.y = pose_stamped_msg.pose.position.y;
            p.z = pose_stamped_msg.pose.position.z;

            marker.points.push_back(p);
        }
    }
    visualization_publisher.publish( marker );
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
    std::string camera_frame_id_param = "camera_link";
    int min_observations_per_point = 2;

    nh.param<std::string>(name_of_node + "/pointcloud_frame_id", map_frame_id_param, "map");
    nh.param<std::string>(name_of_node + "/camera_frame_id", camera_frame_id_param, "camera_link");

    // Define all subscriers
//    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
//    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);
    //new camera - astra
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);

    //bagfile
//    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
//    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::Publisher map_points_publisher;
    ros::Publisher pose_publisher;
    ros::Publisher pose_number_publisher;
    ros::Publisher kf_graph_publisher;
    ros::Publisher kf_graph_indexes_publisher;
    ros::Publisher covisibility_publisher;
    ros::Publisher visualization_publisher;
    ros::Publisher kf_graph_publisher_pose_stamped;

    pose_publisher = nh.advertise<geometry_msgs::PoseStamped> (name_of_node + "/pose", 1);
    kf_graph_publisher = nh.advertise<geometry_msgs::PoseArray> (name_of_node + "/kf_graph", 1);
    kf_graph_publisher_pose_stamped = nh.advertise<geometry_msgs::PoseStamped> (name_of_node + "/kf_graph_pose_stamped", 10000);
    map_points_publisher = nh.advertise<sensor_msgs::PointCloud2> (name_of_node + "/point_cloud", 1);
    covisibility_publisher = nh.advertise<std_msgs::Int32MultiArray> (name_of_node + "/covisibility", 1);
    kf_graph_indexes_publisher = nh.advertise<std_msgs::Int32MultiArray> (name_of_node + "/kf_graph_indexes", 1);
    pose_number_publisher = nh.advertise<std_msgs::Int32> (name_of_node + "/pose_number", 1);
    visualization_publisher = nh.advertise<visualization_msgs::Marker> (name_of_node + "/visualization", 1);
    ros::Time current_frame_time;

    ros::Rate loop_rate(20);
    std_msgs::Int32MultiArray covisibility_msg;
    while (ros::ok()) //main loop
    {
        cv::Mat position = SLAM.GetCurrentPosition();
        vector<KeyFrame*> KFGraph = SLAM.GetMap()->GetAllKeyFrames();
        sort(KFGraph.begin(),KFGraph.end(),compareKFs);
        vector<MapPoint*> MapPoints = SLAM.GetMap()->GetAllMapPoints();
        current_frame_time = ros::Time::now();
        //POSE PUBLISH
        if(!position.empty())
        {
            //PublishPositionAsTransform (position);
            PublishPositionAsPoseStamped(position, pose_publisher, current_frame_time, map_frame_id_param);
        }
        //GRAPH PUBLISH
        if(!KFGraph.empty())
        {
            PublishGraphAsPoseStamped(KFGraph,kf_graph_publisher,current_frame_time,map_frame_id_param);
            //cout<<"hello from while :)"<<endl;
        }
        //GRAPH POSE STAMPED PUBLISH
        if(!KFGraph.empty())
        {
            PublishGraphAsPosesStamped(KFGraph,kf_graph_publisher_pose_stamped,kf_graph_indexes_publisher,current_frame_time,map_frame_id_param);
            //cout<<"hello from while :)"<<endl;
        }
        //POINTCLOUD PUBLISH
        if(!MapPoints.empty())
        {
            PublishMapPoints(MapPoints, map_points_publisher,map_frame_id_param, min_observations_per_point);
        }
        //COVISIBILITY PUBLISH
        if(!KFGraph.empty())
        {
            covisibility_msg = PublishCovisibility(KFGraph, covisibility_publisher);
        }
        //POSE NUMBER PUBLISH
        if(!KFGraph.empty())
        {
            PublishPoseNumber(KFGraph, SLAM.GetTracking()->GetReferenceKF(), pose_number_publisher);
        }

        //Visualization publish
        if(!KFGraph.empty() and !covisibility_msg.data.empty())
        {
            PublishGraphVisualization(KFGraph, covisibility_msg, visualization_publisher);
        }

        //TF publishing
        if(!position.empty())
        {
            PublishPositionAsTransform(position,map_frame_id_param,camera_frame_id_param,current_frame_time);
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
