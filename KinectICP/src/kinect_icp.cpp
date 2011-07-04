#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <KinectICP/pointcloud_registration_point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/SVD>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include "KinectICP/COMMAND.h"
#include <iostream>
#include <vector>
#include <sensor_msgs/Imu.h>
//pcl include
#include "pcl_ros/io/bag_io.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <pcl_ros/point_cloud.h>
#include "pcl/registration/registration.h"
#include "pcl/features/feature.h"
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_registration.h"
#include "pcl/registration/icp_nl.h"
#include <pcl/common/transformation_from_correspondences.h>
#include <Eigen/Core>
#include <math.h>
#include "pcl/filters/statistical_outlier_removal.h" // to filter outliers

#include "pcl/filters/voxel_grid.h" //for downsampling the point cloud

#include "pcl/kdtree/kdtree_flann.h" //for the kdtree
//#include "pcl/octree/octree.h"	//for octree

#include "pcl/registration/transforms.h" //for the transformation function
#include <pcl/features/normal_3d_omp.h>

#include <KinectICP/icp/icp_correspondences_check.h> //for icp
#include <algorithm> //for the sort and unique functions

#include <ctime>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
using namespace std;

class PointCloudRegistration
{
  public:
    PointCloudRegistration();
    ~PointCloudRegistration();
    void pointcloudRegistrationCallBack(const sensor_msgs::PointCloud2& msg);
    void imuCallback (const sensor_msgs::Imu& imuMsg);
    void commandCallback (const lcm_mavlink_ros::COMMAND& commandMsg);
    void viconCallback (const geometry_msgs::PoseStamped& viconMsg);
    void pose2DToTf(const geometry_msgs::Pose2D& pose, btTransform& t);
    void tfToPose2D(const btTransform& t, geometry_msgs::Pose2D& pose);
    Eigen::Matrix4f getOverlapTransformation();
    void publishPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pointcloud);
    pcl::PointCloud<pcl::PointXYZRGB> convertFromMsgToPointCloud(const sensor_msgs::PointCloud2& pcl_msg, double downsample_leafsize);

    void run();

  private:
    ros::NodeHandle nh_;
    std::string merged_pointcloud_topic_, subscribe_pointcloud_topic_, subscribe_imu_topic_, frame_id_;
    int max_number_of_iterations_icp_, max_nn_icp_, max_nn_overlap_;
    double downsample_leafsize_, epsilon_z_, epsilon_curvature_, epsilon_transformation_, radius_icp_, radius_overlap_;
    bool downsample_pointcloud_before_, downsample_pointcloud_after_, filter_outliers_, curvature_check_;
    int scan_index_, counter_;
    clock_t start, end;
    Eigen::Matrix4f final_transformation_, previous_final_transformation_, change_rotation_;
    Eigen::Matrix4f vicontransform;
    ros::Subscriber pointcloud_subscriber_;
    //to fly
    ros::Subscriber imuSubscriber_;
    ros::Subscriber viconSubscriber_;
    ros::Subscriber commandSubscriber_;
    Eigen::Quaternion<float> quat_imu;
    Eigen::Quaternion<float> quat_vicon;
    Eigen::Vector3f pos_vicon;
        btTransform prevWorldToBase_;

    ros::Publisher pointcloud_merged_publisher_;
    ros::Publisher poseStampedtoMAVLINK_pub_;
    geometry_msgs::PoseStamped poseStmpd_mavlink_;
    double ttImage;
    bool take_vicon;
    bool reset_map;
    //pcl::IterativeClosestPointCorrespondencesCheck<pcl::PointXYZRGB, pcl::PointXYZRGB> icp_; // for icp
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_;  // for kdtree

    //pcl::octree::OctreePointCloud<PointXYZRGB> octree_(128.0f); //for octree	float resolution = 128.0f;

    bool firstCloudReceived_, secondCloudReceived_;
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud2_current_, pointcloud2_merged_, pointcloud2_transformed_;
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud2_current_publish_, pointcloud2_merged_publish_, pointcloud2_transformed_publish_;

    btTransform curr_rotation_, prev_rotation_;
    btQuaternion final_rotation_;
    double Roll_ ,Pitch_, Yaw_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void estimateRigidTransformationSVD (const pcl::PointCloud<pcl::PointXYZRGB> &cloud_src, const pcl::PointCloud<pcl::PointXYZRGB> &cloud_tgt, Eigen::Matrix4f &transformation_matrix)
{
         ROS_ASSERT (cloud_src.points.size () == cloud_tgt.points.size ());

        if (cloud_src.points.size () != cloud_tgt.points.size ())
        {
          ROS_ERROR ("[pcl::estimateRigidTransformationSVD] Number or points in source (%zu) differs than target (%zu)!", cloud_src.points.size (), cloud_tgt.points.size ());
          return;
        }

         // <cloud_src,cloud_src> is the source dataset
         transformation_matrix.setIdentity ();

         Eigen::Vector4f centroid_src, centroid_tgt;
         // Estimate the centroids of source, target
         compute3DCentroid (cloud_src, centroid_src);
         compute3DCentroid (cloud_tgt, centroid_tgt);

         // Subtract the centroids from source, target
         Eigen::MatrixXf cloud_src_demean;
         demeanPointCloud (cloud_src, centroid_src, cloud_src_demean);


         Eigen::MatrixXf cloud_tgt_demean;
         demeanPointCloud (cloud_tgt, centroid_tgt, cloud_tgt_demean);

         // Assemble the correlation matrix H = source * target'
         Eigen::Matrix3f H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner<3, 3>();

         // Compute the Singular Value Decomposition
         Eigen::JacobiSVD<Eigen::Matrix3f> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
         Eigen::Matrix3f u = svd.matrixU ();
         Eigen::Matrix3f v = svd.matrixV ();


         Eigen::Matrix3f R = v * u.transpose ();

         // Return the correct transformation
         transformation_matrix.topLeftCorner<3, 3> () = R;
         Eigen::Vector3f Rc = R * centroid_src.head<3> ();
         transformation_matrix.block <3, 1> (0, 3) = centroid_tgt.head<3> () - Rc;

        //return transofrmation_matrix;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f Ransac(const pcl::PointCloud<pcl::PointXYZRGB> & PointCloud1, const pcl::PointCloud<pcl::PointXYZRGB> & PointCloud2, int ransac_iterations)
{
        srand(time(0));
        pcl::TransformationFromCorrespondences tfc;
        Eigen::Matrix4f transformation;
        //transformation.setIdentity();

        float max_error = 0.075;
        int noInliers = 0;

        Eigen::Vector4f surfCloud1, surfCloud2; // used to transform PointCloud coordinates to homogenous coordinates
        pcl::PointCloud<pcl::PointXYZRGB> source, target;

        for (int i = 0; i < ransac_iterations; i++)
        {


                tfc.reset();
                pcl::PointCloud<pcl::PointXYZRGB> source_temp, target_temp;

                int tempNoInliers = 0;
                int sample_size = 3; // chose randomly 3 points from the correspondences
                int ealier_id = 0;

                for (int k = 0; k < sample_size; k++)
                {
                        //cout<<"Test "<<PointCloud1.size()<<endl;

                        int id = rand() % (PointCloud1.size());

                        if((id - ealier_id) == 0)	// checking that the same point is not drawn twice
                                break;

                        Eigen::Vector3f from(PointCloud1.points[id].x,PointCloud1.points[id].y,PointCloud1.points[id].z);
                        Eigen::Vector3f   to(PointCloud2.points[id].x,PointCloud2.points[id].y,PointCloud2.points[id].z);

                        tfc.add(from, to);
                        ealier_id = id;
                }


                Eigen::Matrix4f tempTransformation(tfc.getTransformation().matrix());

                //calculating error and finding inliers
                for(size_t j = 0; j<PointCloud1.size();j++)
                {
                        surfCloud1[0] = PointCloud1.points[j].x;
                        surfCloud1[1] = PointCloud1.points[j].y;
                        surfCloud1[2] = PointCloud1.points[j].z;
                        surfCloud1[3] = 1;
                        surfCloud2[0] = PointCloud2.points[j].x;
                        surfCloud2[1] = PointCloud2.points[j].y;
                        surfCloud2[2] = PointCloud2.points[j].z;
                        surfCloud2[3] = 1;

                        Eigen::Vector4f vec = tempTransformation * surfCloud1 - surfCloud2;
                        double error = vec.dot(vec);

                        if(error<max_error)
                        {
                                tempNoInliers++;
                                source_temp.push_back(PointCloud1.points[j]);
                                target_temp.push_back(PointCloud2.points[j]);
                        }
                }

                // only in case the next iteration brings us more inliers than all the previous ones we update the transformation
                if((tempNoInliers > noInliers))
                {
                        noInliers = tempNoInliers;
                        source = source_temp;
                        target = target_temp;
                }
        }
        estimateRigidTransformationSVD(source, target, transformation);

        return transformation;
}

//--------------------------------------------------------------------------------------------------------------------------------------
Eigen::Matrix4f PointCloudRegistration::getOverlapTransformation()
{
  // In this function we extract the overlapped region of the two points cloud and compute
  // the transformation and return it.

  if(firstCloudReceived_ == false && secondCloudReceived_ == false )
  {
    ROS_ERROR("getOverlapTransformation called before receiving atleast 2 point clouds");
    exit(1);
  }
  else
  {
        int iter = 15;
        Eigen::Matrix4f transformation;
        if(counter_>2)
                transformation = final_transformation_;
        else
                transformation.setIdentity();
        int max_overlap = 1;
        double radius_overlap = 0.1;
        int prev =0, next = 0;
        int count=0;
        for(int i =0; i<=iter;i++)
        {
                pcl::PointCloud<pcl::PointXYZRGB> transformed_PointCloud_current;

                pcl::transformPointCloud(pointcloud2_current_, transformed_PointCloud_current,transformation);

                pcl::PointCloud<pcl::PointXYZRGB> overlap_model, overlap_current;

                int count = 0;
                for(size_t idx = 0 ; (idx < transformed_PointCloud_current.points.size()); idx++ )
                {
                        if((!isnan(transformed_PointCloud_current.points[idx].x)) && (!isnan(transformed_PointCloud_current.points[idx].y)) && (!isnan(transformed_PointCloud_current.points[idx].z)))
                        {
                                std::vector<int> k_indices(max_overlap);
                                std::vector<float> k_distances(max_overlap);
                                kdtree_. nearestKSearch (transformed_PointCloud_current, idx, 1, k_indices, k_distances);
                                if(k_indices.size() > 0)
                                {

                                        if((!isnan(kdtree_.getInputCloud()->points[k_indices[0]].x))
                                        && (!isnan(kdtree_.getInputCloud()->points[k_indices[0]].y))
                                        && (!isnan(kdtree_.getInputCloud()->points[k_indices[0]].z)))
                                        {
                                                overlap_current.points.push_back(transformed_PointCloud_current.points[idx]);
                                                overlap_model.points.push_back (kdtree_.getInputCloud()->points[k_indices[0]]);

                                        }
                                }
                        }
                }

                if(i == iter)
                {
                        cout<<"overlap_current.size "<< overlap_current.size()<<endl;

                }


                Eigen::Matrix4f temp_transformation;

                if(counter_>2)
                {
                        temp_transformation = Ransac(overlap_current, overlap_model, 100);
                }
                else
                        estimateRigidTransformationSVD(overlap_current, overlap_model, temp_transformation);

                //estimateRigidTransformationSVD(overlap_current, overlap_model, temp_transformation);
                transformation = transformation * temp_transformation;
        }
    return (transformation);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudRegistration::publishPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pointcloud)
{
  sensor_msgs::PointCloud2 mycloud;
  pcl::toROSMsg(pointcloud, mycloud);

  if( downsample_pointcloud_after_ == true)
  {
    // for downsampling of pointclouds
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor_;
    sensor_msgs::PointCloud2 mycloud_downsampled;

    //Now we will downsample the point cloud
    sor_.setInputCloud (boost::make_shared<sensor_msgs::PointCloud2> (mycloud));
    sor_.setLeafSize (downsample_leafsize_, downsample_leafsize_, downsample_leafsize_);
    sor_.filter (mycloud_downsampled);
    mycloud_downsampled.header.frame_id = frame_id_;
    mycloud_downsampled.header.stamp = ros::Time::now();

    pointcloud_merged_publisher_.publish(mycloud_downsampled);
  }
  else
  {
    mycloud.header.frame_id = frame_id_;
    mycloud.header.stamp = ros::Time::now();

    pointcloud_merged_publisher_.publish(mycloud);
  }
  ROS_INFO("Merged Point cloud published");
}
/*
//------------------------------------------------------------------------------------------------------------------------------------------//
void PointCloudRegistration::publishPose(const btTransform& transform, const ros::Time& time, const std::string poseFrame_, double height)
{
  geometry_msgs::Pose2D pose;
  tfToPose2D(transform, pose);
  posePublisher_.publish(pose);

  geometry_msgs::PoseStamped poseStmpd;
  geometry_msgs::PoseStamped poseStmpd_mavlink_;

  poseStmpd.header.stamp = time;
  poseStmpd.header.frame_id = poseFrame_;
  poseStmpd.pose.position.x = pose.x;
  poseStmpd.pose.position.y = pose.y;
  poseStmpd.pose.position.z = height;
 //ROS_INFO("Position x: %f",poseStmpd.pose.position.x);
 //ROS_INFO("Position y: %f",poseStmpd.pose.position.y);
 //ROS_INFO("Position z: %f",poseStmpd.pose.position.z);
  btQuaternion rotation;

  rotation.setRPY(0.0, 0.0, pose.theta);
  poseStmpd.pose.orientation.x = rotation.getX();
  poseStmpd.pose.orientation.y = rotation.getY();
  poseStmpd.pose.orientation.z = rotation.getZ();
  poseStmpd.pose.orientation.w = rotation.getW();

  ros::Time time1 = poseStmpd.header.stamp;
  double time1_ =  time1.toSec();
  //ROS_INFO("synchronized, timestamp pose1: %f", time1_);

  pose3DPublisher_.publish(poseStmpd);

  poseStmpd_mavlink_ = poseStmpd;
  poseStmpd_mavlink_.header.frame_id = "world1";
  poseStmpd_mavlink_.header.stamp=ros::Time::now();
  poseStmpd_mavlink_.pose.position.z = -height;
  poseStmpd_mavlink_.pose.position.x = -pose.x;

  rotation.setRPY(0.0, 0.0, M_PI-pose.theta);
  poseStmpd_mavlink_.pose.orientation.x = rotation.getX();
  poseStmpd_mavlink_.pose.orientation.y = rotation.getY();
  poseStmpd_mavlink_.pose.orientation.z = rotation.getZ();
  poseStmpd_mavlink_.pose.orientation.w = rotation.getW();


  poseStampedtoMAVLINK_pub_.publish(poseStmpd_mavlink_);

}
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloudRegistration::PointCloudRegistration(): nh_("~")
{
  nh_.param("publish_merged_pointcloud_topic", merged_pointcloud_topic_, std::string("/merged_pointcloud"));
  nh_.param("subscribe_pointcloud_topic", subscribe_pointcloud_topic_, std::string("/shoulder_cloud"));
  nh_.param("subscribe_imu_topic", subscribe_imu_topic_, std::string("/fromMAVLINK/Imu"));
  nh_.param("max_number_of_iterations_icp", max_number_of_iterations_icp_, 50);
  nh_.param("max_nn_icp", max_nn_icp_, 100);
  nh_.param("max_nn_overlap", max_nn_overlap_, 10);
  nh_.param("radius_icp", radius_icp_, 0.05);
  nh_.param("radius_overlap", radius_overlap_, 0.1);
  nh_.param("curvature_check", curvature_check_, true);
  nh_.param("downsample_pointcloud_before", downsample_pointcloud_before_, true);
  nh_.param("downsample_pointcloud_after", downsample_pointcloud_after_, false);
  nh_.param("filter_outliers", filter_outliers_, true);
  nh_.param("downsample_leafsize", downsample_leafsize_, 0.1);
  nh_.param("epsilon_z", epsilon_z_, 0.001);
  nh_.param("epsilon_curvature", epsilon_curvature_, 0.001);
  nh_.param("epsilon_transformation", epsilon_transformation_, 1e-6);
  firstCloudReceived_ = false;
  secondCloudReceived_ = false;
  scan_index_ = 0;
  counter_ = 0;
  take_vicon = false;
  //icp_.setMaximumIterations(max_number_of_iterations_icp_);
 // icp_.setTransformationEpsilon(epsilon_transformation_);
 // icp_.setParameters(radius_icp_, max_nn_icp_, epsilon_z_, epsilon_curvature_, curvature_check_ );
  ROS_INFO("pointcloud_registration node is up and running.");

  run();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void downSample(const pcl::PointCloud<pcl::PointXYZRGB>& src, pcl::PointCloud<pcl::PointXYZRGB>& to)
{
        pcl::VoxelGrid<pcl::PointXYZRGB> down_sampler;
        down_sampler.setLeafSize (0.04, 0.04, 0.04);
        pcl::PCLBase<pcl::PointXYZRGB>::PointCloudConstPtr const_cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> > (src);
        down_sampler.setInputCloud (const_cloud_ptr);
        down_sampler.filter(to);
        ROS_INFO("gicp.cpp: Downsampling from %i to %i", (int) src.points.size(), (int) to.points.size());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZRGB> PointCloudRegistration::convertFromMsgToPointCloud(const sensor_msgs::PointCloud2& pcl_msg, double downsample_leafsize)
{
        //incrementing the scan index
        //scan_index_++;
        // Declaring some variables required in this function
        //sensor_msgs::PointCloud2 pcl_msg;
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl_step01, pointcloud_pcl_step02;
        pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl_normals;
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
        pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtree;
        kdtree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();
        std::vector<int> indices;

        // STEP 01: Check if we should downsample the input cloud or not

        //ROS_INFO ("PointCloud before downsampling: %d .", pcl_msg.width * pcl_msg.height);
        sensor_msgs::PointCloud2 pointcloud_downsampled_msg;
        pcl::VoxelGrid<sensor_msgs::PointCloud2> grid; // for downsampling of pointclouds

        //Now we will downsample the point cloud
        grid.setInputCloud (boost::make_shared<sensor_msgs::PointCloud2> (pcl_msg));
        grid.setLeafSize (downsample_leafsize, downsample_leafsize, downsample_leafsize);
        grid.filter (pointcloud_downsampled_msg);
        ROS_INFO ("PointCloud after downsampling: %d .", pointcloud_downsampled_msg.width * pointcloud_downsampled_msg.height);

        // Converting from PointCloud2 msg format to pcl pointcloud format
        pcl::fromROSMsg(pointcloud_downsampled_msg, pointcloud_pcl_step01);



        // STEP 02: Check if we should filter the outliers or not


        // Removing outliers
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(pointcloud_pcl_step01));
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (pointcloud_pcl_step02);

        kdtree->setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> > (pointcloud_pcl_step02));

        return (pointcloud_pcl_step02);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloudRegistration::~PointCloudRegistration()
{
  ROS_INFO("Shutting down pointcloud_registration node!.");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudRegistration::run()
{
        //start measure time



  //to fly
  imuSubscriber_ = nh_.subscribe ("/fromMAVLINK/Imu",  10, &PointCloudRegistration::imuCallback,  this);
  pointcloud_subscriber_ = nh_.subscribe("/camera/rgb/points", 10, &PointCloudRegistration::pointcloudRegistrationCallBack, this);
  viconSubscriber_= nh_.subscribe("/fromMAVLINK/Vicon",10,&PointCloudRegistration::viconCallback,this);
  commandSubscriber_= nh_.subscribe("/fromMAVLINK/COMMAND",10,&PointCloudRegistration::commandCallback,this);

  pointcloud_merged_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/merged_pointcloud", 10);
  poseStampedtoMAVLINK_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/toMAVLINK/bodyPoseStamped",10);
  ros::spin();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudRegistration::imuCallback (const sensor_msgs::Imu& imuMsg)
{

        btQuaternion q(imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w);
        btMatrix3x3 m(q);
        double Raw, Pitch, Yaw;
        m.getRPY(Raw, Pitch, Yaw);
        cout<<"Raw "<<Raw<<" Pitch "<<Pitch<<" Yaw "<< Yaw<<endl;
        btQuaternion q2;
        q2.setRPY(Raw, Pitch, Yaw);
        curr_rotation_.setRotation(q2);				//no information about Yaw rotation

        btTransform change_rot = prev_rotation_.inverse()*curr_rotation_;					//curr_rotation_*prev_rotation_.inverse();
        btQuaternion temp_rot(change_rot.getRotation());

        btMatrix3x3 m2(temp_rot);
        double Raw2, Pitch2, Yaw2;
        m2.getRPY(Raw2, Pitch2, Yaw2);
        //1st
        change_rotation_ << 	sin(Yaw2)*sin(Pitch2)*sin(Raw2)+cos(Yaw2)*cos(Raw2),		sin(Yaw2)*sin(Pitch2)*cos(Raw2)-cos(Yaw2)*sin(Raw2),		sin(Yaw2)*cos(Pitch),		0,
                                cos(Pitch2)*sin(Raw2),						cos(Pitch2)*cos(Raw2),						-sin(Pitch2),			0,
                                cos(Yaw2)*sin(Pitch2)*sin(Raw2)-sin(Yaw2)*cos(Raw2),		sin(Pitch2)*cos(Raw2),						cos(Pitch2),			0,
                                0,								0,								0,				1;


        //2nd
/*
        change_rotation_ << 	cos(Pitch2),	sin(Pitch2)*sin(Raw2),	sin(Pitch2)*cos(Raw2),	0,
                                0,		cos(Raw2),		-sin(Raw2),		0,
                                -sin(Pitch2),	cos(Pitch2)*sin(Raw2),	cos(Pitch2)*cos(Raw2),	0,
                                0,		0,			0,			1;


        cout<<"change_rotation "<<counter_<<endl;
        cout<<change_rotation_<<endl;
        cout<<"Raw "<<Raw2<<" Pitch "<<Pitch2<<" Yaw "<< Yaw2<<endl;
*/
        prev_rotation_ = curr_rotation_;

}
//**************************************************************************************************************************pose2DtoTf()
void PointCloudRegistration::pose2DToTf(const geometry_msgs::Pose2D& pose, btTransform& t)
{
  t.setOrigin(btVector3(pose.x, pose.y, 0.0));
  btQuaternion q;
  q.setRPY(0, 0, pose.theta);
  t.setRotation(q);
}

//**************************************************************************************************************************tfToPose2D()
void PointCloudRegistration::tfToPose2D(const btTransform& t, geometry_msgs::Pose2D& pose)
{
  btMatrix3x3 m(t.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  pose.x = t.getOrigin().getX();
  pose.y = t.getOrigin().getY();
  pose.theta = yaw;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////viconCallback()
void PointCloudRegistration::viconCallback (const geometry_msgs::PoseStamped& viconMsg)
{
        std::cout<<"in viconcallback"<<std::endl;
        quat_vicon.x()=viconMsg.pose.orientation.x;
        quat_vicon.y()=viconMsg.pose.orientation.y;
        quat_vicon.z()=viconMsg.pose.orientation.z;
        quat_vicon.w()=viconMsg.pose.orientation.w;

        pos_vicon[0]=viconMsg.pose.position.x;
        pos_vicon[1]=viconMsg.pose.position.y;
        pos_vicon[2]=viconMsg.pose.position.z;

        Eigen::Quaternion<float> quat_vicon_eigen;
        quat_vicon_eigen.x()=-quat_vicon.y();
        quat_vicon_eigen.y()=-quat_vicon.z();
        quat_vicon_eigen.z()=quat_vicon.x();
        quat_vicon_eigen.w()=quat_vicon.w();


        btQuaternion quat_tmp(quat_vicon.y(),quat_vicon.z(),quat_vicon.x(),quat_vicon.w());

        btMatrix3x3 m(quat_tmp);

        std::cout<<"m vicon:"<<m.getRow(0)[0]<<" "<<m.getRow(0)[1]<<" "<<m.getRow(0)[2]<<std::endl
                        <<" "<<m.getRow(1)[0]<<" "<<m.getRow(1)[1]<<" "<<m.getRow(1)[2]<<std::endl
                        <<" "<<m.getRow(2)[0]<<" "<<m.getRow(2)[1]<<" "<<m.getRow(2)[2]<<std::endl;

        vicontransform=Eigen::Matrix4f::Identity();
        //		Eigen::Matrix4f vicontransform;

        Eigen::Vector3f tmp_vec;

        tmp_vec[0]=m.getColumn(0)[0];
        tmp_vec[1]=m.getColumn(0)[1];
        tmp_vec[2]=m.getColumn(0)[2];

        vicontransform.block<3,1>(0,0)=tmp_vec;

        tmp_vec[0]=m.getColumn(1)[0];
        tmp_vec[1]=m.getColumn(1)[1];
        tmp_vec[2]=m.getColumn(1)[2];

        vicontransform.block<3,1>(0,1)=tmp_vec;

        tmp_vec[0]=m.getColumn(2)[0];
        tmp_vec[1]=m.getColumn(2)[1];
        tmp_vec[2]=m.getColumn(2)[2];

        vicontransform.block<3,1>(0,2)=tmp_vec;

        tmp_vec[0]=pos_vicon[1];
        tmp_vec[1]=pos_vicon[2];
        tmp_vec[2]=pos_vicon[0];

        vicontransform.block<3,1>(0,3)=tmp_vec;
/*	if(take_vicon==true)
        {
                cout<<"vicontransform "<<endl;
                cout<<vicontransform<<endl;
        }
*/
}

////////////////////////////////////////////////////////////////////////////////////////////////////////comandCallback()
void PointCloudRegistration::commandCallback (const lcm_mavlink_ros::COMMAND& commandMsg)
{
        ROS_INFO("in commandcallback");

        if(commandMsg.command==200)
                take_vicon=true;
        if(commandMsg.command==201)
                reset_map=true;
}

//-------------------------------------------------------------------------------------------------------------------------------//
void PointCloudRegistration::pointcloudRegistrationCallBack(const sensor_msgs::PointCloud2& pointcloud_msg)
{


        counter_++;
        frame_id_ = pointcloud_msg.header.frame_id;

        if( firstCloudReceived_ == false)
        {
                pointcloud2_current_ = convertFromMsgToPointCloud(pointcloud_msg,0.1);
                //pointcloud2_current_publish_ = convertFromMsgToPointCloud(pointcloud_msg,0.01);
                ROS_INFO("Size of point cloud received = %d", (int) pointcloud2_current_.points.size());
                firstCloudReceived_ = true;
                ROS_INFO("Received first point cloud.");
                kdtree_.setInputCloud(boost::make_shared< pcl::PointCloud < pcl::PointXYZRGB> > (pointcloud2_current_));
                //octree_.addPointsFromInputCloud ();
                //pcl::fromROSMsg(pointcloud_msg, pointcloud2_current_publish_);

                pointcloud2_merged_ = pointcloud2_current_;
                //pointcloud2_merged_publish_ = pointcloud2_current_publish_;
                //pointcloud2_transformed_publish_ = pointcloud2_current_publish_;
                //publishPointCloud(pointcloud2_merged_publish_);
        }
        else if( secondCloudReceived_ == false)
        {
                ROS_INFO("Received second point cloud.");
                secondCloudReceived_ = true;
                pointcloud2_current_ = convertFromMsgToPointCloud(pointcloud_msg,0.1);
                //pointcloud2_current_publish_ = convertFromMsgToPointCloud(pointcloud_msg,0.01);

                //Now we get the transformation from the overlapped regions of the 2 point clouds
                final_transformation_= getOverlapTransformation();
                // pcl::fromROSMsg(pointcloud_msg, pointcloud2_current_publish_);
                //end = clock();

                pcl::transformPointCloud(pointcloud2_current_, pointcloud2_transformed_, final_transformation_);
                pointcloud2_merged_ += pointcloud2_transformed_;


                //pcl::transformPointCloud(pointcloud2_current_publish_, pointcloud2_transformed_publish_, final_transformation_);
                //pointcloud2_merged_publish_ += pointcloud2_transformed_publish_;

                //publishPointCloud(pointcloud2_merged_publish_);
                //publishPointCloud(pointcloud2_transformed_publish_);

        }
        else
        {
                if(take_vicon == true)
                {
                          final_transformation_ = vicontransform;
                          take_vicon=false;
                }
                //if((counter_==1)||(counter_%1 == 0))
                //{

                        ROS_INFO("Received point cloud number: %d", counter_);
                        pointcloud2_current_ = convertFromMsgToPointCloud(pointcloud_msg,0.2);

                        kdtree_.setInputCloud(boost::make_shared< pcl::PointCloud < pcl::PointXYZRGB> > (pointcloud2_merged_));
                        //odtree_.addPointsFromInputCloud ();

                        //Now we get the transformation from the overlapped regions of the 2 point clouds

                //	cout<<"1st final_transformation_ "<<endl;
                //	cout<<final_transformation_<<endl;
                //	Eigen::Vector4d translation;
                //	translation << final_transformation_(0,3), final_transformation_(1,3), final_transformation_(2,3), final_transformation_(3,3);

                        final_transformation_ = final_transformation_*change_rotation_;			//			change_rotation_*final_transformation_;
                        ttImage = 0.;
                        ttImage = (double)cvGetTickCount();
                        final_transformation_= getOverlapTransformation();
                        //stop measuring time
                        ttImage = (double)cvGetTickCount() - ttImage;
                        //print in milliseconds
                        double s = 1./(cvGetTickFrequency()*1000.);
                        ROS_INFO("pointcloud callback time:\t%6.1f ms\n", ttImage*s);

                        cout<<final_transformation_<<endl;
                        Eigen::Matrix3f	rot;
                        //rot = final_transformation_.block(0,0,2,2);

                        Eigen::Matrix4f base_to_cam;
                        base_to_cam <<  0,	1,	0,	0,
                                        0,	0,	1,	0,
                                        1,	0,	0,	0,
                                        0,	0,	0,	1;
                        Eigen::Matrix4f cam_to_world;
                        cam_to_world = final_transformation_;
                        Eigen::Matrix4f base_to_world;
                        rot = (base_to_cam.block(0,0,3,3)).inverse()*(cam_to_world.block(0,0,3,3))*(base_to_cam.block(0,0,3,3));
                        //cout<<rot<<endl;
                        base_to_world = final_transformation_;			//(cam_to_world*base_to_cam).inverse();	//(final_transformation_.inverse())*(temp.inverse());
                        base_to_world.block(0,0,3,3) = rot;
                        /*
                                base_to_cam.inverse()*cam_to_world   --> roll-pitch invert
                                cam_to_world*base_to_cam ----> wrong
                        */
                        Yaw_ = atan2(base_to_world(1,0),base_to_world(0,0));
                        Pitch_ = atan2(-base_to_world(2,0), sqrt(base_to_world(2,1)*base_to_world(2,1)+base_to_world(2,2)*base_to_world(2,2)));
                        Roll_ = atan2(base_to_world(2,1),base_to_world(2,2));
                        //poseStmpd_mavlink_ = poseStmpd;
                          poseStmpd_mavlink_.header.frame_id = "world1";
                          poseStmpd_mavlink_.header.stamp=ros::Time::now();
                          poseStmpd_mavlink_.pose.position.x = base_to_world(0,3);
                          poseStmpd_mavlink_.pose.position.y = base_to_world(1,3);
                          poseStmpd_mavlink_.pose.position.z = base_to_world(2,3);

                          final_rotation_.setRPY(Roll_, Pitch_, Yaw_);
                          poseStmpd_mavlink_.pose.orientation.x = final_rotation_.getX();
                          poseStmpd_mavlink_.pose.orientation.y = final_rotation_.getY();
                          poseStmpd_mavlink_.pose.orientation.z = final_rotation_.getZ();
                          poseStmpd_mavlink_.pose.orientation.w = final_rotation_.getW();

                        //cout<<"Roll: "<<Roll_<<", Pitch: "<<Pitch_<<" Yaw: "<<Yaw_<<endl;

                          poseStampedtoMAVLINK_pub_.publish(poseStmpd_mavlink_);
                /*	final_transformation_(0,3) = translation(0);
                        final_transformation_(1,3) = translation(1);
                        final_transformation_(2,3) = translation(2);
                        final_transformation_(3,3) = translation(3);
                */
                //	cout<<"middle final_transformation_ "<<endl;
                //	cout<<final_transformation_<<endl;

                        final_transformation_= getOverlapTransformation();
                //	cout<<"2nd final_transformation_ "<<endl;
                //	cout<<final_transformation_<<endl;




                        //if((counter_%10 == 0))
                        //{
                                pcl::transformPointCloud(pointcloud2_current_, pointcloud2_transformed_, final_transformation_);
                                pointcloud2_merged_ += pointcloud2_transformed_;
                                publishPointCloud(pointcloud2_transformed_);
                        //}
/*
                        ttImage = 0.;
                        ttImage = (double)cvGetTickCount();
                        pointcloud2_current_publish_ = convertFromMsgToPointCloud(pointcloud_msg,0.05);
                        //stop measuring time
                        ttImage = (double)cvGetTickCount() - ttImage;
                        //print in milliseconds
                        double s = 1./(cvGetTickFrequency()*1000.);

                        ROS_INFO("publish time:\t%6.1f ms\n", ttImage*s);
                        pcl::transformPointCloud(pointcloud2_current_publish_, pointcloud2_transformed_publish_, final_transformation_);
                        //pointcloud2_merged_publish_ += pointcloud2_transformed_publish_;
                        //publishPointCloud(pointcloud2_merged_publish_);
                        publishPointCloud(pointcloud2_transformed_publish_);
*/



                //Downsample if pointcloud2_merged_ > 50,000 or pointcloud2_merged_publish_.size()>500000
/*		if(pointcloud2_merged_.size()>1000000000)
                {
                        pcl::PointCloud<pcl::PointXYZRGB>& pointcloud2_merged_1 = pointcloud2_merged_, pointcloud2_merged_2 = pointcloud2_merged_;
                        downSample(pointcloud2_merged_1, pointcloud2_merged_2);
                        pointcloud2_merged_ = pointcloud2_merged_2;
                }
                if(pointcloud2_merged_publish_.size()>100000000000)
                {
                        pcl::PointCloud<pcl::PointXYZRGB>& pointcloud2_merged_publish_1 = pointcloud2_merged_publish_, pointcloud2_merged_publish_2 = pointcloud2_merged_publish_;
                        downSample(pointcloud2_merged_publish_1, pointcloud2_merged_publish_2);
                        pointcloud2_merged_publish_ = pointcloud2_merged_publish_2;
                }
*/
        }

        previous_final_transformation_ = final_transformation_;
}


//------------------------------------------------------------------------------------------------------------------//
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_registration");
    PointCloudRegistration pointcloud_registration;
    return(0);
}
