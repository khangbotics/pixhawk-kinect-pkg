#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include "pcl/io/io.h"
#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <vector>
#include <sys/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include "/opt/ros/diamondback/stacks/common_msgs/visualization_msgs/msg_gen/cpp/include/visualization_msgs/Marker.h"
#include "/home/pixhawk/pixhawk/mavlink-ros-pkg/lcm_mavlink_ros/msg_gen/cpp/include/lcm_mavlink_ros/COMMAND.h"
#include "psm_pose2D.h"

#include <csm/csm_all.h>

const std::string imuTopic_  = "imu";
const std::string scanTopic_ = "scan";
const std::string poseTopic_ = "pose2D";
const std::string pose3DTopic_ = "pose3D";

const double ROS_TO_PM = 100.0;   // convert from cm to m

  struct parameters{
  double slope;
  double y_origin;
  double error;
  };

class PSMpositionNode{

private:

  //to fly
  ros::Subscriber imuSubscriber;
  ros::Subscriber viconSubscriber;
  ros::Subscriber commandSubscriber;

  Eigen::Quaternion<float> quat_imu;
  Eigen::Quaternion<float> quat_vicon;
  Eigen::Vector3f pos_vicon;

  bool notcopied;
  bool take_vicon;
  bool reset_map;
  Eigen::Quaternion<float> quat_rot_heli;
  Eigen::Quaternion<float> quat_rot_keyframe;
	Eigen::Matrix4f imuRot;
  //

  sm_params input_;
  sm_result output_;
  LDP prevLDPScan_;
  bool initialized_can;

 PolarMatcher matcher_;
 PMScan * prevPMScan_;
int contador;
int width_;
int height_;
int begin_;
 bool init_;
 double totalDuration_;
 int pclCount_;

 bool CamParam_init ;
 sensor_msgs::LaserScan hline;
 sensor_msgs::LaserScan vline;

 double fl;
 double min_angle_h;
 double max_angle_h;
 double inc_angle_h;
 double min_angle_v;
 double max_angle_v;
 double inc_angle_v;
ros::Publisher marker_pub; 
 ros::Subscriber sub;
 ros::Subscriber subInfo;
 ros::Publisher laserScan_pub;
 pcl::PointCloud<pcl::PointXYZ> cloud;


 ros::Subscriber scanSubscriber_;
 ros::Subscriber imuSubscriber_;
 ros::Publisher  posePublisher_;
 ros::Publisher  pose3DPublisher_;
 ros::Publisher poseStampedtoMAVLINK_pub;

 tf::TransformBroadcaster tfBroadcaster_;
 tf::TransformListener    tfListener_;
 btTransform prevWorldToBase_;
 btTransform baseToLaser_;
 btTransform laserToBase_;

visualization_msgs::Marker points, line_strip, arrow;

//for Ransac
  int iterations;
  double m,y0;
  double max_slope;
  double min_inl_dist;
  double best_error;
  int minNumInliers;
  double m_abs;
  
  double a, b, c;
  int tempNoInliers, rpoint_1, rpoint_2;
  double dist2line;
 
  double bestLine[2];
  double ransacHeight;
double previousHeight;
int smMethod;

  struct depthPoint{
  double x;
  double y;
  };

 double heightLine;

 depthPoint XYpoint;

 parameters modelParameters;
 

//

 bool initialized_;


 boost::mutex imuMutex_;
 double prevImuAngle_;   // the yaw angle when we last perfomred a scan match
 double currImuAngle_;   // the most recent yaw angle we have received

 // **** parameters

 bool   publishTf_;
 bool   publishPose_;
 bool   useTfOdometry_;
 bool   useImuOdometry_;

 int    minValidPoints_;
 int    searchWindow_;
 double maxError_;
 int    maxIterations_;
 double stopCondition_;
 int scanmatchMethod;
 std::string frameP_;
 std::string worldFrame_;
 std::string baseFrame_;
 std::string laserFrame_;
 
 bool initializeLasermsgs(const sensor_msgs::PointCloud2& pcloud);
 void getInfo(const sensor_msgs::CameraInfo& caminfo);
 void pointCloudcallback(const sensor_msgs::PointCloud2& pcloud);
 void getParams();
 bool initialize(const sensor_msgs::LaserScan& scan);

 void publishTf(const btTransform& transform, 
                   const ros::Time& time);
 void publishPose(const btTransform& transform, const ros::Time& time, std::string poseFrame_, double height);

 void rosToPMScan(const sensor_msgs::LaserScan& scan, 
                     const btTransform& change,
                           PMScan* pmScan);
 void pose2DToTf(const geometry_msgs::Pose2D& pose, btTransform& t);
 void tfToPose2D(const btTransform& t, geometry_msgs::Pose2D& pose);
 void getCurrentEstimatedPose(btTransform& worldToBase, 
                                 const sensor_msgs::LaserScan& scanMsg);


 void getMotion(const sensor_msgs::LaserScan& scan,  std::vector <PSMpositionNode::depthPoint> *depthLine);
 parameters fitline(std::vector<PSMpositionNode::depthPoint> *);

 double ransac(std::vector <depthPoint> *);
 void getMotion_can(const sensor_msgs::LaserScan& scan,  std::vector <depthPoint> *depthLine);
 LDP rosToLDPScan(const sensor_msgs::LaserScan& scan,
                     const geometry_msgs::Pose2D& laserPose);
 bool initializeCan(const sensor_msgs::LaserScan& scan);

 void imuCallback(const sensor_msgs::Imu& imuMsg);
 void commandCallback (const lcm_mavlink_ros::COMMAND& commandMsg);
 void viconCallback (const geometry_msgs::PoseStamped& viconMsg);

public:

PSMpositionNode();
virtual ~PSMpositionNode();

};
