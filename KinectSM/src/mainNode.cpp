//Main.cpp
// Here all the files will be collapsed into one main file

#include "mainNode.h"



int main(int argc, char **argv)
{

  ros::init(argc, argv, "getHorizontalline");
  PSMpositionNode psmpositionNode;
  ros::spin();
  
}

PSMpositionNode::PSMpositionNode()
{
  ROS_INFO("Creating Localization node");

  ros::NodeHandle n;
/*
#ifdef USE_PROJECTED_SCANS
  ROS_INFO("Creating CanonicalScanMatcher node [Projected scans]");
#else
  ROS_INFO("Creating CanonicalScanMatcher node");
#endif*/

initialized_can = false;

  init_   = false;
  initialized_ = false;
  CamParam_init = false;
  previousHeight = 0.0;
  totalDuration_ = 0.0;
  pclCount_    = 0;
  smMethod = 1;
  prevWorldToBase_.setIdentity();
  getParams();  

  sub = n.subscribe("camera/rgb/points", 100, &PSMpositionNode::pointCloudcallback, this);
  subInfo = n.subscribe("camera/depth/camera_info", 100, &PSMpositionNode::getInfo, this); 
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",10);
//    posePublisher_  = nh.advertise<geometry_msgs::Pose2D>(poseTopic_, 10);
  pose3DPublisher_  = n.advertise<geometry_msgs::PoseStamped>(pose3DTopic_, 10);
}

PSMpositionNode::~PSMpositionNode()
{
  ROS_INFO("Destroying PSMposition node");
}


//**************************************************************************************************************************getParams()
void PSMpositionNode::getParams()
{

  ros::NodeHandle nh_private("~");

  std::string odometryType;

  // **** wrapper parameters
  
  if (!nh_private.getParam ("world_frame", worldFrame_))
    worldFrame_ = "world";
  if (!nh_private.getParam ("base_frame", baseFrame_))
    baseFrame_ = "base_frame";
  if (!nh_private.getParam ("publish_tf", publishTf_))
    publishTf_ = true;
  if (!nh_private.getParam ("publish_pose", publishPose_))
    publishPose_ = true;
  if (!nh_private.getParam ("odometry_type", odometryType))
    odometryType = "none";

  if (odometryType.compare("none") == 0)
  {
    useTfOdometry_  = false;
    useImuOdometry_ = false;
  }
  else if (odometryType.compare("tf") == 0)
  {
    useTfOdometry_  = true;
    useImuOdometry_ = false;
  }
  else if (odometryType.compare("imu") == 0)
  {
    useTfOdometry_  = false;
    useImuOdometry_ = true;
  }
  else
  {
    ROS_WARN("Unknown value of odometry_type parameter passed to psm_node. \
              Using default value (\"none\")");
    useTfOdometry_  = false;
    useImuOdometry_ = false;
  }
if(smMethod==1){
  // **** PSM parameters

  if (!nh_private.getParam ("min_valid_points", minValidPoints_))
    minValidPoints_ = 200;
  if (!nh_private.getParam ("search_window", searchWindow_))
    searchWindow_ = 10;
  if (!nh_private.getParam ("max_error", maxError_))
    maxError_ = 0.20;
  if (!nh_private.getParam ("max_iterations", maxIterations_))
    maxIterations_ = 20;
  if (!nh_private.getParam ("stop_condition", stopCondition_))
    stopCondition_ = 0.01;
  }
  // **** CSM parameters - comments copied from algos.h (by Andrea Censi)
  if(smMethod ==2){
  // Maximum angular displacement between scans
  if (!nh_private.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  if (!nh_private.getParam ("max_linear_correction", input_.max_linear_correction))
    input_.max_linear_correction = 0.50;

  // Maximum ICP cycle iterations
  if (!nh_private.getParam ("max_iterations", input_.max_iterations))
    input_.max_iterations = 10;

  // A threshold for stopping (m)
  if (!nh_private.getParam ("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = 0.001;

  // A threshold for stopping (rad)
  if (!nh_private.getParam ("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = 0.00872;

  // Maximum distance for a correspondence to be valid
  if (!nh_private.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
    input_.max_correspondence_dist = 0.1;

  // Noise in the scan (m)
  if (!nh_private.getParam ("sigma", input_.sigma))
    input_.sigma = 0.005;

  // Use smart tricks for finding correspondences.
  if (!nh_private.getParam ("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!nh_private.getParam ("restart", input_.restart))
    input_.restart = 0;

  // Restart: Threshold for restarting
  if (!nh_private.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!nh_private.getParam ("restart_dt", input_.restart_dt))
    input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!nh_private.getParam ("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!nh_private.getParam ("clustering_threshold", input_.clustering_threshold))
    input_.clustering_threshold = 0.1;

  // Number of neighbour rays used to estimate the orientation
  if (!nh_private.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = 40;

  // If 0, it's vanilla ICP
  if (!nh_private.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  if (!nh_private.getParam ("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = 1;

  // Discard correspondences based on the angles - threshold angle, in degrees
  if (!nh_private.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = 40.0;

  // Percentage of correspondences to consider: if 0.9,
	// always discard the top 10% of correspondences with more error
  if (!nh_private.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = 0.80;

  // Parameters describing a simple adaptive algorithm for discarding.
	//  1) Order the errors.
	//	2) Choose the percentile according to outliers_adaptive_order.
	//	   (if it is 0.7, get the 70% percentile)
	//	3) Define an adaptive threshold multiplying outliers_adaptive_mult
	//	   with the value of the error at the chosen percentile.
	//	4) Discard correspondences over the threshold.
	//	This is useful to be conservative; yet remove the biggest errors.
  if (!nh_private.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = 0.6;

  if (!nh_private.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = 1.0;

  //If you already have a guess of the solution, you can compute the polar angle
	//	of the points of one scan in the new position. If the polar angle is not a monotone
	//	function of the readings index, it means that the surface is not visible in the 
	//	next position. If it is not visible, then we don't use it for matching.
  if (!nh_private.getParam ("do_visibility_test", input_.do_visibility_test))
    input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!nh_private.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  if (!nh_private.getParam ("do_compute_covariance", input_.do_compute_covariance))
    input_.do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  if (!nh_private.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
    input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the 
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  if (!nh_private.getParam ("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the 
  // correspondence by 1/sigma^2
  if (!nh_private.getParam ("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = 0;
}

}

void PSMpositionNode::getMotion_can(const sensor_msgs::LaserScan& scan,  std::vector <depthPoint> *depthLine)//*********************************************getMotion_can()
{
  ROS_DEBUG("Received scan");
  

  // **** if this is the first scan, initialize and leave the function here

  if (!initialized_can)
  {   
    initialized_can = initializeCan(scan);
    if (initialized_can) ROS_INFO("Matcher initialized");
    return;
  }

  // **** attmempt to match the two scans

  // CSM is used in the following way:
  // The reference scan (prevLDPcan_) always has a pose of 0
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the world frame since the last scan (btTransform change)
  // The computed correction is then propagated using the tf machinery

  prevLDPScan_->odometry[0] = 0;
  prevLDPScan_->odometry[1] = 0;
  prevLDPScan_->odometry[2] = 0;

  prevLDPScan_->estimate[0] = 0;
  prevLDPScan_->estimate[1] = 0;
  prevLDPScan_->estimate[2] = 0;

  prevLDPScan_->true_pose[0] = 0;
  prevLDPScan_->true_pose[1] = 0;
  prevLDPScan_->true_pose[2] = 0;

  btTransform currWorldToBase;
  btTransform change;
  change.setIdentity();

  // what odometry model to use
  if (useTfOdometry_) 
  {
    // get the current position of the base in the world frame
    // if no transofrm is available, we'll use the last known transform

    getCurrentEstimatedPose(currWorldToBase, scan);
    change = laserToBase_ * prevWorldToBase_.inverse() * currWorldToBase * baseToLaser_;
  }
  else if (useImuOdometry_)
  {
    imuMutex_.lock();
    double dTheta = currImuAngle_ - prevImuAngle_;
    prevImuAngle_ = currImuAngle_;
    change.getRotation().setRPY(0.0, 0.0, dTheta);
    imuMutex_.unlock();
  }

  geometry_msgs::Pose2D p;
  tfToPose2D(change, p);
  LDP currLDPScan = rosToLDPScan(scan, p);

  input_.laser_ref  = prevLDPScan_;
  input_.laser_sens = currLDPScan;
  input_.first_guess[0] = 0;
  input_.first_guess[1] = 0;
  input_.first_guess[2] = 0;

  sm_icp(&input_, &output_);

  if (!output_.valid) 
  {
    ROS_WARN("Error in scan matching");
    ld_free(prevLDPScan_);
    prevLDPScan_ = currLDPScan;
    return;
  }

  // **** calculate change in position

  double dx = output_.x[0];
  double dy = output_.x[1];
  double da = output_.x[2]; 

  // change = scan match result for how much laser moved between scans, 
  // in the world frame
  change.setOrigin(btVector3(dx, dy, 0.0));
  btQuaternion q;
  q.setRPY(0, 0, da);
  change.setRotation(q);

  frameP_ = worldFrame_;
  heightLine = ransac(depthLine);
  // **** publish the new estimated pose as a tf
   
  currWorldToBase = prevWorldToBase_ * baseToLaser_ * change * laserToBase_;

  if (publishTf_  ) publishTf  (currWorldToBase, scan.header.stamp);
  if (publishPose_) publishPose(currWorldToBase, scan.header.stamp, frameP_, heightLine);

  // **** swap old and new

  ld_free(prevLDPScan_);
  prevLDPScan_ = currLDPScan;
  prevWorldToBase_ = currWorldToBase;

  // **** timing information - needed for profiling only
  
}

LDP PSMpositionNode::rosToLDPScan(const sensor_msgs::LaserScan& scan, 
                          const geometry_msgs::Pose2D& basePose)//***************************************************************rostoLDPScan()
{
  unsigned int n = scan.ranges.size();
  
  LDP ld = ld_alloc_new(n);
	
  for (int i = 0; i < n; i++)
  {
    ld->readings[i] = scan.ranges[i];

#ifdef USE_PROJECTED_SCANS
    ld->theta[i]    = scan.angles[i];
#else
    ld->theta[i]    = scan.angle_min + (double)i * scan.angle_increment;
#endif
    
    if (scan.ranges[i] == 0 || scan.ranges[i] > scan.range_max)  
      ld->valid[i] = 0;
    else
      ld->valid[i] = 1;
      
    ld->cluster[i]  = -1;
  }

  ld->min_theta = ld->theta[0];
  ld->max_theta = ld->theta[n-1];

  ld->odometry[0] = basePose.x;
  ld->odometry[1] = basePose.y;
  ld->odometry[2] = basePose.theta;

	return ld;
}

bool PSMpositionNode::initializeCan(const sensor_msgs::LaserScan& scan)//*******************************************************************************initializeScan()
{
  laserFrame_ = scan.header.frame_id;

  // **** get base to laser tf

  tf::StampedTransform baseToLaserTf;
  try
  {
   tfListener_.waitForTransform(baseFrame_, scan.header.frame_id, scan.header.stamp, ros::Duration(1.0));
   tfListener_.lookupTransform (baseFrame_, scan.header.frame_id, scan.header.stamp, baseToLaserTf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("ScanMatcherNode: Could get initial laser transform, skipping scan (%s)", ex.what());
    return false;
  }
  baseToLaser_ = baseToLaserTf;
  laserToBase_ = baseToLaser_.inverse();

  // **** create the first pm scan from the laser scan message

  geometry_msgs::Pose2D p;
  p.x = 0;
  p.y = 0;
  p.theta = 0;
  prevLDPScan_ = rosToLDPScan(scan, p);

  double x, y, theta, temp;
  x = baseToLaser_.getOrigin().getX();
  y = baseToLaser_.getOrigin().getY();
  baseToLaser_.getBasis().getRPY(temp, temp, theta);

  input_.laser[0] = x; 
  input_.laser[1] = y; 
  input_.laser[2] = theta; 

  input_.min_reading = scan.range_min;
  input_.max_reading = scan.range_max;

  // **** get the initial worldToBase tf

  getCurrentEstimatedPose(prevWorldToBase_, scan);

  return true;
}


//**************************************************************************************************************************initializeLasermsgs()
bool PSMpositionNode::initializeLasermsgs(const sensor_msgs::PointCloud2& pcloud)
{
  
  if (CamParam_init == true)
  {
  height_ = pcloud.height;
  width_ = pcloud.width;
  begin_ = width_*(height_/2)+1;

  hline.header = pcloud.header;
  hline.header.frame_id  = "hkinectline";	
  hline.ranges.resize(width_);
  hline.time_increment = 0.001; //where can i get this from?
  hline.scan_time = 1/30;
  hline.range_min = 0.1;
  hline.range_max = 5;
  hline.angle_min = min_angle_h;
  hline.angle_max = max_angle_h;
  hline.angle_increment = inc_angle_h;

  vline.header = pcloud.header;
  vline.header.frame_id  = "vkinectline";	
  vline.ranges.resize(height_);
  vline.time_increment = 0.001; //where can i get this from?
  vline.scan_time = 1/30;
  vline.range_min = 0.1;
  vline.range_max = 5000;
  vline.angle_min = min_angle_v;
  vline.angle_max = max_angle_v;
  vline.angle_increment = inc_angle_v;
  return true;
  }
  else
  {
  ROS_WARN("ScanMatcherNode: Couldnt get CameraParameters, skipping scan");
  return false;
  }

}

//**************************************************************************************************************************pointCloudcallback()
void PSMpositionNode::pointCloudcallback(const sensor_msgs::PointCloud2& pcloud)
{

  ROS_DEBUG("Received PointCloud");
  pclCount_++;
  struct timeval start, end;
  gettimeofday(&start, NULL);

  // **** if this is the first scan, initialize and leave the function here

  if (!init_)
  {   
    init_ = initializeLasermsgs(pcloud);
    if (init_) ROS_INFO("Lasermsgs initialized");
    return;
  }
  hline.header = pcloud.header;
  hline.header.frame_id  = "hkinectline";	

  pcl::fromROSMsg (pcloud, cloud);
  std::vector <depthPoint> verticalLine;

  //get the horizontal central line
  for (int i=0; i<width_; i++)
  {
       
    if (isnan(cloud.points[begin_+i].x) || isnan(cloud.points[begin_+i].y) || isnan(cloud.points[begin_+i].z))
    {
      hline.ranges[width_-1-i] = 0;
    }
    else
    {
      hline.ranges[width_-1-i] = sqrt(pow(cloud.points[begin_+i].x,2)+pow(cloud.points[begin_+i].y,2)+pow(cloud.points[begin_+i].z,2));
    }
  }
//ROS_INFO("Distance to wall: %f",hline.ranges[width_/2]);
  //get the vertical central line
  for (int i=0; i<height_; i++)
  {
    
    //for altitude estimation
    XYpoint.x = cloud.points[(i+0.5)*width_].z ;
    XYpoint.y = cloud.points[(i+0.5)*width_].y ;	

    if (isnan(cloud.points[(i+0.5)*width_].x) || isnan(cloud.points[(i+0.5)*width_].y) || isnan(cloud.points[(i+0.5)*width_].z))
    {
      vline.ranges[i] = 0;
    }
    else
    {
      verticalLine.push_back(XYpoint);
      //vline.ranges[i] = sqrt(pow(cloud.points[(i+0.5)*width_].x,2)+pow(cloud.points[(i+0.5)*width_].y,2)+pow(cloud.points[(i+0.5)*width_].z,2));
    }
  }

  if(smMethod==1)getMotion(hline, &verticalLine);
  if(smMethod==2)getMotion_can(hline, &verticalLine);
  gettimeofday(&end, NULL);
  double dur = ((end.tv_sec   * 1000000 + end.tv_usec  ) - 
                (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;
  totalDuration_ += dur;
  double ave = totalDuration_/pclCount_;

  ROS_INFO("dur:\t %.3f ms \t ave:\t %.3f ms", dur, ave);

}

//**************************************************************************************************************************initialize()
bool PSMpositionNode::initialize(const sensor_msgs::LaserScan& scan)
{
  // **** get base to laser tf

  tf::StampedTransform baseToLaserTf;
  try
  {
   tfListener_.waitForTransform(baseFrame_, scan.header.frame_id, scan.header.stamp, ros::Duration(1.0));
   tfListener_.lookupTransform (baseFrame_, scan.header.frame_id, scan.header.stamp, baseToLaserTf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("ScanMatcherNode: Could get initial laser transform, skipping scan (%s)", ex.what());
    return false;
  }
  baseToLaser_ = baseToLaserTf;
  laserToBase_ = baseToLaser_.inverse();

  // **** pass parameters to matcher and initialise

  matcher_.PM_L_POINTS         = scan.ranges.size();

  matcher_.PM_FOV              = (scan.angle_max - scan.angle_min) * 180.0 / M_PI;
  matcher_.PM_MAX_RANGE        = scan.range_max * ROS_TO_PM;

  matcher_.PM_TIME_DELAY       = 0.00;

  matcher_.PM_MIN_VALID_POINTS = minValidPoints_;
  matcher_.PM_SEARCH_WINDOW    = searchWindow_;
  matcher_.PM_MAX_ERROR        = maxError_ * ROS_TO_PM;

  matcher_.PM_MAX_ITER         = maxIterations_;
  matcher_.PM_MAX_ITER_ICP     = maxIterations_;
  matcher_.PM_STOP_COND        = stopCondition_ * ROS_TO_PM;
  matcher_.PM_STOP_COND_ICP    = stopCondition_ * ROS_TO_PM;

  matcher_.pm_init();

  // **** get the initial worldToBase tf

  getCurrentEstimatedPose(prevWorldToBase_, scan);

  // **** create the first pm scan from the laser scan message

  btTransform t;
  t.setIdentity();
  prevPMScan_ = new PMScan(scan.ranges.size());
  rosToPMScan(scan, t, prevPMScan_);
  ROS_INFO("initialized");
  return true;
}

//**************************************************************************************************************************getMotion()
void PSMpositionNode::getMotion(const sensor_msgs::LaserScan& scan,  std::vector <depthPoint> *depthLine)
{
  ROS_DEBUG("Received kinectLine");

  // **** attmempt to match the two scans

  // PM scan matcher is used in the following way:
  // The reference scan (prevPMScan_) always has a pose of 0
  // The new scan (currPMScan) has a pose equal to the movement
  // of the laser in the world frame since the last scan (btTransform change)
  // The computed correction is then propagated using the tf machinery
 if (!initialized_)
  {   
    initialized_ = initialize(scan);
    if (initialized_) ROS_INFO("Matcher Horizontal initialized");
    return;
  }

  prevPMScan_->rx = 0;
  prevPMScan_->ry = 0;
  prevPMScan_->th = 0; 

  btTransform currWorldToBase;
  btTransform change;
  change.setIdentity();

  // what odometry model to use
  if (useTfOdometry_) 
  {
    // get the current position of the base in the world frame
    // if no transofrm is available, we'll use the last known transform

    getCurrentEstimatedPose(currWorldToBase, scan);
    change = laserToBase_ * prevWorldToBase_.inverse() * currWorldToBase * baseToLaser_;
  }
  else if (useImuOdometry_)
  {
    imuMutex_.lock();
    double dTheta = currImuAngle_ - prevImuAngle_;
    prevImuAngle_ = currImuAngle_;
    change.getRotation().setRPY(0.0, 0.0, dTheta);
    imuMutex_.unlock();
  }

  PMScan * currPMScan = new PMScan(scan.ranges.size());

  rosToPMScan(scan, change, currPMScan);


  try
  {         
    matcher_.pm_psm(prevPMScan_, currPMScan);                         
  }
  catch(int err)
  {
    ROS_WARN("Error in scan matching");
    delete prevPMScan_;
    prevPMScan_ = currPMScan;
    return;
  };    

  // **** calculate change in position

  // rotate by -90 degrees, since polar scan matcher assumes different laser frame
  // and scale down by 100
  double dx =  currPMScan->ry / ROS_TO_PM;
  double dy = -currPMScan->rx / ROS_TO_PM;
  double da =  currPMScan->th; 

  // change = scan match result for how much laser moved between scans, 
  // in the world frame
  change.setOrigin(btVector3(dx, dy, 0.0));
  btQuaternion q;
  q.setRPY(0, 0, da);
  change.setRotation(q);

  
  // **** publish the new estimated pose as a tf
   
  currWorldToBase = prevWorldToBase_ * baseToLaser_ * change * laserToBase_;

  heightLine = ransac(depthLine);

  frameP_ = worldFrame_;

  if (publishTf_  ) publishTf  (currWorldToBase, scan.header.stamp);

  if (publishPose_) publishPose(currWorldToBase, scan.header.stamp, frameP_, heightLine);

  // **** swap old and new

  delete prevPMScan_;
  prevPMScan_      = currPMScan;
  prevWorldToBase_ = currWorldToBase;

}

//**************************************************************************************************************************getCurrentEstimatedPose()
void PSMpositionNode::getCurrentEstimatedPose(btTransform& worldToBase, 
                                      const sensor_msgs::LaserScan& scanMsg)
{
  tf::StampedTransform worldToBaseTf;
  try
  {
     tfListener_.lookupTransform (worldFrame_, baseFrame_, scanMsg.header.stamp, worldToBaseTf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - use the pose from our last estimation
    ROS_WARN("Transform unavailable, using last estimated pose (%s)", ex.what());
    worldToBase = prevWorldToBase_;
    return;
  }
  worldToBase = worldToBaseTf;
}

//**************************************************************************************************************************publishPose()
void PSMpositionNode::publishPose(const btTransform& transform, const ros::Time& time, const std::string poseFrame_, double height)
{
  geometry_msgs::Pose2D pose;
  tfToPose2D(transform, pose);
  posePublisher_.publish(pose);

  geometry_msgs::PoseStamped poseStmpd;

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


 		//for trajectory visualization***********************************************************

  
  		  points.header.frame_id = line_strip.header.frame_id = arrow.header.frame_id = poseFrame_;
		  points.header.stamp = line_strip.header.stamp = arrow.header.stamp = time;
		  points.ns = line_strip.ns = arrow.ns = "points_and_lines";
		  points.action = line_strip.action = arrow.action = visualization_msgs::Marker::ADD;
		  points.pose.orientation.w = line_strip.pose.orientation.w = arrow.pose.orientation.w = 1;

		  points.id = 0;
		  line_strip.id = 1;
		  arrow.id = 2;

		  points.type = visualization_msgs::Marker::POINTS;
		  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
		  arrow.type = visualization_msgs::Marker::LINE_LIST;

		  // POINTS markers use x and y scale for width/height respectively
		  points.scale.x = 0.03;
		  points.scale.y = 0.03;

		  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
		  line_strip.scale.x = 0.01;
		  arrow.scale.x = 0.01;
  
		  // Points are green
		  points.color.g = 1.0f;
		  points.color.a = 1.0;

		  // Line strip is blue
		  line_strip.color.b = 1.0;
		  line_strip.color.a = 1.0;

		  // Arrow is red
		  arrow.color.r = 1.0;
		  arrow.color.a = 1.0;

		  // Create the vertices for the points and lines
		  geometry_msgs::Point p;
		  p = poseStmpd.pose.position;

  	 	  if(pclCount_%20==0)
		  {
		    points.points.push_back(p);
		    line_strip.points.push_back(p);

		    // The line list needs two points for each line

		    arrow.points.push_back(p);
		    p.x+=0.2*cos(pose.theta); 
		    p.y+=0.2*sin(pose.theta); 
		    arrow.points.push_back(p);
		    marker_pub.publish(points);
		    marker_pub.publish(line_strip);
		    marker_pub.publish(arrow);
		  }

}

//**************************************************************************************************************************pubishTf()
void PSMpositionNode::publishTf(const btTransform& transform, 
                                  const ros::Time& time)
{
  tf::StampedTransform transformMsg (transform, time, worldFrame_, baseFrame_);
  tfBroadcaster_.sendTransform (transformMsg);
}

//**************************************************************************************************************************rostoPMScan()
void PSMpositionNode::rosToPMScan(const sensor_msgs::LaserScan& scan, 
                          const btTransform& change,
                                PMScan* pmScan)
{
  geometry_msgs::Pose2D pose;
  tfToPose2D(change, pose);

  // FIXME: rotate x & y by 90 degree?

  pmScan->rx = pose.x * ROS_TO_PM;
  pmScan->ry = pose.y * ROS_TO_PM;
  pmScan->th = pose.theta;

  for (int i = 0; i < scan.ranges.size(); ++i)
  {
    if (scan.ranges[i] == 0) 
    {
      pmScan->r[i] = 99999;  // hokuyo uses 0 for out of range reading
    }
    else
    {
      pmScan->r[i] = scan.ranges[i] * ROS_TO_PM;
      pmScan->x[i] = (pmScan->r[i]) * matcher_.pm_co[i];
      pmScan->y[i] = (pmScan->r[i]) * matcher_.pm_si[i];
      pmScan->bad[i] = 0;
    }

    pmScan->bad[i] = 0;
  }

  matcher_.pm_median_filter  (pmScan);
  matcher_.pm_find_far_points(pmScan);
  matcher_.pm_segment_scan   (pmScan);  
}

//**************************************************************************************************************************getInfo()
void PSMpositionNode::getInfo(const sensor_msgs::CameraInfo& caminfo)
{

   if(CamParam_init == false)
   {	
   ROS_INFO("Getting Camera Parameters");
    const boost::array<double, 9ul> K_matrix = caminfo.K;
   fl = K_matrix[0];
   min_angle_h = atan((1 - K_matrix[2])/fl);
   max_angle_h = atan((caminfo.width - K_matrix[2])/fl);
   inc_angle_h = atan(1/fl);
   min_angle_v = atan((1 - K_matrix[2])/fl);
   max_angle_v = atan((caminfo.height - K_matrix[2])/fl);
   inc_angle_v = atan(1/fl);


   CamParam_init = true;
   }
   return;
}

//**************************************************************************************************************************pose2DtoTf()
void PSMpositionNode::pose2DToTf(const geometry_msgs::Pose2D& pose, btTransform& t)
{
  t.setOrigin(btVector3(pose.x, pose.y, 0.0));
  btQuaternion q;
  q.setRPY(0, 0, pose.theta);
  t.setRotation(q);
}

//**************************************************************************************************************************tfToPose2D()
void PSMpositionNode::tfToPose2D(const btTransform& t, geometry_msgs::Pose2D& pose)
{
  btMatrix3x3 m(t.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  pose.x = t.getOrigin().getX();
  pose.y = t.getOrigin().getY();
  pose.theta = yaw;
}

//**************************************************************************************************************************ransac()
double PSMpositionNode::ransac( std::vector <depthPoint> *depthLine)
{

 ROS_INFO("ransac started");
 
  min_inl_dist = 0.1;
  best_error = 1;
  minNumInliers = 50;
  srand ( time(NULL) );
  iterations = 10000;
  max_slope = 0.15;
  depthPoint pointModel;
  parameters temp_parameters; //m, y0, error
 // ROS_INFO("SizeDepthLine: %d", depthLine.size());            
  //inliers is a vector of model points
  if(depthLine->size() > height_/2){
  for(int j=0;j<iterations;j++)
  {
   std::vector <depthPoint> inliers;
    tempNoInliers = 0;
    rpoint_1 = rand() % (depthLine->size());
    rpoint_2 = rand() % (depthLine->size());

    //check that it doesnt select the same point twice
    while (rpoint_1 == rpoint_2){
    rpoint_2 = rand() % (depthLine->size());
    }
    
    //Line that fits the points
    m = (depthLine->at(rpoint_2).y-depthLine->at(rpoint_1).y)/(depthLine->at(rpoint_2).x-depthLine->at(rpoint_1).x);
    y0 = depthLine->at(rpoint_1).y-m*depthLine->at(rpoint_1).x;
 
    m_abs=fabs(m);

    //check if the line can actually be the floor i.e. small slope and y0>0(?)

    if(max_slope < m_abs || y0<0){
    //ROS_INFO("Too steep");    
    continue;}
    //ROS_INFO("slope of test model%f",m);
    //ROS_INFO("y0 of test model%f",y0);
    a = -m;;
    b = 1.0; 
    c = -y0;

    //calculate distance to the line and find inliers
    for(int k = 0; k<depthLine->size();k++)
    {
      if(k != rpoint_1 && k != rpoint_2)
      {
        dist2line = fabs(a*depthLine->at(k).x+b*depthLine->at(k).y+c)/sqrt(a*a+b*b);

  //ROS_INFO("Distance: %f", dist2line);            
        if(dist2line<min_inl_dist)
        {
        tempNoInliers++;

        pointModel.y = depthLine->at(k).y;
        pointModel.x = depthLine->at(k).x;
        inliers.push_back(pointModel);
	}
      }
    }
 //ROS_INFO("Inliers %d",tempNoInliers);    
    if(tempNoInliers > minNumInliers)
    {
//ROS_INFO("slope of test model%f",m);
//ROS_INFO("y0 of test model%f",y0);
      temp_parameters = fitline (&inliers);
 // ROS_INFO("Error: %f", temp_parameters.error);      
      if(temp_parameters.error < best_error)
      {      
	bestLine[0] = temp_parameters.slope;
        bestLine[1] = temp_parameters.y_origin;
        best_error = temp_parameters.error;
 // ROS_INFO("Best Error: %f", best_error);
      }
    }   
  }
 
   a = -bestLine[0];
  b = 1.0;
  c = -bestLine[1];
  
  //ROS_INFO("slope of corrected model%f",a);
  //ROS_INFO("y0 of corrected model%f",c);
  ransacHeight = fabs(c)/sqrt(a*a+b*b);

  if(bestLine[1]>0){
    if(pclCount_<50){
      previousHeight = ransacHeight;
      //ROS_INFO("Height: %f", ransacHeight);
    }
    else{
      if((fabs(previousHeight-ransacHeight)/previousHeight)<0.1){
        previousHeight = ransacHeight;
        //ROS_INFO("Height: %f", ransacHeight);
      }
      else {
        //ROS_INFO("Change in height too big");
        ransacHeight= previousHeight;    
      }
    }
  }
  else
  {
    //ROS_INFO("Wrong calculation, using previous height");
    ransacHeight= previousHeight;  
  }
  

}
return ransacHeight;
//ROS_INFO("ransac finished");  

}

//**************************************************************************************************************************fitline()
parameters PSMpositionNode::fitline(std::vector<depthPoint> *modelptr)
{
   double xprom, yprom;
  double sdx, sdy;
  double m_slope, m_y0;
  double sumY, sumX, sumXY, sumX2, sumY2;
  double m_y, m_x;
  int sizeInliers;
  double r;
  double m_error;
  double d2l;

  sizeInliers =  modelptr->size();
//ROS_INFO("SizeInliers: %d", sizeInliers);  
  sumY=0;
  sumX=0;
  sumY2=0;
  sumX2=0;
  sumXY=0;
  m_error=0;


  for(int p=0;p<sizeInliers;p++)
  {
    m_y = modelptr->at(p).y;
    m_x = modelptr->at(p).x;
    sumY = sumY + m_y;
    sumX = sumX + m_x;
//ROS_INFO("xparticle %f",m_x);
//ROS_INFO("yyparticle %f",m_y);
  }
  yprom = sumY/sizeInliers;
  xprom = sumX/sizeInliers;

  for(int p=0;p<sizeInliers;p++)
  {
    m_y = modelptr->at(p).y;
    m_x = modelptr->at(p).x;
    sumX2 = sumX2+pow((m_x-xprom),2);
    sumY2 = sumY2+pow((m_y-yprom),2);
    sumXY = sumXY+(xprom-m_x)*(yprom-m_y);
  }

  sdx = sqrt((1.0/(static_cast<float>(sizeInliers)-1.0))*sumX2);
  sdy = sqrt((1.0/(static_cast<float>(sizeInliers)-1.0))*sumY2);

 // ROS_INFO("sumY2: %f", sumY2);  
  r = sumXY/((static_cast<float>(sizeInliers)-1.0)*sdx*sdy);

  m_slope = r*(sdy/sdx);
  m_y0 = yprom-m_slope*xprom;

  modelParameters.slope = m_slope;
  //ROS_INFO("%d",testP.slope);  
  modelParameters.y_origin = m_y0;
  
  for(int p=0;p<sizeInliers;p++)
  {
    m_y = modelptr->at(p).y;
    m_x = modelptr->at(p).x;
    d2l= (-m_slope*m_x+m_y-m_y0)/sqrt(m_slope*m_slope+m_y0*m_y0);
if (d2l<0)
d2l=-d2l;
m_error=m_error+d2l;
  }
m_error = m_error/sizeInliers;
  modelParameters.error = m_error;
  return modelParameters;
}


